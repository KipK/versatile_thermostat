
import asyncio
import logging
from datetime import datetime, timedelta
from unittest.mock import MagicMock, AsyncMock

# Configure logging
logging.basicConfig(level=logging.INFO)

# Import the class to test
# Adjust the import path as needed
import sys
import os
sys.path.append(os.getcwd())
from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager

async def reproduce():
    print("Starting reproduction script...")
    
    # Mock Home Assistant
    hass = MagicMock()
    hass.config.path = MagicMock(return_value="test_auto_tpi.json")
    hass.async_add_executor_job = AsyncMock()
    
    # Initialize Manager
    manager = AutoTpiManager(hass, "test_unique_id", "Test Thermostat", cycle_min=5)
    
    # Initial State
    print(f"Initial heating_cycles_count: {manager.heating_cycles_count}")
    assert manager.heating_cycles_count == 0
    assert manager.state.last_learning_status == "startup"
    
    # Cycle 1: Start
    # update(room_temp, ext_temp, power_percent, target_temp, hvac_action)
    await manager.update(19.0, 10.0, 50.0, 20.0, "heating")
    print(f"After Cycle 1 Start heating_cycles_count: {manager.heating_cycles_count}")
    assert manager.heating_cycles_count == 1
    
    # Simulate time passing (learning window passed)
    manager.state.learning_end_date = datetime.now() - timedelta(minutes=1)
    
    # Cycle 2: End of Cycle 1 / Start of Cycle 2
    # Temp increased from 19.0 to 19.5 (Progress > 0)
    # Target 20.0 > Start 19.0 (Target Diff > 0)
    # This should trigger learning
    print("Updating with progress (Learning should occur)...")
    await manager.update(19.5, 10.0, 50.0, 20.0, "heating")
    
    print(f"After Cycle 2 Start heating_cycles_count: {manager.heating_cycles_count}")
    assert manager.heating_cycles_count == 2
    assert manager.state.last_learning_status == "success_indoor"
    
    # Simulate time passing
    manager.state.learning_end_date = datetime.now() - timedelta(minutes=1)
    
    # Cycle 3: End of Cycle 2 / Start of Cycle 3
    # Force NO learning by setting last_power to 100% (condition is 0 < last_power < 100)
    manager.state.last_power = 100.0
    
    print("Updating with last_power=100 (No learning should occur)...")
    await manager.update(19.8, 10.0, 50.0, 20.0, "heating")
    
    print(f"After Cycle 3 Start heating_cycles_count: {manager.heating_cycles_count}")
    assert manager.heating_cycles_count == 3
    assert manager.state.last_learning_status == "power_saturated"
    
    print("Verification successful!")

if __name__ == "__main__":
    asyncio.run(reproduce())
