import sys
import os
import asyncio
import logging
from datetime import datetime, timedelta
from unittest.mock import MagicMock
import numpy as np

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, TpiCycle, DataPoint

# Configure logging
logging.basicConfig(level=logging.INFO)

async def test_auto_tpi_logic():
    print("Starting Auto TPI Logic Test...")
    
    # Mock Home Assistant
    hass = MagicMock()
    hass.config.path.return_value = "test_auto_tpi.json"
    
    # Fix for await hass.async_add_executor_job
    async def mock_async_add_executor_job(func, *args, **kwargs):
        return func(*args, **kwargs)
    
    hass.async_add_executor_job.side_effect = mock_async_add_executor_job
    
    # Initialize Manager
    manager = AutoTpiManager(hass, "test_vtherm", "Test VTherm", cycle_min=5)
    await manager.start_learning()
    
    print(f"Initial state: Cycles={manager.heating_cycles_count}, Points={manager.data_points}")
    
    # Test Cycle Detection
    # 1. Start Heating (via mode change)
    print("\nTesting Cycle Detection...")
    
    await manager.on_thermostat_mode_changed("HEAT", 21.0)
    
    if manager._current_tpi_cycle:
        print("PASS: Cycle start detected")
    else:
        print("FAIL: Cycle start NOT detected")
        
    # 2. Add points
    start_time = datetime.now().timestamp()
    await manager.update(20.0, 10.0, 100.0, 21.0, "heating")
    
    # Simulate time passing for duration
    if manager._current_tpi_cycle:
        manager._current_tpi_cycle.start_time = start_time - 3600 # 1 hour ago
        # Add a point 1 hour later
        point = DataPoint(
            timestamp=start_time, 
            room_temp=22.0, 
            ext_temp=10.0, 
            power_percent=100.0, 
            target_temp=21.0, 
            is_heating=True
        )
        manager._current_tpi_cycle.data_points.append(point)
    
    # 3. Stop Heating (via mode change)
    await manager.on_thermostat_mode_changed("OFF", 21.0)
    
    if manager.heating_cycles_count == 1:
        cycle = manager._completed_tpi_cycles[0]
        duration = cycle.end_time - cycle.start_time
        print(f"PASS: Cycle detected (Duration: {duration:.1f}s)")
    else:
        print(f"FAIL: Cycle NOT detected (Count: {manager.heating_cycles_count})")

    # Test Calculation Logic (Mock Data)
    print("\nTesting Calculation Logic...")
    
    # Generate synthetic cycles
    # Model: dT/dt = -alpha*(Tr-Text) + beta*Power
    # Let's say alpha = 0.1, beta = 20.0 (degrees gained per hour at 100% power... high but illustrative)
    
    manager._completed_tpi_cycles.clear()
    
    alpha = 0.1
    beta = 5.0 # K/h at 100% power
    
    import random
    
    for i in range(50):
        duration_h = 1.0 # 1 hour cycles
        avg_room = 20.0
        # Vary external temp and power to provide variance for regression
        avg_ext = 10.0 + random.uniform(-5, 5)
        avg_power = 0.5 + random.uniform(-0.3, 0.3)
        avg_power = max(0.1, min(1.0, avg_power))
        
        # Expected dT/dt = -alpha * (Tr-Text) + beta * Power
        expected_derivative = -alpha * (avg_room - avg_ext) + beta * avg_power
        
        # Add some noise
        # expected_derivative += random.uniform(-0.05, 0.05)
        
        expected_evolution = expected_derivative * duration_h
        
        # Create a synthetic cycle
        start = datetime.now().timestamp() - (i * 7200)
        end = start + (duration_h * 3600)
        
        p1 = DataPoint(timestamp=start, room_temp=avg_room - (expected_evolution / 2), ext_temp=avg_ext, power_percent=avg_power*100, target_temp=21.0, is_heating=True)
        p2 = DataPoint(timestamp=end, room_temp=avg_room + (expected_evolution / 2), ext_temp=avg_ext, power_percent=avg_power*100, target_temp=21.0, is_heating=True)
        
        cycle = TpiCycle(
            start_time=start,
            duration_target=300,
            data_points=[p1, p2],
            end_time=end
        )
        manager._completed_tpi_cycles.append(cycle)
        
    params = await manager.calculate()
    
    if params:
        print("Calculation Result:", params)
        k_ext = params.get("tpi_coef_ext")
        k_int = params.get("tpi_coef_int")
        
        # Expected:
        # alpha = 0.1, beta = 5.0
        # k_ext = alpha / beta = 0.1 / 5.0 = 0.02
        
        if 0.015 <= k_ext <= 0.025:
            print(f"PASS: k_ext {k_ext} is close to expected 0.02")
        else:
            print(f"FAIL: k_ext {k_ext} is NOT close to expected 0.02 (alpha={alpha}, beta={beta})")
            
    else:
        print("FAIL: Calculation returned None")

if __name__ == "__main__":
    asyncio.run(test_auto_tpi_logic())
