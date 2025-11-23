import sys
import os
import asyncio
from datetime import datetime
from unittest.mock import MagicMock, patch

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, HVACAction, DataPoint

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
    manager = AutoTpiManager(hass, "test_vtherm", cycle_min=5)
    await manager.start_learning()
    
    print(f"Initial state: Cycles={manager.heating_cycles_count}, Points={manager.data_points}")
    
    # Test Cycle Detection
    # 1. Start Heating
    print("\nTesting Cycle Detection...")
    # Initialize previous state to ensure transition detection works
    manager._last_hvac_action = HVACAction.IDLE
    
    await manager.update(20.0, 10.0, 100.0, 21.0, HVACAction.HEATING)
    if manager._current_cycle_start:
        print("PASS: Cycle start detected")
    else:
        print("FAIL: Cycle start NOT detected")
        
    # 2. Continue Heating (simulate time passing)
    if manager._current_cycle_start:
        manager._current_cycle_start = datetime.now().replace(minute=datetime.now().minute - 10) # 10 mins ago
    
    await manager.update(20.5, 10.0, 100.0, 21.0, HVACAction.HEATING)
    
    # 3. Stop Heating
    await manager.update(21.0, 10.0, 0.0, 21.0, HVACAction.IDLE)
    
    if manager.heating_cycles_count == 1:
        print(f"PASS: Cycle detected (Duration: {manager._heating_cycles[0]['duration']:.1f}s)")
    else:
        print(f"FAIL: Cycle NOT detected (Count: {manager.heating_cycles_count})")

    # Test Calculation Logic (Mock Data)
    print("\nTesting Calculation Logic...")
    
    # Generate synthetic data for a simple model: dT/dt = -0.1*(Tr-Text) + 0.5*Power
    # Alpha = 0.1 (1/h), Beta = 0.5 (K/h)
    # Time constant = 10 hours
    # k_ext should be alpha/beta = 0.1/0.5 = 0.2
    # k_int should be 1/(beta * 0.5) = 1/(0.5*0.5) = 4.0 -> clipped to 0.4? No, wait.
    # desired_response_time = 0.5 hours.
    # k_int = 1 / (0.5 * 0.5) = 4.0. Clipped to 0.4.
    
    manager._data.clear()
    manager._heating_cycles = [{'start': '2023-01-01', 'duration': 1000}] * 15 # Fake cycles
    
    start_time = datetime.now().timestamp() - 100000
    
    for i in range(200):
        t = start_time + i * 300 # 5 min steps
        tr = 20.0
        text = 10.0 # Diff = 10
        power = 50.0 # 0.5
        target = 20.0
        
        # Expected derivative (approx)
        # dT/dt = -0.1 * 10 + 0.5 * 0.5 = -1 + 0.25 = -0.75 K/h
        
        # We need to manually set derivative because update() calculates it from temp changes
        # But here we just inject points with pre-calculated derivatives for calculate()
        
        point = DataPoint(
            timestamp=t,
            room_temp=tr,
            ext_temp=text,
            power_percent=power,
            target_temp=target,
            is_heating=True,
            temp_derivative=-0.75 # Perfect derivative
        )
        manager._data.append(point)
        
    params = await manager.calculate()
    
    if params:
        print("Calculation Result:", params)
        k_ext = params.get("tpi_coef_ext")
        k_int = params.get("tpi_coef_int")
        
        # Alpha ~ 0.1, Beta ~ 0.5
        # k_ext = 0.1 / 0.5 = 0.2
        # k_int = 1 / (0.5 * 0.5) = 4.0 -> clipped to 0.4
        
        if 0.15 <= k_ext <= 0.25:
            print(f"PASS: k_ext {k_ext} is close to expected 0.2")
        else:
            print(f"FAIL: k_ext {k_ext} is NOT close to expected 0.2")
            
        if k_int == 0.4:
             print(f"PASS: k_int {k_int} is clipped to 0.4 as expected (calculated ~4.0)")
        else:
             print(f"FAIL: k_int {k_int} is unexpected")
             
    else:
        print("FAIL: Calculation returned None")

if __name__ == "__main__":
    asyncio.run(test_auto_tpi_logic())
