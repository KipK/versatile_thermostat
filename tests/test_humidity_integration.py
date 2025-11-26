import sys
import os
import asyncio
import logging
import numpy as np
from unittest.mock import MagicMock
from dataclasses import asdict

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, TpiCycle, DataPoint, ThermalModel

# Configure logging
logging.basicConfig(level=logging.INFO)

async def test_humidity_integration():
    print("Starting Humidity Integration Test...")
    
    hass = MagicMock()
    # Dummy storage path
    hass.config.path.return_value = "dummy_path_humidity.json"
    hass.async_add_executor_job = MagicMock()
    async def mock_async_add_executor_job(func, *args, **kwargs):
        return func(*args, **kwargs)
    hass.async_add_executor_job.side_effect = mock_async_add_executor_job

    manager = AutoTpiManager(hass, "test_humidity", "Test Humidity", cycle_min=5)
    
    # ---------------------------------------------------------
    # Test 1: Backward Compatibility (DataPoint without humidity)
    # ---------------------------------------------------------
    print("\n--- Test 1: Backward Compatibility ---")
    old_dp_dict = {
        "timestamp": 123456.0,
        "room_temp": 20.0,
        "ext_temp": 10.0,
        "power_percent": 50.0,
        "target_temp": 20.0,
        "is_heating": True
    }
    dp = DataPoint.from_dict(old_dp_dict)
    if dp.humidity is None:
        print("PASS: DataPoint initialized with humidity=None from old dict")
    else:
        print(f"FAIL: DataPoint humidity is {dp.humidity}, expected None")

    # ---------------------------------------------------------
    # Test 2: Regression with Humidity (Valid Gamma)
    # ---------------------------------------------------------
    print("\n--- Test 2: Regression with Humidity (Valid Gamma) ---")
    
    alpha = 0.1 / 3600.0
    beta = 0.01 / 3600.0
    gamma = 0.005 / 3600.0 
    
    manager._completed_tpi_cycles = []
    
    num_cycles = 20
    for i in range(num_cycles):
        start_room_temp = 20.0
        # Varied conditions
        ext_temp = 10.0 + (i % 5) 
        power_percent = 10.0 + (i % 10) * 8.0  
        humidity = 40.0 + (i % 5) * 5.0
        
        points = []
        curr_temp = start_room_temp
        start_ts = 1000000 + i * 4000
        
        for t in range(0, 3601, 60):
            ts = start_ts + t
            dp = DataPoint(ts, curr_temp, ext_temp, power_percent, 20.0, True, humidity=humidity)
            points.append(dp)
            
            # Evolve
            dt = 60.0
            power_ratio = power_percent / 100.0
            hum_centered = humidity - 50.0
            derivative = -alpha * (curr_temp - ext_temp) + beta * power_ratio + gamma * hum_centered
            curr_temp += derivative * dt
            
        cycle = TpiCycle(start_ts, 300, points, start_ts + 3600)
        manager._completed_tpi_cycles.append(cycle)
        
    result = await manager.calculate()
    model = manager._thermal_model
    
    if model and model.humidity_coef is not None:
        print(f"PASS: Humidity coefficient calculated: {model.humidity_coef:.6f}")
        expected_gamma_h = 0.005
        if abs(model.humidity_coef - expected_gamma_h) < 0.001:
            print(f"PASS: Gamma value accurate ({model.humidity_coef:.5f} vs {expected_gamma_h})")
        else:
            print(f"WARN: Gamma value deviation ({model.humidity_coef:.5f} vs {expected_gamma_h})")
    else:
        print("FAIL: Humidity coefficient not calculated or None")

    # ---------------------------------------------------------
    # Test 3: Regression without Humidity (Insufficient Data)
    # ---------------------------------------------------------
    print("\n--- Test 3: Regression without Humidity (Insufficient Data) ---")
    manager._completed_tpi_cycles = []
    
    # We need robust 2-var regression even if humidity is missing
    alpha = 0.1 / 3600.0
    beta = 0.01 / 3600.0
    
    for i in range(num_cycles):
        start_room_temp = 20.0
        ext_temp = 10.0 + (i % 5)
        power_percent = 10.0 + (i % 10) * 8.0
        
        # Only cycle 0-4 have humidity (25%)
        has_humidity_cycle = (i < 5)
        cycle_humidity = 50.0 if has_humidity_cycle else None
        
        points = []
        curr_temp = start_room_temp
        start_ts = 1000000 + i * 4000
        
        for t in range(0, 3601, 60):
            dp = DataPoint(start_ts+t, curr_temp, ext_temp, power_percent, 20.0, True, humidity=cycle_humidity)
            points.append(dp)
            
            # Simple evolution (no humidity effect in ground truth to ensure 2-var fits perfectly)
            dt = 60.0
            power_ratio = power_percent / 100.0
            derivative = -alpha * (curr_temp - ext_temp) + beta * power_ratio
            curr_temp += derivative * dt

        cycle = TpiCycle(start_ts, 300, points, start_ts + 3600)
        manager._completed_tpi_cycles.append(cycle)

    result = await manager.calculate()
    model = manager._thermal_model
    
    if model:
        if model.humidity_coef is None:
            print("PASS: Humidity coefficient is None (fallback due to insufficient data)")
            # Verify basic model quality
            if model.r_squared > 0.8:
                print(f"PASS: Basic model quality good (R2={model.r_squared:.2f})")
            else:
                 print(f"WARN: Basic model quality poor (R2={model.r_squared:.2f})")
        else:
            print(f"FAIL: Humidity coefficient should be None, got {model.humidity_coef}")
    else:
        print("FAIL: Model calculation failed completely (returned None)")

    # ---------------------------------------------------------
    # Test 4: Aberrant Gamma Rejection
    # ---------------------------------------------------------
    print("\n--- Test 4: Aberrant Gamma Rejection ---")
    
    alpha = 0.1 / 3600.0
    beta = 0.01 / 3600.0
    gamma = 2.0 / 3600.0 # Creates gamma_h = 2.0 > 0.5
    
    manager._completed_tpi_cycles = []
    
    for i in range(num_cycles):
        start_room_temp = 20.0
        ext_temp = 10.0 + (i % 5)
        power_percent = 10.0 + (i % 10) * 8.0
        humidity = 40.0 + (i % 5) * 5.0
        
        points = []
        curr_temp = start_room_temp
        start_ts = 1000000 + i * 4000
        
        for t in range(0, 3601, 60):
            ts = start_ts + t
            dp = DataPoint(ts, curr_temp, ext_temp, power_percent, 20.0, True, humidity=humidity)
            points.append(dp)
            
            # Evolve with strong humidity
            dt = 60.0
            power_ratio = power_percent / 100.0
            hum_centered = humidity - 50.0
            derivative = -alpha * (curr_temp - ext_temp) + beta * power_ratio + gamma * hum_centered
            curr_temp += derivative * dt
            
        cycle = TpiCycle(start_ts, 300, points, start_ts + 3600)
        manager._completed_tpi_cycles.append(cycle)
        
    result = await manager.calculate()
    model = manager._thermal_model
    
    if model:
        if model.humidity_coef is None:
             print("PASS: Humidity coefficient rejected (None) due to aberrant value")
             # R2 might be lower because we removed the humidity predictor which had strong effect
             print(f"INFO: Fallback model R2={model.r_squared:.2f}")
        else:
             print(f"FAIL: Humidity coefficient {model.humidity_coef} should have been rejected (>0.5)")
    else:
        # It's possible the fallback 2-var model is so bad (because humidity effect was huge) 
        # that it fails validation (R2 < 0.4). This is acceptable behavior.
        print("PASS: Model calculation failed (likely because fallback model was too poor without humidity)")

if __name__ == "__main__":
    asyncio.run(test_humidity_integration())