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

async def test_adaptive_tpi_coefficients():
    print("Starting Adaptive TPI Coefficients Test...")
    
    hass = MagicMock()
    # Dummy storage path
    hass.config.path.return_value = "dummy_path.json"
    hass.async_add_executor_job = MagicMock()
    async def mock_async_add_executor_job(func, *args, **kwargs):
        return func(*args, **kwargs)
    hass.async_add_executor_job.side_effect = mock_async_add_executor_job

    manager = AutoTpiManager(hass, "test_adaptive", cycle_min=5)
    
    num_cycles = 20
    # beta in K/s for 100% power (0-1 range for power in formula?)
    # In calculate(), power is normalized 0-1.
    # So if I use power=50, it is 0.5.
    # beta_target is K/s for Power=1.0.
    beta_target_ks = 0.01 
    
    # Test cases: Tau = 0.5h, 2h, 6h, 12h
    test_cases = [0.5, 1.5, 2.0, 6.0, 12.0]
    
    for tau_target in test_cases:
        print(f"\n--- Testing Tau = {tau_target} hours ---")
        alpha_target_ks = 1.0 / (tau_target * 3600.0)
        
        manager._completed_tpi_cycles = []
        
        for i in range(num_cycles):
            # Conditions
            start_room_temp = 20.0 + (i % 5) * 0.5
            ext_temp = 10.0 + (i % 7) * 1.0
            power_percent = 20.0 + (i % 10) * 8.0 
            
            # Generate points using Euler integration
            # Cycle duration 1h (3600s)
            # Step 60s
            points = []
            curr_temp = start_room_temp
            start_ts = 1000000 + i * 4000
            
            for t in range(0, 3601, 60): # 0 to 3600
                ts = start_ts + t
                
                # Create point
                dp = DataPoint(ts, curr_temp, ext_temp, power_percent, 20.0, True)
                points.append(dp)
                
                # Evolve temp
                # dT/dt = -alpha*(T - Text) + beta*Power(0-1)
                power_ratio = power_percent / 100.0
                dt = 60.0
                derivative = -alpha_target_ks * (curr_temp - ext_temp) + beta_target_ks * power_ratio
                curr_temp += derivative * dt
                
            cycle = TpiCycle(start_ts, 300, points, start_ts + 3600)
            manager._completed_tpi_cycles.append(cycle)
            
        # Run calculation
        result = await manager.calculate()
        
        if not result:
            print("FAIL: Calculation returned None (likely invalid coefficients)")
            continue
            
        k_int = result.get("tpi_coef_int")
        k_ext = result.get("tpi_coef_ext")
        tau_calc = result.get("time_constant_hours")
        desired_resp = result.get("desired_response_hours")
        
        print(f"Calculated: Tau={tau_calc}h, k_int={k_int}, k_ext={k_ext}, DesiredResp={desired_resp}h")
        
        # 1. Check Tau accuracy
        # With better generation, error should be small
        if abs(tau_calc - tau_target) > 0.1 * tau_target: 
             print(f"WARN: Tau calculated {tau_calc} differs from target {tau_target}")
        else:
             print(f"PASS: Tau accurate ({tau_calc} vs {tau_target})")
        
        # 2. Check Desired Response
        expected_resp = max(0.5, tau_calc * 0.1)
        if abs(desired_resp - expected_resp) > 0.05:
            print(f"FAIL: Desired response {desired_resp} != expected {expected_resp}")
        else:
            print(f"PASS: Desired response correct ({desired_resp})")
            
        # 3. Check consistency between k_ext and k_int
        
        # Unboost k_int
        k_int_unboosted = k_int
        if tau_calc < 2.0:
            k_int_unboosted /= 1.2
            
        # Derived unclipped k_ext from k_int
        derived_k_ext_raw = (k_int_unboosted * desired_resp) / tau_calc
        
        # Check if k_ext matches the clipped version
        expected_k_ext = min(0.20, max(0.01, derived_k_ext_raw))
        
        if abs(k_ext - expected_k_ext) < 0.02:
            print(f"PASS: k_ext consistent with k_int (Got {k_ext}, expected ~{expected_k_ext:.3f})")
            if tau_calc < 2.0:
                print("      (Boost accounted for)")
        else:
            print(f"FAIL: k_ext inconsistent. Got {k_ext}, expected ~{expected_k_ext:.3f} (raw {derived_k_ext_raw:.4f})")

if __name__ == "__main__":
    asyncio.run(test_adaptive_tpi_coefficients())