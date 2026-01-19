import logging
import pytest
import time
from unittest.mock import MagicMock
from datetime import datetime, timedelta

from custom_components.versatile_thermostat.prop_algo_smartpi import ABEstimator, SmartPI

def test_theil_sen_slope_robustness():
    """Test Theil-Sen slope estimation with outliers."""
    # 1. Perfect linear: y = 2x + 1
    # x in minutes for expectation, but function takes seconds
    # Let's say we expect 2.0 per minute.
    # So over 1 min (60s), change is 2.0.
    
    # Samples: (0, 0), (60, 0.05), ...
    # Expected slope: 0.05 deg/min = 0.000833 deg/s
    samples = []
    for i in range(6):
        samples.append((i * 60.0, float(i * 0.05)))
        
    slope_min, method, n = ABEstimator.robust_dTdt_per_min(samples)
    assert method == "theil_sen"
    assert n == 6
    assert abs(slope_min - 0.05) < 1e-4
    
    # 2. Add outlier at index 2 (time 120)
    # y should be 0.10. change to 2.0 (Huge spike).
    samples_outlier = list(samples)
    samples_outlier[2] = (120.0, 2.0) 
    
    # Simple start-end for 5 mins (300s): (0.25 - 0)/5 = 0.05.
    # Theil-Sen uses all pairs.
    slope_min_rob, method, n = ABEstimator.robust_dTdt_per_min(samples_outlier)
    assert method == "theil_sen"
    # Should still be close to 0.05
    assert abs(slope_min_rob - 0.05) < 0.02
    

@pytest.mark.asyncio
async def test_integration_robustness_end_outlier():
    """Test integration: End-point outlier ruins simple slope but not robust slope."""
    hass = MagicMock()
    pi = SmartPI(
        hass=hass, 
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="test"
    )
    pi.est.reset()
    
    # Prepare data: 10 mins window
    # True slope: 0.05 C/min (0.5 deg / 10 min)
    # Start: 20.0, End expected: 20.5
    now_ts = time.monotonic()
    start_ts = now_ts - 600.0
    
    # Fill history
    pi.dt_est._tin_history.clear()
    for i in range(10): # 0 to 9 mins
        t = start_ts + i * 60.0
        val = 20.0 + 0.05 * i
        pi.dt_est._tin_history.append((t, val))
        
    # Add outlier at 10 mins (End of cycle)
    # Expected 20.5. Put 22.5 (+2.0 spike)
    t_end = start_ts + 600.0
    val_outlier = 22.5
    pi.dt_est._tin_history.append((t_end, val_outlier))
    
    # Setup state for update_learning
    pi.learn_win_active = True
    pi.learn_win_start_ts = start_ts
    pi.learn_T_int_start = 20.0
    pi.learn_T_ext_start = 0.0 # Delta = 20
    pi.learn_u_int = 0.0
    pi.learn_t_int_s = 0.0
    
    # Call update_learning
    # Simulate a single 10-minute update that completes the window
    
    # Spy on est.learn
    pi.est.learn = MagicMock(wraps=pi.est.learn)
    
    # Call update_learning
    pi.update_learning(
        dt_min=10.0, 
        current_temp=val_outlier, 
        ext_temp=0.0, 
        u_active=0.5
    )
    
    # Verify method used
    assert pi.est.diag_dTdt_method == "theil_sen"
    
    # Verify passed slope
    # Simple slope would use (22.5 - 20) / 10 = 0.25 C/min
    # True slope is 0.05
    # Theil-Sen should be near 0.05
    
    args, kwargs = pi.est.learn.call_args
    dT_passed = kwargs['dT_int_per_min']
    
    # Allow some tolerance
    assert abs(dT_passed - 0.05) < 0.05
    assert dT_passed < 0.15 # Proof it ignored the 0.25 spike
    
    print(f"Passed slope: {dT_passed}")
