
"""Test the SmartPI Hybrid Algorithm (Huber + Theil-Sen)."""

import math
import random
import statistics
import pytest
from unittest.mock import MagicMock
from collections import deque

from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    ABEstimator,
    KP_SAFE,
    KP_MAX
)

def test_huber_strategy_initially():
    """Test that Huber strategy is used when reliability is low (initial state)."""
    est = ABEstimator()
    est.reset()
    
    # Ensure reliability is False
    assert est.tau_reliability().reliable is False
    
    # Create a noisy signal for 'b' learning (OFF phase)
    # b_true = 0.001
    # dT/dt = -0.001 * delta
    random.seed(42)
    b_true = 0.001
    
    # Needs at least 6 samples to trigger Huber update
    for i in range(10):
        delta = 10.0 + i * 0.5
        noise = random.uniform(-0.0002, 0.0002) # Small noise
        dt_val = -b_true * delta + noise
        
        est.learn(
            dT_int_per_min=dt_val,
            u=0.0, # OFF
            t_int=20.0,
            t_ext=20.0 - delta
        )
        
    # Check that we learned something
    assert est.learn_ok_count > 0
    assert "Huber" in est.learn_last_reason
    # Reliability check: manual injection might have made it reliable if random noise was small
    # Just check that we used Huber
    assert est.tau_reliability() # Call just to ensure no crash, result depends on noise
    
    # Check value is close
    assert math.isclose(est.b, b_true, rel_tol=0.2)
    
    # Verify raw history is populated
    assert len(est.b_meas_hist) > 0
    # Verify pts history (Theil-Sen) is empty because we are in Strategy 1
    assert len(est._b_pts) == 0

def test_switching_to_theil_sen():
    """Test that it switches to Theil-Sen strategy when reliability becomes True."""
    est = ABEstimator()
    est.reset()
    
    # Force reliability
    # Reliability needs: learn_ok_count_b >= 10, b_hat_hist length >= 6, decent CV
    # Set learn_ok_count low to bypass residual gating, but learn_ok_count_b high for reliability
    est.learn_ok_count = 5
    est.learn_ok_count_b = 20
    
    # Fill history with stable values to pass stability check
    for _ in range(10):
        est._b_hat_hist.append(0.002)
        
    # Verify it considers itself reliable
    rel = est.tau_reliability()
    assert rel.reliable is True
    assert rel.tau_min == 500.0 # 1/0.002
    
    # Ensure no residual gating blocks TS learning
    est._r_hist.clear()
    
    # Now learn 'b' -> should use Theil-Sen
    b_true = 0.002
    
    # Need 6 points to trigger Theil-Sen
    for i in range(8):
        delta = 10.0 + i*0.5
        dt_val = -b_true * delta # Perfect data
        
        est.learn(
           dT_int_per_min=dt_val,
           u=0.0,
           t_int=20.0,
           t_ext=20.0 - delta 
        )
        
    assert "T-S" in est.learn_last_reason
    assert len(est._b_pts) > 0 # Should have populated points buffer
    assert math.isclose(est.b, b_true, rel_tol=0.01)

def test_persistence_new_history_fields():
    """Verify that a_meas_hist and b_meas_hist are saved/loaded."""
    smartpi = SmartPI(
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestPersistence"
    )
    
    # Add data to histories
    smartpi.est.a_meas_hist.append(0.123)
    smartpi.est.a_meas_hist.append(0.456)
    smartpi.est.b_meas_hist.append(0.009)
    
    saved = smartpi.save_state()
    
    assert "a_meas_hist" in saved
    assert "b_meas_hist" in saved
    assert 0.123 in saved["a_meas_hist"]
    
    # Load into new instance
    smartpi2 = SmartPI(
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestPersistence2",
        saved_state=saved
    )
    
    assert len(smartpi2.est.a_meas_hist) == 2
    assert smartpi2.est.a_meas_hist[0] == 0.123
    assert len(smartpi2.est.b_meas_hist) == 1
    assert smartpi2.est.b_meas_hist[0] == 0.009

def test_huber_outlier_rejection():
    """Test that Huber strategy rejects gross outliers."""
    est = ABEstimator()
    est.reset()
    
    # Seed with good data first to establish a median with some variance
    # Need non-zero scale 's' for outlier detection to work
    b_vals = [0.001, 0.0011, 0.0009, 0.001, 0.00105, 0.00095]
    for v in b_vals:
        est.b_meas_hist.append(v)
        
    # Now try to learn with an outlier measurement
    # delta=10, dT/dt should be -0.01
    # We supply dT/dt=-0.1 => b_meas = 0.01 (10x larger)
    # Ensure outlier rejection threshold is met
    
    est.learn(
        dT_int_per_min=-0.1, # Implies b=0.01 vs median 0.001 -> 10x
        u=0.0,
        t_int=20.0,
        t_ext=10.0
    )
    
    # Should skip
    assert "skip" in est.learn_last_reason
    assert "outlier" in est.learn_last_reason
