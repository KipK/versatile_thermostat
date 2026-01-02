"""Test the Robust AutoPI algorithm features."""
from collections import deque
import statistics
import pytest
from custom_components.versatile_thermostat.autopi_algorithm import ABEstimator, AutoPI, median_and_mad, robust_huber_location, TauReliability
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_median_and_mad():
    """Test the median and MAD calculation."""
    values = [1, 2, 3, 4, 100] # 100 is outlier
    m, mad = median_and_mad(values)
    assert m == 3
    # deviations from 3: [2, 1, 0, 1, 97] -> sorted: [0, 1, 1, 2, 97]
    # median dev = 1
    assert mad == 1

def test_robust_huber_location():
    """Test robust location with outliers."""
    values = deque([10.0, 10.1, 9.9, 10.0, 50.0]) # 50.0 is outlier
    loc, debug = robust_huber_location(values)
    
    # Should reject 50.0 and be close to 10.0
    assert 9.9 <= loc <= 10.1
    assert debug["scale"] > 0
    
def test_ab_estimator_convergence():
    """Test that estimator converges to a and b."""
    est = ABEstimator()
    # Simulate a system: dT = a*u - b*(T - Text)
    # Let a=0.01, b=0.002
    true_a = 0.01
    true_b = 0.002
    
    # Check convergence for 'b' (cooling u=0)
    for _ in range(20):
        # T_in drops: dT = -0.002 * (20 - 10) = -0.02
        est.learn(u_eff=0.0, t_in_prev=20.0, t_in_now=19.98, t_out_prev=10.0, dt_min=1.0)
    
    assert 0.0015 < est.b < 0.0025
    assert est.learn_ok_count > 0
    
    # Check convergence for 'a' (heating u=1)
    # dT = 0.01*1 - 0.002*(20-10) = 0.01 - 0.02 = -0.01 ? No, usually heating > losses
    # Let Text = 20, T=20 => dT = 0.01
    for _ in range(20):
        est.learn(u_eff=1.0, t_in_prev=20.0, t_in_now=20.01, t_out_prev=20.0, dt_min=1.0)
        
    assert 0.009 < est.a < 0.011

def test_autopi_gain_scheduling():
    """Test that gains are scheduled based on error."""
    api = AutoPI(10, 0, 0, "test")
    # Use parameters where Kp is NOT saturated (Kp < 1.5)
    # Kp ~ tau / (K * (lam + theta))
    # Let a=0.2, b=0.002 -> K=100, tau=500
    # theta=3 (180s)
    # lam_base=6
    # Kp_high ~ 500 / (100 * 9) = 0.55
    api.est.a = 0.2
    api.est.b = 0.002 
    # Make sure reliability is OK
    for _ in range(20): api.est.b_hist.append(0.002)
    api.est.learn_ok_count = 100
    
    # Pre-set Kp to avoid smoothing from 0.8 down to 0.55 confusing things
    api.Kp = 0.55

    # 1. Large error -> High gains
    # Error > 0.40
    api.calculate(20, 15, 0, 0, VThermHvacMode_HEAT)
    kp_high = api.Kp
    
    # 2. Small error -> Low gains
    # Error < 0.10 -> lam increases -> Kp decreases
    # Fill e_filt
    for _ in range(20):
        # Iterate to settle e_filt and smooth Kp
        api.calculate(20, 19.95, 0, 0, VThermHvacMode_HEAT)
        
    kp_low = api.Kp
    
    # Check we observed the decrease
    assert kp_low < kp_high

def test_autopi_anti_windup():
    """Test anti-windup clamping."""
    api = AutoPI(10, 0, 0, "test")
    # Force saturation logic
    api.Kp = 1.0; api.Ki = 0.1
    api.integral = 0
    
    # Heat mode, Error > 0, Output > 1.0 (Saturation High)
    # calculate(target, current)
    # u = Kp*e + Ki*I
    # Let e = 5 => u = 5.
    api.calculate(25, 20, 0, 0, VThermHvacMode_HEAT)
    
    # Should not integrate if saturated and pushing further
    i1 = api.integral
    api.calculate(25, 20, 0, 0, VThermHvacMode_HEAT)
    i2 = api.integral
    
    # With back calculation, it might change slightly, but shouldn't just run away linearly
    # If standard integration: I += 5 = 5.
    # If Conditional Int: I stays 0.
    # If BackCalc: I += gain * (1-5)/Ki = 0.4 * (-4)/0.1 = -16?? 
    assert i2 <= i1 + 0.1 # Should not grow significantly
