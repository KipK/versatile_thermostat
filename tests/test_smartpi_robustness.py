"""Test the ABEstimator estimator for SmartPI."""

from custom_components.versatile_thermostat.smartpi_algorithm import ABEstimator, SmartPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT


def test_rls_convergence_b():
    """Test that ABEstimator converges to correct b (cooling phase)."""
    est = ABEstimator()
    
    # Simulate cooling: dT = -b * (T_int - T_ext)
    # With T_int=20, T_ext=10, delta=10
    # If true_b = 0.002, then dT = -0.002 * 10 = -0.02 °C/min
    
    for _ in range(50):
        # No heating (u=0)
        est.learn(dT_int_per_min=-0.02, u=0.0, t_int=20.0, t_ext=10.0)
    
    # Should converge close to true_b
    assert 0.0015 < est.b < 0.0030, f"b should be ~0.002, got {est.b}"


def test_rls_convergence_a():
    """Test that ABEstimator converges to correct a (heating phase)."""
    est = ABEstimator()
    
    # Simulate heating: dT = a * u - b * (T_int - T_ext)
    # With u=1, T_int=20, T_ext=0 (delta=20), if true_a = 0.01 and est.b=0.001 (init)
    # then dT = 0.01 * 1 - 0.001 * 20 = 0.01 - 0.02 = -0.01 °C/min
    
    for _ in range(50):
        # Heating with losses
        est.learn(dT_int_per_min=-0.01, u=1.0, t_int=20.0, t_ext=0.0)
    
    # Should converge close to true_a
    assert 0.008 < est.a < 0.012, f"a should be ~0.01, got {est.a}"


def test_smartpi_gain_adaptation():
    """Test that Kp/Ki adapt based on thermal time constant."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI"
    )
    
    # Set a low b (long time constant)
    smartpi.est.b = 0.001  # tau = 1000 min
    # We need to ensure we have enough samples for reliability
    # Both learn_ok_count and learn_ok_count_b must be sufficient
    smartpi.est.learn_ok_count = 20
    smartpi.est.learn_ok_count_b = 20
    
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    kp_slow = smartpi.Kp
    
    # Set a high b (short time constant)
    smartpi.est.b = 0.01  # tau = 100 min

    # Reset the rate-limiting timestamp to allow immediate recalculation
    # Without this, the second calculate() call is skipped due to dt < 3s
    smartpi._last_calculate_time = None
    
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    kp_fast = smartpi.Kp
    
    # Fast system should have lower Kp (it responds quickly)
    # Based on formula: Kp = 0.35 + 0.9 * sqrt(tau/200)
    assert kp_fast < kp_slow, f"Kp_fast ({kp_fast}) should be < Kp_slow ({kp_slow})"


def test_smartpi_consistency_check():
    """Test that consistency check rejects solar gain outliers but accepts drift."""
    est = ABEstimator()

    # 1. Warm-up phase (need CONSISTENCY_MIN_SAMPLES, default 10)
    # Train 'b' to a stable value of 0.002
    # dT = -b * delta. With delta=10, dT should be -0.02
    for _ in range(50):
        est.learn(dT_int_per_min=-0.02, u=0.0, t_int=20.0, t_ext=10.0)

    assert est.learn_ok_count_b >= 10
    stable_b = est.b
    assert 0.0019 < stable_b < 0.0021

    # 2. Solar Event (Outlier)
    # Sun hits: effective losses drop to near zero or even negative (heating)
    # Let's say effective b becomes 0.0001 (20x smaller)
    # dT = -0.0001 * 10 = -0.001
    # This implies b_meas = 0.0001
    # Current b is ~0.002. Diff is 0.0019, which is ~95% of b.
    # Threshold is 50%. So this should be REJECTED.
    
    prev_skip_count = est.learn_skip_count
    est.learn(dT_int_per_min=-0.001, u=0.0, t_int=20.0, t_ext=10.0)
    
    assert est.learn_skip_count == prev_skip_count + 1
    assert "consistency" in est.learn_last_reason
    assert est.b == stable_b, "b should not change on outlier"

    # 3. Small Drift (Acceptable)
    # Insulation improves slightly, b drops by 10% (to 0.0018)
    # dT = -0.0018 * 10 = -0.018
    # This implies b_meas = 0.0018. Diff is 0.0002.
    # 0.0002 / 0.002 = 10% change. << 50%. Should be ACCEPTED.
    est.learn(dT_int_per_min=-0.018, u=0.0, t_int=20.0, t_ext=10.0)

    assert est.learn_skip_count == prev_skip_count + 1, "Should not skip valid drift"
    assert "update:b" in est.learn_last_reason
    # NOTE: We do not assert est.b < stable_b here because the median filter (size 30)
    # prevents a single sample from moving the estimate. The fact that it was NOT skipped
    # (verified above) is sufficient to prove the consistency check allowed it.
