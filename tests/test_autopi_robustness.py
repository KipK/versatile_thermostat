"""Test the RLS estimator for AutoPI."""

from custom_components.versatile_thermostat.autopi_algorithm import RLS, AutoPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT


def test_rls_convergence_b():
    """Test that RLS converges to correct b (cooling phase)."""
    rls = RLS()
    
    # Simulate cooling: dT = -b * (T_int - T_ext)
    # With T_int=20, T_ext=10, delta=10
    # If true_b = 0.002, then dT = -0.002 * 10 = -0.02 °C/min
    true_b = 0.002
    
    for _ in range(50):
        # No heating (u=0)
        rls.update(u=0.0, t_int=20.0, t_ext=10.0, dt_int=-0.02)
    
    # Should converge close to true_b
    assert 0.0015 < rls.theta_b < 0.0030, f"b should be ~0.002, got {rls.theta_b}"


def test_rls_convergence_a():
    """Test that RLS converges to correct a (heating phase)."""
    rls = RLS()
    
    # Simulate heating: dT = a * u - b * (T_int - T_ext)
    # With u=1, T_int=T_ext=20 (no losses), if true_a = 0.01
    # then dT = 0.01 * 1 = 0.01 °C/min
    true_a = 0.01
    
    for _ in range(50):
        # Full heating, no temperature difference (no losses)
        rls.update(u=1.0, t_int=20.0, t_ext=20.0, dt_int=0.01)
    
    # Should converge close to true_a
    assert 0.008 < rls.theta_a < 0.012, f"a should be ~0.01, got {rls.theta_a}"


def test_autopi_gain_adaptation():
    """Test that Kp/Ki adapt based on thermal time constant."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Set a low b (long time constant)
    autopi.rls.theta_b = 0.001  # tau = 1000 min
    
    autopi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    kp_slow = autopi.Kp
    ki_slow = autopi.Ki
    
    # Set a high b (short time constant)
    autopi.rls.theta_b = 0.01  # tau = 100 min
    
    autopi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    kp_fast = autopi.Kp
    ki_fast = autopi.Ki
    
    # Fast system should have lower Kp (it responds quickly)
    # Based on formula: Kp = 0.3 + 0.5 * (tau/100)
    # tau_slow = 1000 -> Kp = 0.3 + 5 = 5.3, clamped to 2.0
    # tau_fast = 100 -> Kp = 0.3 + 0.5 = 0.8
    assert kp_fast < kp_slow, f"Kp_fast ({kp_fast}) should be < Kp_slow ({kp_slow})"
