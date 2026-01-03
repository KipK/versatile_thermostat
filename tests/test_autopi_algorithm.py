"""Test the AutoPI algorithm."""

import logging
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.autopi_algorithm import AutoPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_autopi_instantiation():
    """Test instantiation of AutoPI."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=30,
        minimal_deactivation_delay=10,
        name="TestAutoPI"
    )
    assert autopi
    assert autopi.Kp == 0.5  # Default
    assert autopi.Ki == 0.02  # Default


def test_autopi_calculation():
    """Test basic calculation."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Initial state
    autopi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Check if calculation produces a value
    assert autopi.on_percent >= 0
    assert autopi.on_percent <= 1
    
    # Verify diagnostics
    diag = autopi.get_diagnostics()
    assert "Kp" in diag
    assert "Ki" in diag
    assert "u_ff" in diag
    assert "effective_Kp" in diag
    assert "effective_Ki" in diag


def test_feedforward_scaling_during_overshoot():
    """Test that feedforward is scaled down during overshoot (temp > setpoint)."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )

    # Set up a known model state for predictable behavior
    autopi.rls.theta_a = 0.02  # Reasonable heating efficiency
    autopi.rls.theta_b = 0.001  # Reasonable losses
    # Reduce covariance to simulate well-learned model
    autopi.rls.P11 = 10.0
    autopi.rls.P22 = 10.0

    # Calculate with normal heating (temp below setpoint)
    autopi.calculate(
        target_temp=20,
        current_temp=19,  # 1°C below setpoint (positive error)
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    diag_normal = autopi.get_diagnostics()
    u_ff_normal = diag_normal.get("u_ff", 0)

    # Now calculate with overshoot (temp above setpoint)
    autopi.calculate(
        target_temp=20,
        current_temp=20.3,  # 0.3°C above setpoint (negative error)
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    diag_overshoot = autopi.get_diagnostics()
    u_ff_overshoot = diag_overshoot.get("u_ff", 0)

    # Feedforward should be reduced during overshoot
    assert u_ff_overshoot < u_ff_normal, \
        f"u_ff during overshoot ({u_ff_overshoot}) should be less than normal ({u_ff_normal})"
    # At -0.3°C error (OVERSHOOT_SCALE_RANGE), u_ff should be 0
    assert u_ff_overshoot == 0, f"u_ff should be 0 at -0.3°C overshoot, got {u_ff_overshoot}"


def test_overshoot_unwinding():
    """Test that integral is reduced when in overshoot."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
        max_step_per_min=1.0  # Allow fast changes to avoid rate limiting issues
    )
    
    # Set up model to minimize feedforward impact
    autopi.rls.theta_a = 0.001
    autopi.rls.theta_b = 0.001
    autopi.rls.P11 = 500.0  # Medium confidence
    autopi.rls.P22 = 500.0

    # Manually set some integral to simulate accumulated error
    autopi.integral = 10.0
    integral_before = autopi.integral
    
    # Now simulate overshoot (error goes negative)
    # This should trigger the proportional decay: integral -= 0.5 * abs(e) * cycle_min
    # With e = -0.5, decay = 0.5 * 0.5 * 10 = 2.5
    autopi.calculate(
        target_temp=20,
        current_temp=20.5,  # Negative error = -0.5
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    integral_after = autopi.integral
    # Integral should be reduced (proportional decay based on overshoot)
    assert integral_after < integral_before, \
        f"Integral should be reduced on overshoot: before={integral_before}, after={integral_after}"
    # Expected: 10.0 - 2.5 = 7.5
    assert integral_after < 8.0, f"Integral should decay significantly, got {integral_after}"


def test_save_and_load_state():
    """Test state persistence."""
    autopi1 = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI1"
    )
    
    # Modify state
    autopi1.rls.theta_a = 0.015
    autopi1.rls.theta_b = 0.003
    autopi1.rls.P11 = 50.0
    autopi1.rls.P22 = 75.0
    autopi1.integral = 5.0
    autopi1.Kp = 1.2
    autopi1.Ki = 0.08
    
    # Save and create new instance with saved state
    saved = autopi1.save_state()
    
    autopi2 = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI2",
        saved_state=saved
    )
    
    assert autopi2.rls.theta_a == 0.015
    assert autopi2.rls.theta_b == 0.003
    assert autopi2.rls.P11 == 50.0
    assert autopi2.rls.P22 == 75.0
    assert autopi2.integral == 5.0
    assert autopi2.Kp == 1.2
    assert autopi2.Ki == 0.08


def test_reset_learning():
    """Test that reset_learning clears all learned state."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Modify state
    autopi.rls.theta_a = 0.02
    autopi.rls.theta_b = 0.005
    autopi.integral = 10.0
    
    # Reset
    autopi.reset_learning()
    
    # Check defaults restored
    assert autopi.rls.theta_a == 0.0005  # A_INIT
    assert autopi.rls.theta_b == 0.0010  # B_INIT
    assert autopi.integral == 0.0


def test_anti_windup():
    """Test anti-windup when output saturates."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
        max_step_per_min=1.0  # Allow fast changes for this test
    )
    autopi.Kp = 1.0
    autopi.Ki = 0.1
    autopi.integral = 0
    
    # High error causing saturation
    for _ in range(10):
        autopi.calculate(
            target_temp=25,
            current_temp=20,  # error = 5
            ext_current_temp=0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
    
    # Integral should be limited by MAX_INTEGRAL (50)
    from custom_components.versatile_thermostat.autopi_algorithm import MAX_INTEGRAL
    assert autopi.integral <= MAX_INTEGRAL, \
        f"Integral should be capped at {MAX_INTEGRAL}, got {autopi.integral}"


def test_gain_scheduling():
    """Test that effective gains are reduced near setpoint."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Set up model for consistent gains
    autopi.rls.theta_a = 0.01
    autopi.rls.theta_b = 0.002
    
    # Calculate with large error (2°C)
    autopi.calculate(
        target_temp=20,
        current_temp=18,  # error = 2
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    diag_large_error = autopi.get_diagnostics()
    
    # Calculate with small error (0.5°C)
    autopi.calculate(
        target_temp=20,
        current_temp=19.5,  # error = 0.5
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    diag_small_error = autopi.get_diagnostics()
    
    # Effective gains should be smaller when error is small
    assert diag_small_error["effective_Kp"] < diag_large_error["effective_Kp"], \
        f"Kp should be reduced near setpoint: {diag_small_error['effective_Kp']} vs {diag_large_error['effective_Kp']}"


def test_simc_tuning():
    """Test that SIMC tuning produces reasonable gains."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Simulate a typical thermal system
    autopi.rls.theta_a = 0.005  # Modest heating efficiency
    autopi.rls.theta_b = 0.002  # tau = 500 min (slow system)
    
    autopi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Gains should be within reasonable bounds
    assert 0.1 <= autopi.Kp <= 2.0, f"Kp out of bounds: {autopi.Kp}"
    assert 0.001 <= autopi.Ki <= 0.15, f"Ki out of bounds: {autopi.Ki}"


def test_adaptive_rate_limiting():
    """Test that rate limiting is stricter near setpoint."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
        max_step_per_min=0.5  # High base rate for testing
    )
    
    # Set up model
    autopi.rls.theta_a = 0.01
    autopi.rls.theta_b = 0.002
    autopi.u_prev = 0.5
    
    # With large error, should allow larger step
    autopi.calculate(
        target_temp=20,
        current_temp=17,  # error = 3
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    u_large_error = autopi.on_percent
    
    # Reset
    autopi.u_prev = 0.5
    
    # With small error, step should be more limited
    autopi.calculate(
        target_temp=20,
        current_temp=19.8,  # error = 0.2
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    u_small_error = autopi.on_percent
    
    # The change from u_prev should be smaller for small error
    delta_large = abs(u_large_error - 0.5)
    delta_small = abs(u_small_error - 0.5)
    
    # Rate limiting is stricter when error < 0.5, so delta should be smaller
    # (unless both saturate at the same limit, which would make them equal)
    assert delta_small <= delta_large, \
        f"Rate limiting should be stricter near setpoint: delta_small={delta_small}, delta_large={delta_large}"
