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
    assert autopi.Kp == 0.8  # Default
    assert autopi.Ki == 0.05  # Default

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
        current_temp=20.5,  # 0.5°C above setpoint (negative error)
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    diag_overshoot = autopi.get_diagnostics()
    u_ff_overshoot = diag_overshoot.get("u_ff", 0)

    # Feedforward should be reduced during overshoot
    assert u_ff_overshoot < u_ff_normal, \
        f"u_ff during overshoot ({u_ff_overshoot}) should be less than normal ({u_ff_normal})"
    # At -0.5°C error, u_ff should be 0
    assert u_ff_overshoot == 0, f"u_ff should be 0 at -0.5°C overshoot, got {u_ff_overshoot}"


def test_overshoot_unwinding():
    """Test that integral is reduced when error changes sign (overshoot)."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )

    # Build up integral with positive error
    for _ in range(10):
        autopi.calculate(
            target_temp=20,
            current_temp=18,  # Positive error
            ext_current_temp=5,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
    
    integral_before = autopi.integral
    assert integral_before > 0, "Integral should have accumulated"

    # Now simulate overshoot (error goes negative)
    autopi.calculate(
        target_temp=20,
        current_temp=20.5,  # Negative error
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    integral_after = autopi.integral
    # Integral should be significantly reduced (0.5 * 0.9 = 0.45x)
    assert integral_after < integral_before * 0.6, \
        f"Integral should be reduced on overshoot: before={integral_before}, after={integral_after}"


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
        name="TestAutoPI"
    )
    autopi.Kp = 1.0
    autopi.Ki = 0.1
    autopi.integral = 0
    
    # High error causing saturation
    for _ in range(5):
        autopi.calculate(
            target_temp=25,
            current_temp=20,  # error = 5
            ext_current_temp=0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
    
    i1 = autopi.integral
    
    # Continue with high error
    autopi.calculate(
        target_temp=25,
        current_temp=20,
        ext_current_temp=0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    i2 = autopi.integral
    
    # Integral should not grow excessively due to anti-windup
    # (saturation reduces integral by 0.9 each time)
    assert i2 < i1 * 1.5, f"Integral should be limited: i1={i1}, i2={i2}"
