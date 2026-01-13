"""Test the SmartPI algorithm."""

import logging
import pytest
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.smartpi_algorithm import (
    SmartPI,
    ABEstimator,
    KP_SAFE,
    KI_SAFE,
    KI_SAFE,
    KP_MAX
)
import math
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT


def test_smartpi_instantiation():
    """Test instantiation of SmartPI."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=30,
        minimal_deactivation_delay=10,
        name="TestSmartPI"
    )
    assert smartpi
    assert smartpi.Kp == KP_SAFE  # Safe default
    assert smartpi.Ki == KI_SAFE  # Safe default


def test_smartpi_calculation():
    """Test basic calculation."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI"
    )
    
    # Initial state
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Check if calculation produces a value
    assert smartpi.on_percent >= 0
    assert smartpi.on_percent <= 1
    
    # Verify diagnostics
    diag = smartpi.get_diagnostics()
    assert "Kp" in diag
    assert "Ki" in diag
    assert "u_ff" in diag
    assert "tau_reliable" in diag
    assert "learn_ok_count" in diag
    assert "learn_last_reason" in diag


def test_conditional_integration_saturation_high():
    """Test that integration is skipped when saturated high and error is positive."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI",
        # Set 2-DOF weight to 1.0 so e_p = e for predictable behavior
        setpoint_weight_b=1.0,
        # Disable near-band scheduling
        near_band_deg=0.0,
    )
    
    # Set up estimator to produce high FF that saturates
    smartpi.est.a = 0.01
    smartpi.est.b = 0.02  # k_ff = b/a = 2.0
    smartpi.est.learn_ok_count = 50  # Enable full FF
    smartpi._cycles_since_reset = 10  # Enable FF warmup
    smartpi._tau_reliable = True
    
    smartpi.integral = 10.0
    smartpi.u_prev = 1.0  # Already at max
    
    integral_before = smartpi.integral
    
    # Positive error (need more heat) with high FF that causes saturation
    smartpi.calculate(
        target_temp=25,
        current_temp=20,  # error = 5 (positive)
        ext_current_temp=5,  # u_ff = 2.0 * (25-5) = 40, clamped to 1.0
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should NOT increase (I:SKIP due to SAT_HI and e>0)
    assert smartpi.integral == integral_before, \
        f"Integral should not increase when saturated: before={integral_before}, after={smartpi.integral}"
    assert "I:SKIP" in smartpi._last_i_mode


def test_conditional_integration_normal():
    """Test that integration works normally when not saturated."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI",
    )
    
    # Note: calculate() recalculates Kp/Ki from tau_reliability
    # With unreliable tau, safe gains (KP_SAFE=0.55, KI_SAFE=0.01) are used
    smartpi.integral = 0.0
    smartpi.u_prev = 0.0  # Start from 0 to avoid rate limiting issues
    
    # With default a=0.0, b=0.0 (unreliable)
    # u_ff = 0
    # With e=0.2, u_pi = Kp*e + Ki*I = 0.55*0.2 + 0.01*0 = 0.11 < 1.0 = NO_SAT
    smartpi.calculate(
        target_temp=20,
        current_temp=19.8,  # error = 0.2 (above deadband 0.05)
        ext_current_temp=19.8,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should have increased: integral += e * cycle_min = 0.2 * 10 = 2.0
    assert smartpi.integral > 0, f"Integral should increase: {smartpi.integral}"
    assert "I:RUN" in smartpi._last_i_mode


def test_integrator_hold():
    """Test that integrator is frozen when integrator_hold is True."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI",
    )
    
    smartpi.integral = 5.0
    integral_before = smartpi.integral
    
    # Calculate with integrator_hold=True
    smartpi.calculate(
        target_temp=20,
        current_temp=18,  # error = 2
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT,
        integrator_hold=True
    )
    
    # Integral should not change
    assert smartpi.integral == integral_before, \
        f"Integral should be held: before={integral_before}, after={smartpi.integral}"
    assert "I:HOLD" in smartpi._last_i_mode


def test_abestimator_learn_b_off():
    """Test learning of b during OFF periods (u < 0.05)."""
    est = ABEstimator()
    
    # OFF phase: u = 0 (no heating)
    # Model: dT/dt = -b * (T_int - T_ext)
    # With T_int = 20, T_ext = 10, delta = 10
    # If b = 0.001, then dT/dt = -0.001 * 10 = -0.01 (cooling)
    
    est.learn(
        dT_int_per_min=-0.01,  # Cooling
        u=0.0,  # OFF phase (u < 0.05)
        t_int=20.0,
        t_ext=10.0
    )
    
    assert est.learn_ok_count == 1
    assert "update:b(off)" in est.learn_last_reason
    # b should be learned: b = -(-0.01) / 10 = 0.001
    assert est.b > 0


def test_abestimator_learn_a_on():
    """Test learning of a during ON phases (u > 0.20)."""
    est = ABEstimator()
    
    # First, learn b during OFF phase so it's not 0
    est.learn(
        dT_int_per_min=-0.01,
        u=0.0,
        t_int=20.0,
        t_ext=10.0
    )
    assert est.b > 0, "b should be learned first"
    
    # ON phase: u = 0.5 (strong heating)
    # Model: dT/dt = a*u - b*(T_int - T_ext)
    # With b already learned, we can estimate a
    # dT/dt = 0.01 (heating), u = 0.5, b = 0.001, delta = 10
    # a = (dT/dt + b*delta) / u = (0.01 + 0.001*10) / 0.5 = 0.02 / 0.5 = 0.04
    
    est.learn(
        dT_int_per_min=0.01,  # Heating
        u=0.5,  # ON phase (u > 0.20)
        t_int=20.0,
        t_ext=10.0
    )
    
    assert est.learn_ok_count == 2
    assert "update:a(on)" in est.learn_last_reason
    assert est.a > 0


def test_abestimator_skip_gray_zone():
    """Test that learning is skipped in gray zone (0.05 <= u <= 0.20)."""
    est = ABEstimator()
    
    est.learn(
        dT_int_per_min=-0.01,
        u=0.10,  # Gray zone: 0.05 <= u <= 0.20
        t_int=19.0,
        t_ext=10.0
    )
    
    assert est.learn_ok_count == 0
    assert "gray zone" in est.learn_last_reason


def test_abestimator_learn_b_off_phase():
    """Test that b is learned during OFF phase (u < 0.05)."""
    est = ABEstimator()
    
    est.learn(
        dT_int_per_min=-0.01,  # Cooling
        u=0.01,  # OFF phase (u < 0.05)
        t_int=19.0,
        t_ext=10.0
    )
    
    # Should learn b in OFF phase, not skip
    assert est.learn_ok_count == 1
    assert "update:b(off)" in est.learn_last_reason


def test_tau_reliability_not_enough_learns():
    """Test tau reliability when not enough learns."""
    est = ABEstimator()
    est.learn_ok_count = 5  # < 10 needed
    
    tau_info = est.tau_reliability()
    
    assert not tau_info.reliable
    assert tau_info.tau_min is None


def test_tau_reliability_ok():
    """Test tau reliability when conditions are met."""
    est = ABEstimator()
    
    # Inject reliable values
    est.learn_ok_count = 15
    est.b = 0.002  # tau = 500
    
    tau_info = est.tau_reliability()
    
    assert tau_info.reliable
    assert tau_info.tau_min == 500.0


def test_save_and_load_state():
    """Test state persistence."""
    smartpi1 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI1"
    )
    
    # Modify state
    smartpi1.est.a = 0.015
    smartpi1.est.b = 0.003
    smartpi1.est.learn_ok_count = 10
    smartpi1.est._b_hist.append(0.003)
    smartpi1.est._b_hist.append(0.0031)
    smartpi1.integral = 5.0
    smartpi1.u_prev = 0.6
    
    # Save and create new instance with saved state
    saved = smartpi1.save_state()
    
    smartpi2 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI2",
        saved_state=saved
    )
    
    assert smartpi2.est.a == 0.015
    assert smartpi2.est.b == 0.003
    assert smartpi2.est.learn_ok_count == 10
    assert len(smartpi2.est._b_hist) == 2
    assert smartpi2.integral == 5.0
    assert smartpi2.u_prev == 0.6





def test_reset_learning():
    """Test that reset_learning clears all learned state."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI"
    )
    
    # Modify state
    smartpi.est.a = 0.02
    smartpi.est.b = 0.005
    smartpi.est.learn_ok_count = 25
    smartpi.integral = 10.0
    
    # Reset
    smartpi.reset_learning()
    
    # Check defaults restored (uses A_INIT and B_INIT, not 0)
    assert smartpi.est.a == smartpi.est.A_INIT
    assert smartpi.est.b == smartpi.est.B_INIT
    assert smartpi.integral == 0.0
    assert smartpi.est.learn_ok_count == 0


def test_deadband_leak():
    """Test that integral leaks in deadband."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI",
        deadband_c=0.1
    )
    
    smartpi.integral = 10.0
    smartpi.u_prev = 0.5
    integral_before = smartpi.integral
    
    # Error within deadband
    smartpi.calculate(
        target_temp=20,
        current_temp=19.95,  # error = 0.05 < deadband 0.1
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should decay (INTEGRAL_LEAK = 0.995)
    assert smartpi.integral < integral_before, \
        f"Integral should leak: before={integral_before}, after={smartpi.integral}"
    assert "LEAK" in smartpi._last_i_mode


def test_ff_disabled_for_low_a():
    """Test that feed-forward is disabled when a is too low."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI"
    )
    
    # Set a very low value
    smartpi.est.a = 1e-5  # Below threshold of 2e-4
    
    smartpi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,  # Large delta would normally give high FF
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # FF should be 0
    assert smartpi.u_ff == 0.0, f"FF should be 0 when a is too low: {smartpi.u_ff}"


def test_heuristic_gains_reliable_tau():
    """Test that gains are calculated via heuristic when tau is reliable."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI"
    )
    
    # Set up for reliable tau
    # New tau range [10, 800]. Let's pick 500. b = 1/500 = 0.002
    smartpi.est.learn_ok_count = 15
    smartpi.est.b = 0.002
    
    # calculate() triggers tau_reliability check inside
    smartpi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Check if reliable
    assert smartpi._tau_reliable
    
    # Heuristic: Kp = 0.35 + 0.9 * sqrt(500 / 200) = 0.35 + 0.9 * 1.5811 = 0.35 + 1.423 = 1.773
    # Clamped to KP_MAX = 2.5 (no clamping needed here since 1.773 < 2.5)
    
    assert math.isclose(smartpi.Kp, 1.773, rel_tol=1e-3)


def test_safe_gains_unreliable_tau():
    """Test that safe gains are used when tau is unreliable."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI",
        # Disable near-band to avoid Kp reduction
        near_band_deg=0.0,
    )
    
    # No learning done, tau unreliable
    smartpi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert not smartpi._tau_reliable
    
    # KP_SAFE = 0.55
    # Aggressiveness default = 1.0
    # Kp = 0.55 * 1.0 = 0.55
    assert smartpi.Kp == KP_SAFE
