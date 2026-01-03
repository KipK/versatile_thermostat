"""Test the AutoPI algorithm."""

import logging
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.autopi_algorithm import (
    AutoPI,
    ABEstimator,
    A_INIT,
    B_INIT,
    KP_SAFE,
    KI_SAFE,
    LEARN_OK_MIN,
)
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
    assert autopi.Kp == KP_SAFE  # Safe default
    assert autopi.Ki == KI_SAFE  # Safe default


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
    assert "tau_reliable" in diag
    assert "learn_ok_count" in diag
    assert "learn_last_reason" in diag


def test_conditional_integration_saturation_high():
    """Test that integration is skipped when saturated high and error is positive."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
    )
    
    # Set up a high integral to force saturation
    autopi.integral = 100.0  # Very high to force saturation
    autopi.Kp = 1.0
    autopi.Ki = 0.1
    autopi.u_prev = 1.0  # Already at max
    
    integral_before = autopi.integral
    
    # Positive error (need more heat) but already saturated at 1.0
    autopi.calculate(
        target_temp=25,
        current_temp=20,  # error = 5 (positive)
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should NOT increase (I:SKIP due to SAT_HI and e>0)
    assert autopi.integral == integral_before, \
        f"Integral should not increase when saturated: before={integral_before}, after={autopi.integral}"
    assert "I:SKIP" in autopi._last_i_mode


def test_conditional_integration_normal():
    """Test that integration works normally when not saturated."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
    )
    
    # Note: calculate() recalculates Kp/Ki from tau_reliability
    # With unreliable tau, safe gains (KP_SAFE=1.0, KI_SAFE=0.003) are used
    autopi.integral = 0.0
    autopi.u_prev = 0.0  # Start from 0 to avoid rate limiting issues
    
    # With default a=0.0005, b=0.001:
    # k_ff = b/a = 2.0
    # To avoid saturation, u_ff must be small, so (target - ext) must be small
    # If ext_current_temp = 19.5, then target - ext = 0.5, u_ff = 2.0 * 0.5 = 1.0 -> still saturates!
    # Let's set ext_current_temp = 19.8, then target - ext = 0.2, u_ff = 0.4
    # With e=1, u_pi = 1.0 * 1 = 1.0, total = 1.4 > 1.0 = SAT_HI -> still saturates!
    # Solution: use very small error so u_pi is small
    # With e=0.2, u_pi = 1.0 * 0.2 = 0.2, u_ff = 0.4, total = 0.6 < 1.0 = NO_SAT
    autopi.calculate(
        target_temp=20,
        current_temp=19.8,  # error = 0.2 (above deadband 0.05)
        ext_current_temp=19.8,  # (target - ext) = 0.2, u_ff = 2 * 0.2 = 0.4
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should have increased: integral += e * cycle_min = 0.2 * 10 = 2.0
    assert autopi.integral > 0, f"Integral should increase: {autopi.integral}"
    assert "I:RUN" in autopi._last_i_mode


def test_integrator_hold():
    """Test that integrator is frozen when integrator_hold is True."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
    )
    
    autopi.integral = 5.0
    integral_before = autopi.integral
    
    # Calculate with integrator_hold=True
    autopi.calculate(
        target_temp=20,
        current_temp=18,  # error = 2
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT,
        integrator_hold=True
    )
    
    # Integral should not change
    assert autopi.integral == integral_before, \
        f"Integral should be held: before={integral_before}, after={autopi.integral}"
    assert "I:HOLD" in autopi._last_i_mode


def test_abestimator_learn_b_off():
    """Test learning of b during OFF periods."""
    est = ABEstimator()
    
    # Simulate cooling phase (u_eff < 0.05)
    # dT/dt = -b * (Tin - Text)
    # With Tin=20, Text=10, delta=10, assume dT=-0.03 over 3 min => dTpm=-0.01
    # b_meas = -(-0.01) / 10 = 0.001
    
    learn_ok, reason, dTpm = est.learn(
        u_eff=0.0,
        t_in_prev=20.0,
        t_in_now=19.97,  # dT = -0.03
        t_out_prev=10.0,
        dt_min=3.0
    )
    
    assert learn_ok, f"Should learn: {reason}"
    assert "update:b(off)" in reason
    assert est.learn_ok_count == 1


def test_abestimator_learn_a_on():
    """Test learning of a during ON periods."""
    est = ABEstimator()
    est.b = 0.001  # Pre-set b to a known value
    
    # Simulate heating phase (u_eff > 0.20)
    # dT/dt = a*u - b*(Tin - Text)
    # With u=0.5, Tin=18, Text=10, delta=8, dT=0.04 over 2 min => dTpm=0.02
    # a_meas = (dTpm + b*delta) / u = (0.02 + 0.001*8) / 0.5 = 0.028/0.5 = 0.056
    
    learn_ok, reason, dTpm = est.learn(
        u_eff=0.5,
        t_in_prev=18.0,
        t_in_now=18.04,  # dT = 0.04
        t_out_prev=10.0,
        dt_min=2.0
    )
    
    assert learn_ok, f"Should learn: {reason}"
    assert "update:a(on)" in reason
    assert est.learn_ok_count == 1


def test_abestimator_gray_zone():
    """Test that learning is skipped in gray zone (0.05 < u < 0.20)."""
    est = ABEstimator()
    
    learn_ok, reason, dTpm = est.learn(
        u_eff=0.10,  # Gray zone
        t_in_prev=19.0,
        t_in_now=19.1,
        t_out_prev=10.0,
        dt_min=5.0
    )
    
    assert not learn_ok
    assert "gray zone" in reason


def test_tau_reliability_not_enough_learns():
    """Test tau reliability when not enough learns."""
    est = ABEstimator()
    est.learn_ok_count = LEARN_OK_MIN - 1
    
    tau_info = est.tau_reliability()
    
    assert not tau_info.reliable
    assert f"learn_ok<{LEARN_OK_MIN}" in tau_info.reason


def test_tau_reliability_ok():
    """Test tau reliability when conditions are met."""
    est = ABEstimator()
    
    # Simulate successful learning with stable b
    for i in range(15):
        # Slight variations in b to simulate real learning
        est.b = 0.001 + 0.00001 * (i % 3)
        est.b_hist.append(est.b)
        est.learn_ok_count += 1
    
    tau_info = est.tau_reliability()
    
    assert tau_info.reliable, f"Should be reliable: {tau_info.reason}"
    assert "tau:OK" in tau_info.reason


def test_save_and_load_state():
    """Test state persistence."""
    autopi1 = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI1"
    )
    
    # Modify state
    autopi1.est.a = 0.015
    autopi1.est.b = 0.003
    autopi1.est.learn_ok_count = 10
    autopi1.est.b_hist.append(0.003)
    autopi1.est.b_hist.append(0.0031)
    autopi1.integral = 5.0
    autopi1.u_prev = 0.6
    
    # Save and create new instance with saved state
    saved = autopi1.save_state()
    
    autopi2 = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI2",
        saved_state=saved
    )
    
    assert autopi2.est.a == 0.015
    assert autopi2.est.b == 0.003
    assert autopi2.est.learn_ok_count == 10
    assert len(autopi2.est.b_hist) == 2
    assert autopi2.integral == 5.0
    assert autopi2.u_prev == 0.6


def test_reset_learning():
    """Test that reset_learning clears all learned state."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Modify state
    autopi.est.a = 0.02
    autopi.est.b = 0.005
    autopi.est.learn_ok_count = 25
    autopi.integral = 10.0
    
    # Reset
    autopi.reset_learning()
    
    # Check defaults restored
    assert autopi.est.a == A_INIT
    assert autopi.est.b == B_INIT
    assert autopi.integral == 0.0
    assert autopi.est.learn_ok_count == 0


def test_deadband_leak():
    """Test that integral leaks in deadband."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI",
        deadband_c=0.1
    )
    
    autopi.integral = 10.0
    autopi.u_prev = 0.5
    integral_before = autopi.integral
    
    # Error within deadband
    autopi.calculate(
        target_temp=20,
        current_temp=19.95,  # error = 0.05 < deadband 0.1
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Integral should decay (INTEGRAL_LEAK = 0.985)
    assert autopi.integral < integral_before, \
        f"Integral should leak: before={integral_before}, after={autopi.integral}"
    assert "LEAK" in autopi._last_i_mode


def test_ff_disabled_for_low_a():
    """Test that feed-forward is disabled when a is too low."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Set a very low value
    autopi.est.a = 1e-5  # Below threshold of 2e-4
    
    autopi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,  # Large delta would normally give high FF
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # FF should be 0
    assert autopi.u_ff == 0.0, f"FF should be 0 when a is too low: {autopi.u_ff}"


def test_heuristic_gains_reliable_tau():
    """Test that gains are calculated via heuristic when tau is reliable."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Set up for reliable tau
    autopi.est.b = 0.002  # tau = 500 min
    for i in range(15):
        autopi.est.b_hist.append(0.002 + 0.00001 * (i % 3))
        autopi.est.learn_ok_count += 1
    
    autopi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Gains should be calculated via heuristic
    # Kp = 0.35 + 0.9 * (tau / 200) = 0.35 + 0.9 * 2.5 = 2.6, clamped to KP_MAX=1.5
    # So Kp should be close to 1.5
    assert autopi._tau_reliable
    assert autopi.Kp > KP_SAFE or autopi.Kp == 1.5  # Should not be safe gains


def test_safe_gains_unreliable_tau():
    """Test that safe gains are used when tau is unreliable."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # No learning done, tau unreliable
    autopi.calculate(
        target_temp=20,
        current_temp=18,
        ext_current_temp=5,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Gains should be safe defaults
    assert not autopi._tau_reliable
    assert autopi.Kp == KP_SAFE
    assert autopi.Ki == KI_SAFE
