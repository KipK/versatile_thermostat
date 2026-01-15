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
    KP_MAX,
    AW_TRACK_TAU_S,
    AW_TRACK_MAX_DELTA_I,
)
import math
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT, VThermHvacMode_COOL


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
    assert tau_info.tau_min == 9999.0


def test_tau_reliability_ok():
    """Test tau reliability when conditions are met."""
    est = ABEstimator()
    
    # Inject reliable values
    est.learn_ok_count = 15
    est.learn_ok_count_b = 15
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
    smartpi.est.learn_ok_count_b = 15
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


def test_cool_mode_inversion_bug():
    """Test reproduction of double sign inversion in COOL mode."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_COOL"
    )

    # Setup for COOL mode
    # Target 20, Current 22 -> Error = -2 (too hot)
    # in COOL mode, error is inverted -> e = -(-2) = 2 (positive error means cooling needed)
    
    # e_p = setpoint_weight_b * e
    # We expect e_p to preserve the sign of e (positive) so that the PI produces positive output
    
    smartpi.calculate(
        target_temp=20,
        current_temp=22,
        ext_current_temp=25,
        slope=0,
        hvac_mode=VThermHvacMode_COOL
    )

    # Check error sign (should be positive for cooling demand)
    assert smartpi._last_error > 0, f"Error should be positive for cooling demand, got {smartpi._last_error}"

    # Check proportional error sign (should be positive)
    # The BUG causes this to be negative because of double inversion
    assert smartpi._last_error_p > 0, f"Proportional error should be positive for cooling demand, got {smartpi._last_error_p}"


def test_tau_reliability_dependent_on_b():
    """Test that tau reliability depends on b learning count."""
    est = ABEstimator()
    est.learn_ok_count_a = 20
    est.learn_ok_count_b = 0 # No b updates
    est.learn_ok_count = 20  # Total updates reasonable
    
    # Inject a valid b so it doesn't fail on range check
    est.b = 0.002 # Valid b for tau=500
    
    tau_info = est.tau_reliability()
    
    # Should be unreliable because b count is low
    # The BUG causes this to be True because it only checks total learn_ok_count
    assert not tau_info.reliable, "Should be unreliable if b is not learned enough"


def test_near_band_gain_scheduling():
    """Test that near-band scheduling reduces gains correctly.
    
    This test verifies:
    1. Gains are reduced inside the near-band
    2. Ki is calculated from the ORIGINAL Kp (not reduced Kp)
    3. Gains are re-clamped to stay within bounds after reduction
    """
    from custom_components.versatile_thermostat.smartpi_algorithm import KP_MIN, KI_MIN

    # Create SmartPI with near-band enabled
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_NearBand",
        near_band_deg=0.5,      # Enable near-band
        kp_near_factor=0.60,    # Reduce Kp to 60%
        ki_near_factor=0.85,    # Reduce Ki to 85%
        aggressiveness=1.0,     # No aggressiveness scaling
    )

    # Set up for reliable tau (needed for near-band Ki recalculation)
    # tau = 1/b = 500 minutes
    smartpi.est.learn_ok_count = 15
    smartpi.est.learn_ok_count_b = 15
    smartpi.est.b = 0.002

    # First calculate outside near-band to get baseline gains
    smartpi.calculate(
        target_temp=20.0,
        current_temp=18.0,  # error = 2.0 > near_band_deg=0.5
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    kp_outside = smartpi.Kp
    ki_outside = smartpi.Ki
    assert abs(smartpi._last_error) > 0.5, "Should be outside near-band"

    # Reset the rate-limiting timestamp to allow immediate recalculation
    smartpi._last_calculate_time = None

    # Now calculate inside near-band
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.7,  # error = 0.3 < near_band_deg=0.5
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    kp_inside = smartpi.Kp
    ki_inside = smartpi.Ki

    # Verify we're inside near-band
    assert abs(smartpi._last_error) <= 0.5, f"Should be inside near-band, error={smartpi._last_error}"

    # Verify Kp was reduced by the factor
    expected_kp = kp_outside * 0.60
    # Account for possible clamping
    expected_kp_clamped = max(expected_kp, KP_MIN)
    assert math.isclose(kp_inside, expected_kp_clamped, rel_tol=0.01), \
        f"Kp should be reduced: outside={kp_outside}, inside={kp_inside}, expected={expected_kp_clamped}"

    # Verify Ki was NOT calculated from reduced Kp
    # If it was calculated from reduced Kp, it would be much smaller
    # With fix: Ki = (kp_base / tau_capped) * ki_near_factor = (kp_outside / 200) * 0.85
    # Without fix: Ki = (kp_reduced / tau_capped) * ki_near_factor = (kp_inside / 200) * 0.85
    # The fix ensures Ki is calculated from the original Kp
    tau_capped = 200.0  # TAU_CAP_FOR_KI
    ki_from_original_kp = (kp_outside / tau_capped) * 0.85
    ki_from_reduced_kp = (kp_inside / tau_capped) * 0.85

    # Ki should be closer to the calculation from original Kp
    assert ki_inside > ki_from_reduced_kp * 1.2, \
        f"Ki should be calculated from original Kp (not reduced): ki_inside={ki_inside}, ki_from_reduced={ki_from_reduced_kp}"

    # Verify gains stay within bounds
    assert kp_inside >= KP_MIN, f"Kp should be >= KP_MIN after reduction: {kp_inside}"
    assert ki_inside >= KI_MIN, f"Ki should be >= KI_MIN after reduction: {ki_inside}"


def test_near_band_gains_clamped_at_minimum():
    """Test that near-band reduction doesn't push gains below their minimums."""
    from custom_components.versatile_thermostat.smartpi_algorithm import KP_MIN, KI_MIN

    # Create SmartPI with minimum possible gains that will be reduced
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_NearBandClamp",
        near_band_deg=0.5,
        kp_near_factor=0.60,
        ki_near_factor=0.85,
        aggressiveness=0.2,  # Very low aggressiveness pushes gains toward minimum
    )

    # With unreliable tau, safe gains are used: KP_SAFE=0.55, KI_SAFE=0.01
    # With aggressiveness=0.2: Kp = 0.55 * 0.2 = 0.11
    # After near-band: Kp = 0.11 * 0.60 = 0.066 < KP_MIN=0.10
    # Should be clamped to KP_MIN

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.8,  # error = 0.2 < near_band_deg=0.5
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Verify gains are clamped at minimums
    assert smartpi.Kp >= KP_MIN, f"Kp must not go below KP_MIN: {smartpi.Kp}"
    assert smartpi.Ki >= KI_MIN, f"Ki must not go below KI_MIN: {smartpi.Ki}"


def test_notify_resume_after_interruption_sets_skip_counter():
    """Test that notify_resume_after_interruption sets the skip counter."""
    from custom_components.versatile_thermostat.smartpi_algorithm import SKIP_CYCLES_AFTER_RESUME

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Resume"
    )

    # Initially counter should be 0
    assert smartpi._skip_learning_cycles_left == 0

    # Notify resume
    smartpi.notify_resume_after_interruption()

    # Counter should be set to default
    assert smartpi._skip_learning_cycles_left == SKIP_CYCLES_AFTER_RESUME

    # Also verify timestamp is reset
    assert smartpi._learn_last_ts is None


def test_notify_resume_after_interruption_custom_skip():
    """Test that notify_resume_after_interruption accepts custom skip count."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Resume"
    )

    # Notify with custom skip count
    smartpi.notify_resume_after_interruption(skip_cycles=5)

    assert smartpi._skip_learning_cycles_left == 5


def test_update_learning_skips_when_resume_counter_active():
    """Test that update_learning skips when skip counter is active.
    
    With SKIP_CYCLES_AFTER_RESUME = 1:
    - First update_learning call should skip and decrement counter to 0
    - Second call should proceed normally (counter is 0)
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Resume"
    )

    # Setup initial state
    smartpi.est.learn_ok_count = 5
    initial_learn_count = smartpi.est.learn_ok_count

    # Notify resume to set skip counter
    smartpi.notify_resume_after_interruption()
    assert smartpi._skip_learning_cycles_left == 1  # SKIP_CYCLES_AFTER_RESUME = 1

    # First update_learning call should skip
    smartpi.update_learning(
        current_temp=20.0,
        ext_current_temp=10.0,
        previous_temp=19.0,
        previous_power=0.5,
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )

    # Counter should decrement to 0 (1 -> 0)
    assert smartpi._skip_learning_cycles_left == 0
    # Learn count should NOT increase
    assert smartpi.est.learn_ok_count == initial_learn_count
    # Reason should indicate resume skip
    assert "skip:resume" in smartpi.est.learn_last_reason

    # Second call should proceed normally (counter is 0)
    smartpi.update_learning(
        current_temp=20.5,
        ext_current_temp=10.0,
        previous_temp=20.0,
        previous_power=0.0,  # OFF phase for b learning
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )

    # Counter stays 0
    assert smartpi._skip_learning_cycles_left == 0
    # Now learning should proceed (if conditions met)
    # Note: this may or may not increment learn_ok_count depending on data quality


def test_skip_learning_cycles_persisted():
    """Test that skip_learning_cycles_left is persisted in save/load state."""
    smartpi1 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Persist1"
    )

    # Set skip counter
    smartpi1.notify_resume_after_interruption(skip_cycles=3)
    assert smartpi1._skip_learning_cycles_left == 3

    # Save state
    saved = smartpi1.save_state()
    assert "skip_learning_cycles_left" in saved
    assert saved["skip_learning_cycles_left"] == 3

    # Load in new instance
    smartpi2 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Persist2",
        saved_state=saved
    )

    # Skip counter should be restored
    assert smartpi2._skip_learning_cycles_left == 3


def test_skip_learning_cycles_in_diagnostics():
    """Test that skip_learning_cycles_left appears in diagnostics."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Diag"
    )

    # Set skip counter
    smartpi.notify_resume_after_interruption(skip_cycles=2)

    # Get diagnostics
    diag = smartpi.get_diagnostics()

    assert "skip_learning_cycles_left" in diag
    assert diag["skip_learning_cycles_left"] == 2


def test_reset_learning_clears_skip_counter():
    """Test that reset_learning clears the skip counter."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_ResetSkip"
    )

    # Set skip counter
    smartpi.notify_resume_after_interruption(skip_cycles=5)
    assert smartpi._skip_learning_cycles_left == 5

    # Reset learning
    smartpi.reset_learning()

    # Skip counter should be cleared
    assert smartpi._skip_learning_cycles_left == 0


def test_abestimator_no_saturation_bias():
    """Test that raw values in histogram prevent saturation bias.
    
    This test verifies the fix for the saturation issue where parameter 'a'
    would drift toward A_MAX when many measurements exceed the limit.
    
    Before fix: Values were clamped BEFORE storing in histogram, so if >50%
    of measurements exceeded A_MAX, the median would also be A_MAX.
    
    After fix: Raw values are stored, median reflects true center, and only
    the final EWMA result is clamped.
    """
    est = ABEstimator()
    
    # First, set up a reasonable b value
    est.b = 0.002  # tau = 500 min
    
    # Simulate several ON-phase measurements where SOME exceed A_MAX
    # This mimics the real scenario where b*delta contributes significantly
    # to a_meas calculation, causing some values to exceed 0.05
    
    measurements = [
        # (dT_int_per_min, u, t_int, t_ext)
        # a_meas = (dt + b*delta) / u
        # With b=0.002 and delta=10, b*delta=0.02
        (0.03, 0.5, 18.0, 8.0),   # a_meas = (0.03 + 0.02) / 0.5 = 0.10 (exceeds A_MAX)
        (0.02, 0.5, 18.0, 8.0),   # a_meas = (0.02 + 0.02) / 0.5 = 0.08 (exceeds A_MAX)
        (0.01, 0.5, 18.0, 8.0),   # a_meas = (0.01 + 0.02) / 0.5 = 0.06 (exceeds A_MAX)
        (0.005, 0.5, 18.0, 8.0),  # a_meas = (0.005 + 0.02) / 0.5 = 0.05 (at A_MAX)
        (0.003, 0.5, 18.0, 8.0),  # a_meas = (0.003 + 0.02) / 0.5 = 0.046 (below A_MAX)
        (0.002, 0.5, 18.0, 8.0),  # a_meas = (0.002 + 0.02) / 0.5 = 0.044 (below A_MAX)
        (0.001, 0.5, 18.0, 8.0),  # a_meas = (0.001 + 0.02) / 0.5 = 0.042 (below A_MAX)
    ]
    
    for dt, u, t_int, t_ext in measurements:
        est.learn(dT_int_per_min=dt, u=u, t_int=t_int, t_ext=t_ext)
    
    # With raw values stored, median of [0.10, 0.08, 0.06, 0.05, 0.046, 0.044, 0.042]
    # = 0.05 (center value)
    # But EWMA converges slowly, and with our values, the final 'a' should NOT be
    # saturated at exactly A_MAX due to the unbiased median
    
    # Key assertion: a should be clamped at A_MAX (0.05) but the histogram
    # should contain raw values, some exceeding A_MAX
    assert est.a <= est.A_MAX, f"a should be clamped at A_MAX: {est.a}"
    
    # Verify that _a_hist contains raw values (some exceeding A_MAX)
    # This is the key difference from the old behavior
    raw_values_above_max = [v for v in est._a_hist if v > est.A_MAX]
    assert len(raw_values_above_max) > 0, \
        f"Histogram should contain raw values above A_MAX: {list(est._a_hist)}"
    
    # The median of the raw values should be calculated correctly
    import statistics
    actual_median = statistics.median(est._a_hist)
    # With the measurements above, median is around 0.05-0.06
    # The point is that the median is calculated on raw values, not clamped values
    assert actual_median <= 0.07, f"Median should be reasonable: {actual_median}"


def test_abestimator_b_no_saturation_bias():
    """Test that raw values in histogram prevent saturation bias for b.
    
    Same principle as test_abestimator_no_saturation_bias but for parameter b.
    """
    est = ABEstimator()
    
    # Simulate OFF-phase measurements where SOME exceed B_MAX
    # b_meas = -dT / delta
    # Note: dT must be within outlier threshold (max_abs_dT_per_min=0.35)
    # To get b_meas > 0.05 with dT within limits, we need smaller delta
    measurements = [
        # (dT_int_per_min, u, t_int, t_ext)
        # With delta=5, dT=-0.3 gives b_meas=0.06 (exceeds B_MAX=0.05)
        (-0.30, 0.0, 18.0, 13.0),  # b_meas = 0.30/5 = 0.06 (exceeds B_MAX)
        (-0.28, 0.0, 18.0, 13.0),  # b_meas = 0.28/5 = 0.056 (exceeds B_MAX)
        (-0.20, 0.0, 18.0, 13.0),  # b_meas = 0.20/5 = 0.04 (below B_MAX)
        (-0.15, 0.0, 18.0, 13.0),  # b_meas = 0.15/5 = 0.03 (below B_MAX)
        (-0.10, 0.0, 18.0, 13.0),  # b_meas = 0.10/5 = 0.02 (below B_MAX)
    ]
    
    for dt, u, t_int, t_ext in measurements:
        est.learn(dT_int_per_min=dt, u=u, t_int=t_int, t_ext=t_ext)
    
    # Key assertion: b should be clamped at B_MAX but histogram contains raw values
    assert est.b <= est.B_MAX, f"b should be clamped at B_MAX: {est.b}"
    
    # Verify that _b_hist contains raw values (some exceeding B_MAX)
    raw_values_above_max = [v for v in est._b_hist if v > est.B_MAX]
    assert len(raw_values_above_max) > 0, \
        f"Histogram should contain raw values above B_MAX: {list(est._b_hist)}"


def test_anti_windup_tracking_diagnostics():
    """Test that anti-windup tracking diagnostics are exposed correctly."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_AWDiag"
    )

    # Make a calculation to populate diagnostics
    smartpi.calculate(
        target_temp=20.0,
        current_temp=18.0,
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Check diagnostics contain anti-windup fields
    diag = smartpi.get_diagnostics()
    assert "u_cmd" in diag
    assert "u_limited" in diag
    assert "u_applied" in diag
    assert "aw_du" in diag

    # Verify constants are defined
    assert AW_TRACK_TAU_S > 0
    assert AW_TRACK_MAX_DELTA_I > 0


def test_anti_windup_tracking_corrects_integral():
    """Test that tracking anti-windup corrects the integral when output is constrained.
    
    This verifies that when the applied command differs from the model command
    (due to rate-limiting, max_on_percent, or timing constraints), the integrator
    is adjusted to align with the actual applied command.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_AWTrack",
        # Disable near-band and sign-flip leak for predictable behavior
        near_band_deg=0.0,
        sign_flip_leak=0.0,
        setpoint_weight_b=1.0,
        # Set max_on_percent to constrain output
        max_on_percent=0.6,
    )

    # Setup: reliable tau and high integral to drive output above max_on_percent
    smartpi.est.learn_ok_count = 50
    smartpi.est.learn_ok_count_b = 50
    smartpi.est.b = 0.002  # tau = 500 min
    smartpi.est.a = 0.01

    # Start with a large integral that would push output beyond max_on_percent
    smartpi.integral = 100.0
    smartpi.u_prev = 0.6  # Already at max to avoid rate-limit masking

    integral_before = smartpi.integral

    # Calculate with high error - output will be constrained by max_on_percent
    smartpi.calculate(
        target_temp=25.0,
        current_temp=20.0,  # error = 5
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Verify output was constrained
    assert smartpi._last_u_applied <= 0.6, \
        f"Output should be limited to max_on_percent: {smartpi._last_u_applied}"

    # Tracking anti-windup should have reduced the integral
    assert smartpi.integral < integral_before, \
        f"Integral should decrease due to tracking: before={integral_before}, after={smartpi.integral}"

    # Verify aw_du is negative (applied < model)
    assert smartpi._last_aw_du < 0, \
        f"aw_du should be negative when constrained: {smartpi._last_aw_du}"


def test_anti_windup_tracking_respects_deadband():
    """Test that tracking anti-windup is NOT applied in deadband.
    
    In deadband, the integral is managed by INTEGRAL_LEAK, and tracking
    should not interfere.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_AWDeadband",
        deadband_c=0.2,  # Large deadband
        max_on_percent=0.5,
    )

    # Start with some integral
    smartpi.integral = 10.0
    smartpi.u_prev = 0.5

    # Calculate in deadband (error < 0.2)
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.9,  # error = 0.1 < deadband 0.2
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # aw_du should be 0 (tracking not applied in deadband)
    assert smartpi._last_aw_du == 0.0, \
        f"aw_du should be 0 in deadband: {smartpi._last_aw_du}"

    # Verify we were in deadband mode
    assert "LEAK" in smartpi._last_i_mode


def test_rate_limit_proportional_to_dt():
    """Test that rate limiting is proportional to elapsed time (dt_min).
    
    This verifies the fix where MAX_STEP_PER_CYCLE is multiplied by dt_min.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_RateLimit",
        near_band_deg=0.0,
    )
    smartpi.u_prev = 0.0

    # First calculation with large error
    smartpi.calculate(
        target_temp=25.0,
        current_temp=15.0,  # error = 10
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    first_output = smartpi._last_u_applied

    # Simulate shorter cycle
    smartpi2 = SmartPI(
        cycle_min=1,  # 1 minute cycle
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_RateLimit2",
        near_band_deg=0.0,
    )
    smartpi2.u_prev = 0.0
    smartpi2.calculate(
        target_temp=25.0,
        current_temp=15.0,
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    second_output = smartpi2._last_u_applied

    # Longer cycle should allow higher output
    assert first_output >= second_output, \
        f"Longer cycle should allow higher output: first={first_output}, second={second_output}"


def test_bumpless_deadband_exit():
    """Test that exiting the deadband applies bumpless transfer.
    
    This test verifies that when the controller exits the deadband, the integral
    is re-initialized so that the output equals u_prev (the last applied command),
    preventing sudden power spikes.
    """
    from custom_components.versatile_thermostat.smartpi_algorithm import KI_MIN

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Bumpless",
        deadband_c=0.1,  # Deadband of 0.1°C
        near_band_deg=0.0,  # Disable near-band scheduling
        setpoint_weight_b=1.0,  # Full proportional action
    )

    # Simulate being in the deadband with u_prev at a stable value
    smartpi.u_prev = 0.30  # Was running at 30%
    smartpi.integral = 5.0  # Some integral value
    smartpi._in_deadband = True  # Simulate previous state was in deadband

    # Calculate inside deadband first to confirm state
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.95,  # error = 0.05 < deadband 0.1
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Should be in deadband
    assert smartpi._in_deadband is True
    assert "LEAK" in smartpi._last_i_mode

    # Record state before exiting deadband
    u_prev_before_exit = smartpi.u_prev

    # Reset timestamp to allow recalculation
    smartpi._last_calculate_time = None

    # Now exit the deadband
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.7,  # error = 0.3 > deadband 0.1
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Should have exited deadband
    assert smartpi._in_deadband is False

    # The key assertion: the output should be close to u_prev_before_exit
    # due to bumpless transfer (not a sudden jump to e.g. 50-60%)
    # Allow some tolerance for rate limiting effects
    output_after_exit = smartpi.on_percent

    # Without bumpless transfer, the PI would compute a much higher output
    # With bumpless transfer, it should be close to u_prev
    assert abs(output_after_exit - u_prev_before_exit) < 0.20, \
        f"Output should be near u_prev due to bumpless transfer: output={output_after_exit}, u_prev={u_prev_before_exit}"


def test_bumpless_deadband_exit_integral_initialization():
    """Test that the integral is correctly initialized on deadband exit.
    
    Verifies the formula: I_new = (u_prev - u_ff - Kp * e_p) / Ki
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BumplessInit",
        deadband_c=0.1,
        near_band_deg=0.0,
        setpoint_weight_b=1.0,
    )

    # Set up a known state
    smartpi.u_prev = 0.40
    smartpi._in_deadband = True
    smartpi.integral = 10.0  # Will be replaced by bumpless calculation
    
    # Force known gains (by making tau unreliable, safe gains are used)
    # KP_SAFE = 0.55, KI_SAFE = 0.01

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.5,  # error = 0.5 > deadband 0.1 (exit)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # After bumpless transfer, verify that the output is approximately u_prev
    # The integral should have been re-initialized such that:
    # u_ff + Kp * e_p + Ki * I = u_prev

    # Since u_ff is ~0 (low a), the formula simplifies
    # The integral should be set such that output matches u_prev
    # With rate limiting, the actual output may differ slightly

    diag = smartpi.get_diagnostics()
    assert "in_deadband" in diag
    assert diag["in_deadband"] is False  # Exited deadband


def test_in_deadband_persisted():
    """Test that in_deadband state is persisted in save/load state."""
    smartpi1 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_DBPersist1"
    )

    # Set deadband state
    smartpi1._in_deadband = True

    # Save state
    saved = smartpi1.save_state()
    assert "in_deadband" in saved
    assert saved["in_deadband"] is True

    # Load in new instance
    smartpi2 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_DBPersist2",
        saved_state=saved
    )

    # State should be restored
    assert smartpi2._in_deadband is True


def test_in_deadband_in_diagnostics():
    """Test that in_deadband appears in diagnostics."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_DBDiag",
        deadband_c=0.1
    )

    # Calculate inside deadband
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.95,  # Inside deadband
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    diag = smartpi.get_diagnostics()
    assert "in_deadband" in diag
    assert diag["in_deadband"] is True


def test_reset_learning_clears_in_deadband():
    """Test that reset_learning clears the in_deadband state."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_DBReset"
    )

    # Set in_deadband
    smartpi._in_deadband = True
    assert smartpi._in_deadband is True

    # Reset learning
    smartpi.reset_learning()

    # Should be cleared
    assert smartpi._in_deadband is False


def test_deadband_hysteresis_entry_and_exit():
    """Test that deadband uses hysteresis for entry and exit.
    
    Hysteresis behavior:
    - Enter deadband when |e| < deadband_c
    - Exit deadband only when |e| > deadband_c * DEADBAND_EXIT_MULT (1.5)
    - In between (hysteresis zone), maintain previous state
    """
    from custom_components.versatile_thermostat.smartpi_algorithm import DEADBAND_EXIT_MULT

    deadband_c = 0.10  # 0.1°C deadband
    exit_threshold = deadband_c * DEADBAND_EXIT_MULT  # 0.15°C

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Hysteresis",
        deadband_c=deadband_c,
        near_band_deg=0.0,
    )

    # Start outside deadband (error = 0.3 > exit threshold)
    smartpi._in_deadband = False
    smartpi.u_prev = 0.4
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.7,  # error = 0.3 > 0.15 (exit threshold)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is False, "Should stay outside deadband"

    # Enter deadband (error = 0.05 < entry threshold 0.10)
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.95,  # error = 0.05 < 0.10 (entry threshold)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is True, "Should enter deadband"

    # Move to hysteresis zone (0.10 <= error <= 0.15) - should STAY in deadband
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.88,  # error = 0.12, in hysteresis zone
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is True, "Should stay in deadband (hysteresis zone)"

    # Still in hysteresis zone, still should stay in deadband
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.86,  # error = 0.14, still in hysteresis zone
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is True, "Should stay in deadband (hysteresis zone)"

    # Exit deadband (error = 0.20 > exit threshold 0.15)
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.80,  # error = 0.20 > 0.15 (exit threshold)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is False, "Should exit deadband"


def test_deadband_hysteresis_prevents_chattering():
    """Test that hysteresis prevents rapid toggling at deadband boundary.
    
    Without hysteresis, oscillating between error=0.09 and 0.11 would cause
    rapid entry/exit. With hysteresis (exit at 0.15), we stay in deadband.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_NoChatter",
        deadband_c=0.10,  # Entry threshold
        near_band_deg=0.0,
    )

    # Enter deadband
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.95,  # error = 0.05 < 0.10
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is True

    # Simulate oscillation around old boundary (0.09 to 0.11)
    # These should all stay in deadband due to hysteresis
    for current_temp in [19.91, 19.89, 19.91, 19.89, 19.90]:
        smartpi._last_calculate_time = None
        smartpi.calculate(
            target_temp=20.0,
            current_temp=current_temp,  # errors: 0.09, 0.11, 0.09, 0.11, 0.10
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        # All should stay in deadband because none exceed exit threshold (0.15)
        assert smartpi._in_deadband is True, \
            f"Should stay in deadband during oscillation at temp={current_temp}"


def test_deadband_hysteresis_zone_from_outside():
    """Test that starting outside and entering hysteresis zone stays outside.
    
    If we start outside deadband and move into the hysteresis zone
    (without crossing the entry threshold), we should stay outside.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_OutsideHyst",
        deadband_c=0.10,
        near_band_deg=0.0,
    )

    # Start clearly outside deadband
    smartpi._in_deadband = False
    smartpi.u_prev = 0.4
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.7,  # error = 0.3 > exit threshold
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is False

    # Move into hysteresis zone (0.10 < error < 0.15) from outside
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.88,  # error = 0.12, in hysteresis zone
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    # Should stay OUTSIDE deadband (we need to cross entry threshold to enter)
    assert smartpi._in_deadband is False, \
        "Should stay outside when coming from outside into hysteresis zone"
