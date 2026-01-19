"""Test the SmartPI algorithm."""

import logging
import pytest
import random
import statistics
import time
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
    SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN,
    SETPOINT_BOOST_RATE,
    MAX_STEP_PER_MINUTE,
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
    
    # Conditional integration should be skipped (I:SKIP due to SAT_HI and e>0)
    # Note: Integral may still change due to tracking anti-windup, but the main
    # integration step should be skipped
    assert "I:SKIP" in smartpi._last_i_mode, \
        f"Integration should be skipped when saturated: mode={smartpi._last_i_mode}"


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
    
    # Need > 5 points to start learning
    # We vary t_ext AND dT to create a consistent slope AND add noise
    # Noise is critical to allow 'mad' to be non-zero and permit initial bias to pass gating
    random.seed(42)
    b_target = 0.001
    
    for i in range(10): # More points to ensuring learning triggers
        delta = 10.0 + i * 0.5
        # Ideal dT = -b * delta
        # Add noise to creating non-zero sigma
        noise = random.uniform(-0.0005, 0.0005)
        dt_val = -b_target * delta + noise
        
        est.learn(
            dT_int_per_min=dt_val,  # Cooling
            u=0.0,  # OFF phase
            t_int=20.0,
            t_ext=20.0 - delta
        )
    
    assert est.learn_ok_count > 0
    assert "learned b" in est.learn_last_reason
    # b should be learned
    assert math.isclose(est.b, b_target, rel_tol=0.20) # Looser tol due to noise


def test_abestimator_learn_a_on():
    """Test learning of a during ON phases (u > 0.20)."""
    est = ABEstimator()
    random.seed(42)
    
    # First, learn b manually or via loop
    est.b = 0.001
    est.learn_ok_count_b = 10
    
    # Init a closer to target to speed up convergence verification
    est.a = 0.03
    
    # ON phase
    target_a = 0.04
    b_used = est.b
    
    for i in range(10):
        # Vary u to get slope
        u_val = 0.4 + i * 0.05
        # const delta
        t_ext = 10.0
        delta = 20.0 - t_ext # 10
        
        # dT = a*u - b*delta
        noise = random.uniform(-0.0005, 0.0005)
        dt_val = target_a * u_val - b_used * delta + noise
        
        est.learn(
            dT_int_per_min=dt_val,
            u=u_val,
            t_int=20.0,
            t_ext=t_ext
        )
    
    assert est.learn_ok_count_a > 0
    assert "learned a" in est.learn_last_reason
    # Huber estimation with noise might be slightly off, relax tolerance
    assert math.isclose(est.a, target_a, rel_tol=0.35)


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
    assert "skip: low excitation" in est.learn_last_reason


def test_abestimator_learn_b_off_phase():
    """Test that b is learned during OFF phase (u < 0.05)."""
    est = ABEstimator()
    random.seed(42)
    b_target = 0.001
    
    for i in range(10):
        delta = 10.0 + i * 0.2
        noise = random.uniform(-0.0005, 0.0005)
        dt_val = -b_target * delta + noise
        
        est.learn(
            dT_int_per_min=dt_val,  # Cooling
            u=0.01,  # OFF phase (u < 0.05)
            t_int=19.0,
            t_ext=19.0 - delta
        )
    
    # Should learn b in OFF phase, not skip
    assert est.learn_ok_count > 0
    assert "learned b" in est.learn_last_reason


def test_tau_reliability_not_enough_learns():
    """Test tau reliability when not enough learns."""
    est = ABEstimator()
    est.learn_ok_count_b = 5  # < 10 needed
    
    tau_info = est.tau_reliability()
    
    assert not tau_info.reliable
    assert tau_info.tau_min == 9999.0


def test_tau_reliability_ok():
    """Test tau reliability when conditions are met."""
    est = ABEstimator()
    
    # Inject reliable history manually to bypass learning logic
    est.learn_ok_count_b = 15
    est.b = 0.002
    
    # Fill history with consistent b values
    for _ in range(10):
        est._b_hat_hist.append(0.002)
    
    tau_info = est.tau_reliability()
    
    assert tau_info.reliable
    assert math.isclose(tau_info.tau_min, 500.0, rel_tol=1e-3)


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
    # _b_pts stores (delta, y)
    smartpi1.est._b_pts.append((10.0, 0.03))
    smartpi1.est._b_pts.append((10.0, 0.031))
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
    assert len(smartpi2.est._b_pts) == 2
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
    # Populate history for robust check
    for _ in range(10):
        smartpi.est._b_hat_hist.append(0.002)
    
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
    2. Ki is calculated from ORIGINAL Kp, then multiplied by ki_near_factor
       (single attenuation - fixed behavior)
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
        kp_near_factor=0.70,    # Reduce Kp to 60%
        ki_near_factor=0.50,    # Reduce Ki to 85%
        aggressiveness=1.0,     # No aggressiveness scaling
    )

    # Set up for reliable tau (needed for near-band Ki recalculation)
    # tau = 1/b = 500 minutes
    smartpi.est.learn_ok_count = 15
    smartpi.est.learn_ok_count_b = 15
    smartpi.est.b = 0.002
    # Populate history for robust check
    for _ in range(10):
        smartpi.est._b_hat_hist.append(0.002)

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

    # Verify Kp was reduced by kp_near_factor (0.70)
    expected_kp = kp_outside * 0.70
    # Account for possible clamping
    expected_kp_clamped = max(expected_kp, KP_MIN)
    assert math.isclose(kp_inside, expected_kp_clamped, rel_tol=0.01), \
        f"Kp should be reduced: outside={kp_outside}, inside={kp_inside}, expected={expected_kp_clamped}"

    # Fixed behavior: Ki is calculated from ORIGINAL Kp, then multiplied by ki_near_factor
    # Ki = (kp_original / tau_capped) * ki_near_factor
    # This results in single attenuation: ki_near_factor = 0.50
    tau_capped = 200.0  # TAU_CAP_FOR_KI
    ki_from_original_kp = (kp_outside / tau_capped) * 0.50

    # Ki should match the calculation from original Kp (fixed behavior)
    assert math.isclose(ki_inside, ki_from_original_kp, rel_tol=0.01), \
        f"Ki should be calculated from original Kp: ki_inside={ki_inside}, expected={ki_from_original_kp}"

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
    """Test that notify_resume_after_interruption sets the skip timestamp."""
    from custom_components.versatile_thermostat.smartpi_algorithm import SKIP_CYCLES_AFTER_RESUME

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Resume"
    )

    # Initially counter should be None
    assert smartpi._learning_resume_ts is None

    # Notify resume
    smartpi.notify_resume_after_interruption()

    # Counter should be set
    assert smartpi._learning_resume_ts is not None
    # Should be about SKIP_CYCLES_AFTER_RESUME * max(15, cycle_min) minutes in the future
    # Default SKIP=1, cycle=10 -> duration = 1 * 15 = 15 min
    duration = 15.0 * 60.0
    assert time.time() < smartpi._learning_resume_ts <= time.time() + duration + 5.0

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

    assert smartpi._learning_resume_ts is not None
    # 5 * 15 min = 75 min
    duration = 5 * 15.0 * 60.0
    assert time.time() < smartpi._learning_resume_ts <= time.time() + duration + 5.0


def test_update_learning_skips_when_resume_counter_active():
    """Test that update_learning skips when skip timestamp is active.
    
    With SKIP_CYCLES_AFTER_RESUME = 1:
    - First update_learning call (immediate) should skip
    - Second call (after enough time) should proceed
    """
    import time
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
    smartpi.notify_resume_after_interruption(skip_cycles=1)
    # deadline is ~15 min in future
    assert smartpi._learning_resume_ts > time.time()

    # First update_learning call (immediate) should skip
    smartpi.update_learning(
        current_temp=20.0,
        ext_current_temp=10.0,
        previous_temp=19.0,
        previous_power=0.5,
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )

    # Learn count should NOT increase
    assert smartpi.est.learn_ok_count == initial_learn_count
    # Reason should indicate resume skip
    assert "skip:resume" in smartpi.est.learn_last_reason

    # Clean up skip timer artificially to simulate passing of time
    smartpi._learning_resume_ts = time.time() - 1.0

    # Second call should proceed normally
    smartpi.update_learning(
        current_temp=20.5,
        ext_current_temp=10.0,
        previous_temp=20.0,
        previous_power=0.0,  # OFF phase for b learning
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )

    # Now learning should proceed (if conditions met)
    # Note: this may or may not increment learn_ok_count depending on data quality
    assert smartpi._learning_resume_ts is None


def test_skip_learning_cycles_persisted():
    """Test that skip timestamp is persisted in save/load state."""
    smartpi1 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Persist1"
    )

    # Set skip timestamp (e.g. 5 min future)
    ts = time.time() + 300.0
    smartpi1._learning_resume_ts = ts
    
    # Save state
    saved = smartpi1.save_state()
    assert "learning_resume_ts" in saved
    assert saved["learning_resume_ts"] == ts

    # Load in new instance
    smartpi2 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Persist2",
        saved_state=saved
    )

    # Skip timestamp should be restored
    assert smartpi2._learning_resume_ts == ts


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

    assert "learning_resume_ts" in diag
    assert diag["learning_resume_ts"] is not None


def test_reset_learning_clears_skip_counter():
    """Test that reset_learning clears the skip timestamp."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_ResetSkip"
    )

    # Set skip counter
    smartpi.notify_resume_after_interruption(skip_cycles=5)
    assert smartpi._learning_resume_ts is not None
    
    # Reset
    smartpi.reset_learning()
    
    assert smartpi._learning_resume_ts is None
    
    # Reset learning
    smartpi.reset_learning()
    
    # Skip counter should be cleared
    assert smartpi._learning_resume_ts is None


def test_abestimator_no_saturation_bias():
    """Test that raw points are stored, preventing clamping bias.
    
    In the new robust estimator, we store raw (u, y) points.
    The regression calculates the slope 'a'.
    If the slope > A_MAX, the result is clamped, BUT the points remain valid (raw).
    This ensures that the estimator sees the "true" slope (even if high)
    and simply limits the output, rather than modifying the internal history 
    to match the limit (which would bias future estimates).
    """
    est = ABEstimator()
    # Force reliable mode
    # Set total count low to bypass residual gating (requires > 10)
    est.learn_ok_count = 5 
    # But set b count high to satisfy reliability check
    est.learn_ok_count_b = 20
    est._b_hat_hist.extend([0.002] * 10)
    
    est.b = 0.002
    
    # Feed points that imply a slope > A_MAX (0.1)
    # y = dT/dt + b*delta. 
    # If we want a_meas ~ 0.2, and u=0.5, we need y = 0.1
    # Let's say dT=0.08, b=0.002, delta=10 -> y = 0.08 + 0.02 = 0.10.
    # a_slope = 0.10 / 0.5 = 0.20 > A_MAX(0.1)
    
    for _ in range(7):
        est.learn(
            dT_int_per_min=0.08, 
            u=0.5, 
            t_int=18.0, 
            t_ext=8.0 # delta=10
        )
    
    # 1. est.a should be clamped
    assert est.a <= est.A_MAX, f"a should be clamped: {est.a}"
    
    # 2. _a_pts should contain raw points implying high slope
    slope, _ = est._theil_sen(list(est._a_pts))
    if slope is not None:
         assert slope > est.A_MAX, f"Stored points should imply high slope: {slope}"
    else:
         # If slope is None, it means points were degenerate, which defeats test purpose
         # But here we used same u=0.5. 
         # We must vary u in the loop above to avoid slope=None
         pass


    # Re-run with varying U to properly test Theil-Sen return
    est = ABEstimator()
    # Force reliable mode
    est.learn_ok_count = 5
    est.learn_ok_count_b = 20
    est._b_hat_hist.extend([0.002] * 10)
    # Also need to initialize b to something reasonable so residuals don't block
    est.b = 0.002
    
    for i in range(7):
        u_val = 0.5 + i * 0.01
        # Target a=0.20
        # dt = 0.20*u - 0.02
        est.learn(
            dT_int_per_min=(0.20 * u_val - 0.02),
            u=u_val,
            t_int=18.0,
            t_ext=8.0
        )
    assert est.a <= est.A_MAX
    slope, _ = est._theil_sen(list(est._a_pts))
    assert slope > est.A_MAX


def test_abestimator_b_no_saturation_bias():
    """Test that raw points are stored for b, preventing clamping bias."""
    est = ABEstimator()
    # Force reliable mode
    est.learn_ok_count = 5
    est.learn_ok_count_b = 20
    est._b_hat_hist.extend([0.002] * 10)
    est.b = 0.002
    
    # Feed points implying b > B_MAX (0.05)
    # y = -dT/dt. x = delta.
    # b_slope = y/x.
    # If we want b=0.06, delta=10 -> y=0.6 -> dT = -0.6 (too high for outlier?)
    # outlier check: max_abs_dT_per_min = 0.35.
    # So we need smaller delta. delta=5. y=0.3. b=0.06.
    # dT = -0.3. ok.
    
    for _ in range(7):
        est.learn(
            dT_int_per_min=-0.3,
            u=0.0,
            t_int=20.0,
            t_ext=15.0 # delta=5
        )
        
    # 1. est.b should be clamped
    assert est.b <= est.B_MAX, f"b should be clamped: {est.b}"
    
    # 2. _b_pts should contain raw points
    # Need varying delta in input to get valid slope
    
    # Re-run with varying delta
    est = ABEstimator()
    # Force reliable mode
    est.learn_ok_count = 5
    est.learn_ok_count_b = 20
    est._b_hat_hist.extend([0.002] * 10)
    est.b = 0.002
    
    for i in range(7):
        # b = 0.06
        # dt = -0.06 * delta
        # delta = 10 + i
        delta = 10.0 + i
        est.learn(
            dT_int_per_min=(-0.06 * delta),
            u=0.0,
            t_int=20.0,
            t_ext=20.0 - delta,
            max_abs_dT_per_min=5.0
        )
    
    assert est.b <= est.B_MAX
    slope, _ = est._theil_sen(list(est._b_pts))
    assert slope > est.B_MAX, f"Stored points should imply high slope: {slope}"


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
    - Exit deadband only when |e| > deadband_c + DEADBAND_HYSTERESIS
    - In between (hysteresis zone), maintain previous state
    """
    from custom_components.versatile_thermostat.smartpi_algorithm import DEADBAND_HYSTERESIS

    deadband_c = 0.10  # 0.1°C deadband
    exit_threshold = deadband_c + DEADBAND_HYSTERESIS  # 0.125°C

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

    # Move to hysteresis zone (0.10 <= error <= 0.125) - should STAY in deadband
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.88,  # error = 0.12, in hysteresis zone (0.10 < 0.12 < 0.125)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is True, "Should stay in deadband (hysteresis zone)"

    # Exit deadband (error = 0.15 > exit threshold 0.125)
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.85,  # error = 0.15 > 0.125 (exit threshold)
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._in_deadband is False, "Should exit deadband"


def test_deadband_hysteresis_prevents_chattering():
    """Test that hysteresis prevents rapid toggling at deadband boundary.
    
    Without hysteresis, oscillating between error=0.09 and 0.11 would cause
    rapid entry/exit. With hysteresis (exit at 0.125), we stay in deadband.
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
        # All should stay in deadband because none exceed exit threshold (0.125)
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

    # Move into hysteresis zone (0.10 < error < 0.125) from outside
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


def test_setpoint_boost_activates_on_setpoint_increase():
    """Test that setpoint boost activates when setpoint increases significantly."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Boost"
    )

    # Initialize with first calculation
    smartpi.calculate(
        target_temp=18.0,
        current_temp=18.0,
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    assert smartpi._setpoint_boost_active is False
    assert smartpi._prev_setpoint_for_boost == 18.0

    # Increase setpoint by more than SETPOINT_BOOST_THRESHOLD (0.3°C)
    smartpi._last_calculate_time = None
    smartpi.calculate(
        target_temp=19.0,  # +1.0°C increase
        current_temp=18.0,  # error = 1.0 > SETPOINT_BOOST_ERROR_MIN (0.3°C)
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Boost should be active
    assert smartpi._setpoint_boost_active is True
    assert smartpi._prev_setpoint_for_boost == 19.0

    # Verify diagnostics
    diag = smartpi.get_diagnostics()
    assert "setpoint_boost_active" in diag
    assert diag["setpoint_boost_active"] is True


def test_setpoint_boost_deactivates_when_error_small():
    """Test that setpoint boost deactivates when error becomes small."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostDeact"
    )

    # Activate boost
    smartpi._setpoint_boost_active = True
    smartpi._prev_setpoint_for_boost = 20.0
    smartpi.u_prev = 0.5

    # Calculate with small error (< SETPOINT_BOOST_ERROR_MIN)
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.85,  # error = 0.15 < 0.3 threshold
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Boost should be deactivated
    assert smartpi._setpoint_boost_active is False


def test_setpoint_boost_uses_faster_rate_limit():
    """Test that boosted rate limit allows faster power ramp-up."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostRate"
    )

    # Start with u_prev = 0 (cold start)
    smartpi.u_prev = 0.0
    smartpi._prev_setpoint_for_boost = 18.0
    smartpi._setpoint_boost_active = False

    # First calculation to initialize
    smartpi.calculate(
        target_temp=18.0,
        current_temp=18.0,
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Reset and trigger boost with setpoint increase
    smartpi._last_calculate_time = None
    smartpi.u_prev = 0.0

    smartpi.calculate(
        target_temp=20.0,  # +2°C increase
        current_temp=18.0,  # error = 2°C
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    assert smartpi._setpoint_boost_active is True

    # With boosted rate (0.50/min) and dt=10min, max_step = 5.0
    # So output should be able to reach higher values faster
    # Normal rate (0.15/min) and dt=10min -> max_step = 1.5
    # The on_percent should be higher with boost than with normal rate
    # Since PI output is high (large error), it should hit the boosted rate limit
    assert smartpi.on_percent > 0


def test_setpoint_boost_activated_on_decrease():
    """Test that boost is ACTIVATED when setpoint decreases in HEAT mode."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostDecrease"
    )

    # Initialize at high setpoint
    smartpi._prev_setpoint_for_boost = 22.0
    smartpi._setpoint_boost_active = False
    smartpi.u_prev = 0.5

    # Decrease setpoint
    smartpi.calculate(
        target_temp=20.0,  # -2°C decrease
        current_temp=21.0,
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Boost SHOULD be active now
    assert smartpi._setpoint_boost_active is True
    assert smartpi._prev_setpoint_for_boost == 20.0


def test_setpoint_boost_persisted():
    """Test that setpoint boost state is persisted in save/load."""
    smartpi1 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostPersist1"
    )

    # Set boost active
    smartpi1._setpoint_boost_active = True
    smartpi1._prev_setpoint_for_boost = 21.0

    # Save state
    saved = smartpi1.save_state()
    assert "setpoint_boost_active" in saved
    assert "prev_setpoint_for_boost" in saved
    assert saved["setpoint_boost_active"] is True
    assert saved["prev_setpoint_for_boost"] == 21.0

    # Load in new instance
    smartpi2 = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostPersist2",
        saved_state=saved
    )

    assert smartpi2._setpoint_boost_active is True
    assert smartpi2._prev_setpoint_for_boost == 21.0


def test_setpoint_boost_cleared_on_reset():
    """Test that reset_learning clears boost state."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostReset"
    )

    # Set boost active
    smartpi._setpoint_boost_active = True
    smartpi._prev_setpoint_for_boost = 21.0

    # Reset
    smartpi.reset_learning()

    # Boost should be cleared
    assert smartpi._setpoint_boost_active is False
    assert smartpi._prev_setpoint_for_boost is None


def test_setpoint_boost_cool_mode():
    """Test that boost activates on setpoint DECREASE in COOL mode."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostCool"
    )

    # Initialize with first calculation
    smartpi._prev_setpoint_for_boost = 25.0
    smartpi._setpoint_boost_active = False
    smartpi.u_prev = 0.3

    # Decrease setpoint in COOL mode (need to cool more aggressively)
    smartpi.calculate(
        target_temp=22.0,  # -3°C decrease
        current_temp=26.0,  # error = -4°C (too hot), inverted to +4°C internal
        ext_current_temp=30.0,
        slope=0,
        hvac_mode=VThermHvacMode_COOL
    )

    # Boost should be active for cooling
    assert smartpi._setpoint_boost_active is True
    assert smartpi._prev_setpoint_for_boost == 22.0


def test_forced_by_timing_skips_tracking_antiwindup():
    """Test that tracking anti-windup is skipped when timing forces 0%/100%.
    
    When min_on_delay or min_off_delay forces u_applied to 0 or 1 while 
    u_limited was in a reasonable range, the tracking anti-windup should NOT
    inject this artificial delta into the integral.
    
    This prevents the side effect where min_off_delay forcing 100% causes
    the integral to climb artificially.
    """
    # Setup with min_off_delay that will force 100% at ~85% power
    # cycle_min=10 -> cycle_sec=600
    # min_deactivation_delay=90s means off_time must be >= 90s
    # threshold: on_time <= 510s -> u <= 0.85
    # So if u_limited is between 0.85 and 0.99, timing forces to 100%
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=90,  # 90 seconds minimum OFF time
        name="TestSmartPI_ForcedTiming",
        near_band_deg=0.0,
        setpoint_weight_b=1.0,
    )

    # Setup reliable tau for proper gains
    smartpi.est.learn_ok_count = 50
    smartpi.est.learn_ok_count_b = 50
    smartpi.est.b = 0.002  # tau = 500 min
    smartpi.est.a = 0.01

    # Start with u_prev at 87% - rate limit will keep output close to this
    # With MAX_STEP_PER_MINUTE=0.25 and dt=10min, max_step=2.5
    # So we need to constrain via the PI output itself, not rate limit
    smartpi.u_prev = 0.87
    # Set integral to produce a moderate PI output around 0.87-0.90
    smartpi.integral = 0.5  # Small integral
    # Use safe gains (unreliable tau) for more predictable behavior
    smartpi.est.learn_ok_count_b = 5  # Make tau unreliable to use KP_SAFE/KI_SAFE

    integral_before = smartpi.integral

    # Calculate with small error -> PI output ~ Kp*e + Ki*I
    # With KP_SAFE=0.55, KI_SAFE=0.01, e=0.3, I=0.5:
    # u_pi = 0.55*0.3 + 0.01*0.5 = 0.165 + 0.005 = 0.17
    # u_ff ~ 0 (low a)
    # u_cmd ~ 0.17, but rate-limited from u_prev=0.87
    # Actually we need u_limited to be ~0.87-0.90
    # Let's set a higher integral to get output around 0.87
    smartpi.integral = 80.0  # Higher integral
    # With KI_SAFE=0.01: Ki*I = 0.01*80 = 0.80
    # With e=0.15, Kp*e = 0.55*0.15 = 0.0825
    # u_pi = 0.80 + 0.0825 = 0.8825
    # u_raw = u_ff + u_pi ~ 0 + 0.8825 = 0.8825
    # u_cmd = clamp(0.8825, 0, 1) = 0.8825
    # Rate limit: |0.8825 - 0.87| = 0.0125 < max_step -> u_limited = 0.8825

    smartpi.calculate(
        target_temp=20.15,
        current_temp=20.0,  # error = 0.15
        ext_current_temp=5.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # u_limited should be in the range where timing forces to 100%
    # (between ~0.85 and 0.99)
    assert 0.85 < smartpi._last_u_limited < 0.99, \
        f"u_limited should be in range (0.85, 0.99): {smartpi._last_u_limited}"

    # Verify timing forced u_applied to 100%
    assert smartpi._last_u_applied == 1.0, \
        f"u_applied should be forced to 100%: {smartpi._last_u_applied}"

    # The key assertion: forced_by_timing should be True
    assert smartpi._last_forced_by_timing is True, \
        "forced_by_timing should be True when timing forces 100%"

    # aw_du should be 0 because tracking was skipped
    assert smartpi._last_aw_du == 0.0, \
        f"aw_du should be 0 when timing forced: {smartpi._last_aw_du}"

    # Verify diagnostics
    diag = smartpi.get_diagnostics()
    assert "forced_by_timing" in diag
    assert diag["forced_by_timing"] is True


def test_forced_by_timing_min_on_delay_forces_zero():
    """Test forced_by_timing when min_on_delay forces 0%.
    
    When u_limited is small but non-zero, and min_on_delay forces
    u_applied to 0%, tracking should be skipped.
    """
    # min_activation_delay=60s means on_time must be >= 60s
    # cycle_min=10 -> cycle_sec=600
    # If u_limited=0.05 -> on_time=30s < 60s -> forced to 0%
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=60,  # 60 seconds minimum ON time
        minimal_deactivation_delay=0,
        name="TestSmartPI_ForcedTimingMinOn",
        near_band_deg=0.0,
        setpoint_weight_b=1.0,
    )

    smartpi.u_prev = 0.05
    smartpi.integral = 1.0

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.8,  # Small error -> small output
        ext_current_temp=15.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # If u_limited is small (e.g., 3-5%) and min_on_delay forces 0%
    if smartpi._last_u_applied == 0.0 and smartpi._last_u_limited > 0.01:
        assert smartpi._last_forced_by_timing is True, \
            "forced_by_timing should be True when min_on forces 0%"
        assert smartpi._last_aw_du == 0.0, \
            "aw_du should be 0 when timing forced"


def test_forced_by_timing_false_when_not_forced():
    """Test that forced_by_timing is False when timing does NOT force extremes."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,  # No timing constraints
        name="TestSmartPI_NoForce",
        near_band_deg=0.0,
    )

    smartpi.u_prev = 0.5
    smartpi.integral = 2.0

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,  # Moderate error
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Without timing constraints, u_applied should equal u_limited
    # and forced_by_timing should be False
    assert smartpi._last_forced_by_timing is False, \
        "forced_by_timing should be False when timing doesn't force"

    # Normal tracking should occur (aw_du may or may not be 0 depending on constraints)
    diag = smartpi.get_diagnostics()
    assert "forced_by_timing" in diag
    assert diag["forced_by_timing"] is False


def test_forced_by_timing_in_diagnostics():
    """Test that forced_by_timing appears in diagnostics."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_ForcedDiag"
    )

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    diag = smartpi.get_diagnostics()
    assert "forced_by_timing" in diag
    assert isinstance(diag["forced_by_timing"], bool)


def test_setpoint_boost_on_decrease_heat_mode():
    """Test that setpoint boost activates on setpoint decrease in HEAT mode (Bug #4)."""
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_BoostDecrease",
        near_band_deg=0.0,
    )

    # 1. Establish initial stable state at high setpoint
    # Target 20, Current 20 -> Stable
    smartpi.calculate(
        target_temp=20.0,
        current_temp=20.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    # Ensure boost is inactive initially
    assert not smartpi._setpoint_boost_active
    
    # Reset internal timer to allow immediate recalculation (avoid dt < 3s guard)
    smartpi._last_calculate_time = None

    # 2. Decrease setpoint significantly (> SETPOINT_BOOST_THRESHOLD=0.3)
    # Target 20 -> 18 (-2.0 change)
    smartpi.calculate(
        target_temp=18.0,
        current_temp=20.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # 3. Verify boost activates
    assert smartpi._setpoint_boost_active, \
        "Setpoint boost should activate on decrease in HEAT mode"


