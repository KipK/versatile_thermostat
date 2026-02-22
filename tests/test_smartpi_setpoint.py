"""Tests for SmartPISetpointManager.filter_setpoint EMA activation fix."""
import math
import pytest

from custom_components.versatile_thermostat.smartpi.setpoint import SmartPISetpointManager
from custom_components.versatile_thermostat.vtherm_hvac_mode import (
    VThermHvacMode_HEAT,
    VThermHvacMode_COOL,
)
from custom_components.versatile_thermostat.smartpi.const import SP_TAU_SLOW, SP_TAU_FAST, SP_BAND


DT_MIN = 5.0  # 5-minute cycle


def _alpha(gap: float, dt_min: float) -> float:
    """Expected alpha for a given gap."""
    w = min(gap / SP_BAND, 1.0)
    tau = SP_TAU_SLOW + (SP_TAU_FAST - SP_TAU_SLOW) * w
    return 1.0 - math.exp(-max(dt_min, 0.001) / max(tau, 1.0))


class TestFilterSetpointEmaActivation:
    """Verify that EMA kicks in at midpoint (not bypassed)."""

    def _make_manager(self) -> SmartPISetpointManager:
        m = SmartPISetpointManager(name="test", enabled=True)
        return m

    # ------------------------------------------------------------------
    # Phase 1: before midpoint, filtered_setpoint == target_temp
    # ------------------------------------------------------------------
    def test_start_phase_returns_target(self):
        """Before midpoint, filter returns raw target_temp."""
        m = self._make_manager()
        # First call initializes state
        result = m.filter_setpoint(
            target_temp=22.0, current_temp=19.0,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )
        assert result == 22.0

        # Setpoint now increases: trigger filtering path
        result = m.filter_setpoint(
            target_temp=22.0, current_temp=19.0,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )
        # Still below midpoint (19 < (19+22)/2 = 20.5)
        assert result == 22.0
        assert m.filtered_setpoint == 22.0

    # ------------------------------------------------------------------
    # Phase 2: first iteration AT midpoint — filtered_setpoint must drop
    # ------------------------------------------------------------------
    def test_midpoint_transition_initializes_ema_from_current_temp(self):
        """
        On the first call where current_temp >= midpoint,
        filtered_setpoint must NOT stay at target_temp.
        It should be initialized to current_temp and then one EMA step applied.
        """
        m = self._make_manager()
        initial_temp = 19.0
        target = 22.0
        midpoint = (initial_temp + target) / 2.0  # 20.5

        # First call: state init (previous setpoint = 19.0)
        m.filter_setpoint(
            target_temp=initial_temp, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )

        # Setpoint jumps to 22 — trigger filter arm
        m.filter_setpoint(
            target_temp=target, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )
        # filtered_setpoint is now target_temp (22.0)
        assert m.filtered_setpoint == target
        assert m.initial_temp_for_filter == initial_temp

        # A few cycles below midpoint — no EMA yet
        for _ in range(3):
            result = m.filter_setpoint(
                target_temp=target, current_temp=20.0,
                hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
            )
            assert result == target, "Should still track raw setpoint before midpoint"

        # First cycle AT midpoint
        current_at_mid = midpoint  # 20.5
        result = m.filter_setpoint(
            target_temp=target, current_temp=current_at_mid,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )

        # The result must NOT be target_temp (that would mean EMA was bypassed)
        assert result < target, (
            f"EMA was bypassed: filtered_setpoint returned {result} == target {target}. "
            "Bug: filtered_setpoint was not initialized from current_temp at midpoint."
        )

        # The result should be an EMA step from current_temp toward target
        gap_expected = abs(target - current_at_mid)  # 22 - 20.5 = 1.5
        alpha = _alpha(gap_expected, DT_MIN)
        expected = alpha * target + (1 - alpha) * current_at_mid
        assert abs(result - expected) < 1e-6, (
            f"Expected EMA result {expected:.4f}, got {result:.4f}"
        )

    # ------------------------------------------------------------------
    # Phase 3: subsequent iterations converge smoothly
    # ------------------------------------------------------------------
    def test_subsequent_iterations_converge_toward_target(self):
        """After midpoint, successive EMA steps must move filtered_setpoint toward target."""
        m = self._make_manager()
        initial_temp = 19.0
        target = 22.0
        midpoint = (initial_temp + target) / 2.0

        # Init
        m.filter_setpoint(
            target_temp=initial_temp, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )
        # Arm filter
        m.filter_setpoint(
            target_temp=target, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
        )

        # Advance past midpoint (current_temp held slightly above midpoint)
        # SP_TAU_SLOW=200 min means full convergence takes many cycles; we only
        # verify monotonic approach and meaningful progress (>50% of gap covered).
        current_sim = midpoint + 0.1
        prev = None
        halfway = midpoint + 0.1 + 0.5 * (target - (midpoint + 0.1))
        for i in range(20):
            result = m.filter_setpoint(
                target_temp=target, current_temp=current_sim,
                hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
            )
            if prev is not None:
                assert result >= prev, (
                    f"filtered_setpoint decreased at step {i}: {result} < {prev}"
                )
            prev = result

        assert result >= halfway, (
            f"EMA made insufficient progress after 20 steps: {result:.3f} < {halfway:.3f}"
        )

    # ------------------------------------------------------------------
    # Clamp: temperature rises faster than EMA
    # ------------------------------------------------------------------
    def test_filtered_setpoint_never_below_current_temp_in_heat(self):
        """
        When current_temp rises faster than the EMA, filtered_setpoint must
        be clamped to current_temp so the error never goes negative.
        """
        m = self._make_manager()
        initial_temp = 19.0
        target = 22.0
        midpoint = (initial_temp + target) / 2.0  # 20.5

        # Init and arm
        m.filter_setpoint(target_temp=initial_temp, current_temp=initial_temp,
                          hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN)
        m.filter_setpoint(target_temp=target, current_temp=initial_temp,
                          hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN)

        # Simulate fast-rising temperature that overtakes the EMA
        for step in range(10):
            # Temperature rising by 0.2°C each cycle — much faster than EMA
            current_fast = midpoint + step * 0.2
            result = m.filter_setpoint(
                target_temp=target, current_temp=current_fast,
                hvac_mode=VThermHvacMode_HEAT, dt_min=DT_MIN
            )
            assert result >= current_fast - 1e-9, (
                f"Step {step}: filtered_setpoint {result:.4f} < current_temp {current_fast:.4f}, "
                "error would go negative"
            )
            assert result <= target + 1e-9, (
                f"Step {step}: filtered_setpoint {result:.4f} > target {target}"
            )

    # ------------------------------------------------------------------
    # COOL mode symmetry
    # ------------------------------------------------------------------
    def test_cool_mode_midpoint_transition(self):
        """Same fix applies to COOL mode (temperature decreasing)."""
        m = self._make_manager()
        initial_temp = 26.0
        target = 23.0
        midpoint = (initial_temp + target) / 2.0  # 24.5

        m.filter_setpoint(
            target_temp=initial_temp, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_COOL, dt_min=DT_MIN
        )
        m.filter_setpoint(
            target_temp=target, current_temp=initial_temp,
            hvac_mode=VThermHvacMode_COOL, dt_min=DT_MIN
        )

        # Below midpoint in COOL (temperature still too high, not yet at midpoint)
        result = m.filter_setpoint(
            target_temp=target, current_temp=25.0,
            hvac_mode=VThermHvacMode_COOL, dt_min=DT_MIN
        )
        assert result == target

        # At midpoint
        result = m.filter_setpoint(
            target_temp=target, current_temp=midpoint,
            hvac_mode=VThermHvacMode_COOL, dt_min=DT_MIN
        )
        assert result > target, (
            f"COOL: EMA bypassed — result {result} should be > target {target}"
        )
