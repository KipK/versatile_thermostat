"""Tests for SmartPISetpointManager — Saturation Guard + asymmetric EMA filter (spec v1.1)."""
import pytest

from custom_components.versatile_thermostat.smartpi.setpoint import SmartPISetpointManager
from custom_components.versatile_thermostat.smartpi.const import (
    SP_TAU_SLOW,
    SP_TAU_FAST,
    SP_SATURATION_THRESHOLD,
    SP_SETPOINT_JUMP_THRESHOLD,
    SP_HYST,
)


DT_MIN = 1.0  # 1-minute cycle (= 60 s)
DT_S = DT_MIN * 60.0


def _alpha(tau_s: float, dt_s: float = DT_S) -> float:
    """Expected EMA alpha for a given tau (seconds)."""
    return dt_s / (tau_s + dt_s)


def _make_manager(enabled: bool = True) -> SmartPISetpointManager:
    return SmartPISetpointManager(name="test", enabled=enabled)


# ---------------------------------------------------------------------------
# Bumpless transfer — initialisation (spec §6.5)
# ---------------------------------------------------------------------------

class TestBumplessTransfer:
    """On first call, filter_state must start from current_temp, not target_temp."""

    def test_init_from_current_temp(self):
        """On first call (FILTER mode), filter_state must start from current_temp, not target_temp."""
        m = _make_manager()
        # delta 0.3 < SP_SETPOINT_JUMP_THRESHOLD (0.5) and < SP_SATURATION_THRESHOLD (1.0)
        target = 18.3
        current = 18.0
        result = m.filter_setpoint(target_temp=target, current_temp=current, dt_min=DT_MIN)
        # filter_state is initialised to current_temp=18.0 (bumpless transfer), then
        # EMA applied once with direction UP and SP_TAU_SLOW
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * current
        assert abs(result - expected) < 1e-9
        assert abs(m.filtered_setpoint - expected) < 1e-9

    def test_init_fallback_when_no_current_temp(self):
        m = _make_manager()
        result = m.filter_setpoint(target_temp=22.0, current_temp=None, dt_min=DT_MIN)
        assert result == 22.0
        assert m.filtered_setpoint == 22.0

    def test_direction_and_tau_initialized(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=22.0, current_temp=19.0, dt_min=DT_MIN)
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW


# ---------------------------------------------------------------------------
# No current_temp — no update
# ---------------------------------------------------------------------------

class TestNoCurrentTemp:
    def test_no_update_when_current_temp_none(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=22.0, current_temp=20.0, dt_min=DT_MIN)
        first_state = m.filtered_setpoint

        result = m.filter_setpoint(target_temp=22.0, current_temp=None, dt_min=DT_MIN)
        assert result == first_state
        assert m.filtered_setpoint == first_state


# ---------------------------------------------------------------------------
# BYPASS mode — |delta_SP| >= SATURATION_THRESHOLD (spec §4)
# ---------------------------------------------------------------------------

class TestBypassMode:
    """When |SP_brut - y| >= threshold, SP_for_P = SP_brut immediately."""

    def test_large_step_up_returns_target(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=21.0, current_temp=19.0, dt_min=DT_MIN)
        assert result == 21.0
        assert m.filtered_setpoint == 21.0

    def test_large_step_down_returns_target(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=21.0, current_temp=21.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=19.0, current_temp=21.0, dt_min=DT_MIN)
        assert result == 19.0
        assert m.filtered_setpoint == 19.0

    def test_bypass_tracks_sp_brut_continuously(self):
        """filter_state follows SP_brut during BYPASS so transition is seamless."""
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        for _ in range(5):
            result = m.filter_setpoint(target_temp=21.0, current_temp=19.5, dt_min=DT_MIN)
            assert result == 21.0
        assert m.filtered_setpoint == 21.0

    def test_external_disturbance_triggers_bypass(self):
        """A temperature drop without setpoint change triggers BYPASS automatically."""
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=19.0, current_temp=17.5, dt_min=DT_MIN)
        assert result == 19.0

    def test_exactly_at_threshold_triggers_bypass(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        result = m.filter_setpoint(
            target_temp=19.0 + SP_SATURATION_THRESHOLD,
            current_temp=19.0,
            dt_min=DT_MIN,
        )
        assert result == 19.0 + SP_SATURATION_THRESHOLD

    def test_just_below_threshold_uses_filter(self):
        """Just below BOTH bypass thresholds → FILTER mode applies.
        Effective threshold is min(SP_SATURATION_THRESHOLD, SP_SETPOINT_JUMP_THRESHOLD)."""
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        # Stay below the tighter threshold (SP_SETPOINT_JUMP_THRESHOLD = 0.5)
        target = 19.0 + SP_SETPOINT_JUMP_THRESHOLD - 0.01
        result = m.filter_setpoint(target_temp=target, current_temp=19.0, dt_min=DT_MIN)
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * 19.0
        assert abs(result - expected) < 1e-9


# ---------------------------------------------------------------------------
# FILTER mode — EMA with direction and hysteresis (spec §5)
# ---------------------------------------------------------------------------

class TestFilterMode:

    def test_ema_direction_up_uses_tau_slow(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        # +0.3 °C: below SP_SETPOINT_JUMP_THRESHOLD (0.5) → FILTER mode, UP direction
        target = 19.3
        result = m.filter_setpoint(target_temp=target, current_temp=19.0, dt_min=DT_MIN)
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * 19.0
        assert abs(result - expected) < 1e-9
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW

    def test_ema_direction_down_uses_tau_fast(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.5, current_temp=19.5, dt_min=DT_MIN)
        # -0.3 °C: below SP_SETPOINT_JUMP_THRESHOLD (0.5) → FILTER mode, DOWN direction
        target = 19.2
        result = m.filter_setpoint(target_temp=target, current_temp=19.5, dt_min=DT_MIN)
        alpha = _alpha(SP_TAU_FAST)
        expected = alpha * target + (1.0 - alpha) * 19.5
        assert abs(result - expected) < 1e-9
        assert m._direction == "DOWN"
        assert m._tau_f_prev == SP_TAU_FAST

    def test_small_step_no_saturation(self):
        """Small step (+0.3 degC) stays in FILTER mode (spec §8.3)."""
        m = _make_manager()
        m.filter_setpoint(target_temp=17.0, current_temp=17.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=17.3, current_temp=17.0, dt_min=DT_MIN)
        assert result < 17.3

    def test_ema_progresses_over_multiple_cycles(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        # +0.3 °C step: below SP_SETPOINT_JUMP_THRESHOLD → stays in FILTER mode every cycle
        target = 19.3
        current = 19.2
        state = 19.0
        for _ in range(10):
            result = m.filter_setpoint(target_temp=target, current_temp=current, dt_min=DT_MIN)
            alpha = _alpha(SP_TAU_SLOW)
            state = alpha * target + (1.0 - alpha) * state
            assert abs(result - state) < 1e-9
        assert result < target
        assert result > 19.0


# ---------------------------------------------------------------------------
# Direction hysteresis — no chattering (spec §5.3)
# ---------------------------------------------------------------------------

class TestDirectionHysteresis:

    def test_no_direction_change_within_hyst(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.5, current_temp=19.5, dt_min=DT_MIN)
        m.filter_setpoint(target_temp=19.5 + SP_HYST + 0.01, current_temp=19.0, dt_min=DT_MIN)
        assert m._direction == "UP"
        for _ in range(5):
            m.filter_setpoint(
                target_temp=m.filtered_setpoint + SP_HYST * 0.4,
                current_temp=19.0,
                dt_min=DT_MIN,
            )
            assert m._direction == "UP"

    def test_direction_changes_when_beyond_hyst(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.5, current_temp=19.5, dt_min=DT_MIN)
        m.filter_setpoint(target_temp=19.7, current_temp=19.0, dt_min=DT_MIN)
        assert m._direction == "UP"
        m.filter_setpoint(
            target_temp=m.filtered_setpoint - SP_HYST - 0.01,
            current_temp=19.0,
            dt_min=DT_MIN,
        )
        assert m._direction == "DOWN"


# ---------------------------------------------------------------------------
# BYPASS -> FILTER transition — C0 continuity (spec §4.3, §8.1)
# ---------------------------------------------------------------------------

class TestBypassToFilterTransition:

    def test_no_jump_at_transition(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        # BYPASS: delta = 2 >= threshold
        m.filter_setpoint(target_temp=21.0, current_temp=19.0, dt_min=DT_MIN)
        # filter_state is now 21.0

        # Temperature rises: delta = 21 - 20.2 = 0.8 < threshold -> FILTER
        result = m.filter_setpoint(target_temp=21.0, current_temp=20.2, dt_min=DT_MIN)
        # SP_brut == filter_state == 21.0 -> within HYST -> EMA from 21.0 to 21.0 = 21.0
        assert abs(result - 21.0) < 1e-9, f"Jump at BYPASS->FILTER: got {result}"


# ---------------------------------------------------------------------------
# Disabled filter
# ---------------------------------------------------------------------------

class TestDisabledFilter:
    def test_passthrough_when_disabled(self):
        m = _make_manager(enabled=False)
        result = m.filter_setpoint(target_temp=22.0, current_temp=18.0, dt_min=DT_MIN)
        assert result == 22.0

    def test_passthrough_always_returns_target(self):
        m = _make_manager(enabled=False)
        for target in [19.0, 21.0, 17.5]:
            result = m.filter_setpoint(target_temp=target, current_temp=19.0, dt_min=DT_MIN)
            assert result == target


# ---------------------------------------------------------------------------
# State persistence (save / load)
# ---------------------------------------------------------------------------

class TestStatePersistence:
    def test_save_and_load_roundtrip(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=21.0, current_temp=19.0, dt_min=DT_MIN)
        m.filter_setpoint(target_temp=21.0, current_temp=20.5, dt_min=DT_MIN)
        m._direction = "DOWN"
        m._tau_f_prev = SP_TAU_FAST

        saved = m.save_state()

        m2 = _make_manager()
        m2.load_state(saved)
        assert m2.filtered_setpoint == m.filtered_setpoint
        assert m2._direction == "DOWN"
        assert m2._tau_f_prev == SP_TAU_FAST

    def test_load_state_invalid_direction_ignored(self):
        m = _make_manager()
        m.load_state({"direction": "INVALID"})
        assert m._direction == "UP"

    def test_load_state_empty(self):
        m = _make_manager()
        m.load_state({})
        assert m.filtered_setpoint is None
        assert m._direction == "UP"

    def test_save_state_does_not_include_old_keys(self):
        m = _make_manager()
        saved = m.save_state()
        assert "last_raw_setpoint" not in saved
        assert "initial_temp_for_filter" not in saved
        assert "direction" in saved
        assert "tau_f_prev" in saved


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_state(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=21.0, current_temp=19.0, dt_min=DT_MIN)
        assert m.filtered_setpoint is not None

        m.reset()
        assert m.filtered_setpoint is None
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW
        assert m.boost_active is False


# ---------------------------------------------------------------------------
# Setpoint step bypass — filter-state lag (SP_SETPOINT_JUMP_THRESHOLD)
# ---------------------------------------------------------------------------

class TestSetpointJumpBypass:
    """When the new setpoint is >= SP_SETPOINT_JUMP_THRESHOLD away from filter_state,
    bypass immediately so error_p = error_i (full proportional response).

    Scenario: room at 19.08 °C, old setpoint 19 °C (filter converged to ≈ 19.0),
    user raises setpoint to 19.5 °C.  Without bypass, error_p would be near-zero
    or negative and the P-term could not drive the system.
    """

    def test_setpoint_step_from_converged_state_bypasses(self):
        """A 0.5 °C step from a fully-converged filter state triggers bypass."""
        m = _make_manager()
        # Simulate filter converged at old setpoint 19.0 °C
        m.filtered_setpoint = 19.0
        m._direction = "UP"
        m._tau_f_prev = SP_TAU_SLOW

        # New setpoint: +0.5 °C  →  |19.5 - 19.0| = 0.5 >= SP_SETPOINT_JUMP_THRESHOLD → BYPASS
        result = m.filter_setpoint(target_temp=19.5, current_temp=19.08, dt_min=DT_MIN)
        assert result == 19.5
        assert m.filtered_setpoint == 19.5

    def test_setpoint_step_error_p_is_positive(self):
        """After bypass, error_p = target - current > 0 (P-term drives heating)."""
        m = _make_manager()
        m.filtered_setpoint = 19.0
        m._direction = "UP"

        result = m.filter_setpoint(target_temp=19.5, current_temp=19.08, dt_min=DT_MIN)
        error_p = result - 19.08
        assert error_p > 0, f"error_p must be positive to drive heating, got {error_p}"

    def test_just_below_jump_threshold_uses_filter(self):
        """A step smaller than SP_SETPOINT_JUMP_THRESHOLD stays in FILTER mode."""
        m = _make_manager()
        m.filtered_setpoint = 19.0
        m._direction = "UP"
        target = 19.0 + SP_SETPOINT_JUMP_THRESHOLD - 0.01  # just below threshold
        result = m.filter_setpoint(target_temp=target, current_temp=19.0, dt_min=DT_MIN)
        # FILTER mode: result must be < target (EMA, not bypass)
        assert result < target

    def test_exactly_at_jump_threshold_triggers_bypass(self):
        """A step exactly equal to SP_SETPOINT_JUMP_THRESHOLD triggers bypass."""
        m = _make_manager()
        m.filtered_setpoint = 19.0
        m._direction = "UP"
        target = 19.0 + SP_SETPOINT_JUMP_THRESHOLD
        result = m.filter_setpoint(target_temp=target, current_temp=19.0, dt_min=DT_MIN)
        assert result == target

    def test_bypass_closes_after_one_cycle(self):
        """After bypass, filter_state = target_temp, so next cycle |target - state| = 0
        and the jump condition no longer fires (FILTER mode resumes)."""
        m = _make_manager()
        m.filtered_setpoint = 19.0
        m._direction = "UP"

        # First cycle: bypass
        m.filter_setpoint(target_temp=19.5, current_temp=19.08, dt_min=DT_MIN)
        assert m.filtered_setpoint == 19.5

        # Second cycle: filter_state == target_temp → |target - state| = 0 < threshold → FILTER
        # EMA(19.5 → 19.5) = 19.5 exactly: filter is converged, no bypass re-fires.
        result2 = m.filter_setpoint(target_temp=19.5, current_temp=19.1, dt_min=DT_MIN)
        assert result2 == 19.5           # still at target (converged)
        assert m.filtered_setpoint == 19.5  # no second bypass
