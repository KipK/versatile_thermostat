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
        """On first call (FILTER mode), filter_state must start from min_error jump, bounded by target."""
        m = _make_manager()
        # To trigger Kick Initial, delta between target and filter_state (current)
        # must be > SP_SATURATION_THRESHOLD (1.0).
        target = 19.5
        current = 18.0
        result = m.filter_setpoint(target_temp=target, current_temp=current, dt_min=DT_MIN)
        # filter_state is initialised to current_temp=18.0.
        # Kick initial guard sets filter_state = min(19.5, 18.5) = 18.5
        # Then EMA applies from 18.5 towards 19.5
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * 18.5
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
# Saturation Guards and Instant Down
# ---------------------------------------------------------------------------

class TestSaturationGuardsAndInstantDown:
    """When the step is large, we clamp the filter state to ensure fast approach."""

    def test_large_step_up_jumps_to_fast_approach_limit(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=21.0, current_temp=19.0, dt_min=DT_MIN)
        # Filter state bounded to 21.0 - 1.0 = 20.0
        # EMA on 20.0 -> 21.0 with tau_slow
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * 21.0 + (1.0 - alpha) * 20.0
        assert abs(result - expected) < 1e-9
        assert abs(m.filtered_setpoint - expected) < 1e-9

    def test_large_step_down_returns_target_instantly(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=21.0, current_temp=21.0, dt_min=DT_MIN)
        result = m.filter_setpoint(target_temp=19.0, current_temp=21.0, dt_min=DT_MIN)
        assert result == 19.0
        assert m.filtered_setpoint == 19.0

    def test_cold_room_startup_clamps_to_target_minus_saturation(self):
        m = _make_manager()
        # Initialisation (bumpless) sets filter to current_temp = 15.0
        target = 19.0
        result = m.filter_setpoint(target_temp=target, current_temp=15.0, dt_min=DT_MIN)
        # Guard 1 triggers: 15.0 < 19.0 - 1.0 (18.0) -> state jumps to 18.0
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * 18.0
        assert abs(result - expected) < 1e-9


# ---------------------------------------------------------------------------
# FILTER mode — EMA (Soft Landing) and Instant Down
# ---------------------------------------------------------------------------

class TestFilterMode:

    def test_ema_direction_up_uses_tau_slow_no_kick(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        
        target = 19.8
        current = 19.0
        
        # We manually bypass the kick for this test to ensure EMA is tested correctly
        m.filtered_setpoint = 19.5 
        
        result = m.filter_setpoint(target_temp=target, current_temp=current, dt_min=DT_MIN)
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * target + (1.0 - alpha) * 19.5
        assert abs(result - expected) < 1e-9
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW

    def test_instant_down_small_step(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.5, current_temp=19.5, dt_min=DT_MIN)
        target = 19.2
        result = m.filter_setpoint(target_temp=target, current_temp=19.5, dt_min=DT_MIN)
        assert result == 19.2
        assert m._direction == "DOWN"
        assert m._tau_f_prev == SP_TAU_FAST

    def test_ema_progresses_over_multiple_cycles(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        target = 19.5
        current = 19.0  # Kept low
        
        # State will jump to 19.0 + 0.5 (kick initial) = 19.5 on the very first cycle
        # We test that it tracks target smoothly if we start higher than kick initial
        m.filtered_setpoint = 19.6
        target = 20.0
        state = 19.6
        for _ in range(10):
            result = m.filter_setpoint(target_temp=target, current_temp=current, dt_min=DT_MIN)
            alpha = _alpha(SP_TAU_SLOW)
            state = alpha * target + (1.0 - alpha) * state
            assert abs(result - state) < 1e-9
        assert result < target
        assert result > 19.0


# ---------------------------------------------------------------------------
# Guard logic to ensure filter doesn't lag ambiant temp and fail P error
# ---------------------------------------------------------------------------

class TestAmbiantGuard:

    def test_filter_state_not_below_current_temp_when_heating(self):
        m = _make_manager()
        m.filter_setpoint(target_temp=19.0, current_temp=19.0, dt_min=DT_MIN)
        # Target is 20.5, room is 19.4. Filter was 19.0.
        # Target (20.5) - Filter (19.0) = 1.5 > SP_SATURATION_THRESHOLD (1.0)
        # min_start_error = 1.0 / 2 = 0.5.
        # Guard 2: filter_state (19.0) < current_temp (19.4) + 0.5 (19.9) 
        # -> filter_state = min(20.5, 19.9) = 19.9
        result = m.filter_setpoint(target_temp=20.5, current_temp=19.4, dt_min=DT_MIN)
        
        # State jumps to 19.9, then EMA towards 20.5
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * 20.5 + (1.0 - alpha) * 19.9
        assert abs(result - expected) < 1e-9 


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

