"""Tests for SmartPISetpointManager — Dual-Track filter (BOOST + EMA Landing)."""
import pytest

from custom_components.versatile_thermostat.smartpi.setpoint import SmartPISetpointManager
from custom_components.versatile_thermostat.smartpi.const import (
    SP_TAU_SLOW,
    SP_TAU_FAST,
    SP_MIN_LANDING_ZONE,
    SP_MAX_LANDING_ZONE,
    SP_HYST,
)


DT_MIN = 1.0  # 1-minute cycle (= 60 s)
DT_S = DT_MIN * 60.0

# Model parameters for tests
A_TEST = 0.01         # °C/min per duty
DEADTIME_TEST = 600.0  # seconds → landing_zone = 0.01 * 600/60 = 0.1°C
LANDING_ZONE_TEST = A_TEST * DEADTIME_TEST / 60.0  # 0.1°C


def _alpha(tau_s: float, dt_s: float = DT_S) -> float:
    """Expected EMA alpha for a given tau (seconds)."""
    return dt_s / (tau_s + dt_s)


def _make_manager(enabled: bool = True) -> SmartPISetpointManager:
    return SmartPISetpointManager(name="test", enabled=enabled)


def _filter(m, target, current, dt_min=DT_MIN, tau_up=SP_TAU_SLOW, a=A_TEST, deadtime=DEADTIME_TEST):
    """Shorthand for filter_setpoint with default test model params."""
    return m.filter_setpoint(
        target_temp=target, current_temp=current, dt_min=dt_min,
        tau_up=tau_up, a=a, deadtime_cool_s=deadtime,
    )


# ---------------------------------------------------------------------------
# Bumpless transfer — initialisation
# ---------------------------------------------------------------------------

class TestBumplessTransfer:
    """On first call, filter_state must start from current_temp, not target_temp."""

    def test_init_from_current_temp_boost_phase(self):
        """First call with large step: BOOST phase returns target directly."""
        m = _make_manager()
        # current=18, target=19.5: remaining=1.5 > landing_zone(0.1) → BOOST
        result = _filter(m, target=19.5, current=18.0)
        assert result == 19.5  # BOOST: returns target
        # But filter_state is EMA from 18.0 towards 19.5
        alpha = _alpha(SP_TAU_SLOW)
        expected_state = alpha * 19.5 + (1.0 - alpha) * 18.0
        assert abs(m.filtered_setpoint - expected_state) < 1e-9

    def test_init_fallback_when_no_current_temp(self):
        m = _make_manager()
        result = _filter(m, target=22.0, current=None)
        assert result == 22.0
        assert m.filtered_setpoint == 22.0

    def test_direction_and_tau_initialized(self):
        m = _make_manager()
        _filter(m, target=22.0, current=19.0)
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW


# ---------------------------------------------------------------------------
# No current_temp — no update
# ---------------------------------------------------------------------------

class TestNoCurrentTemp:
    def test_no_update_when_current_temp_none(self):
        m = _make_manager()
        _filter(m, target=22.0, current=20.0)
        first_state = m.filtered_setpoint

        result = _filter(m, target=22.0, current=None)
        assert result == first_state
        assert m.filtered_setpoint == first_state


# ---------------------------------------------------------------------------
# BOOST phase — full power when far from target
# ---------------------------------------------------------------------------

class TestBoostPhase:
    """When remaining > landing_zone, filter returns target directly."""

    def test_small_step_boost(self):
        """18→19 (step=1.0, landing=0.1): BOOST returns target."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0)  # init
        result = _filter(m, target=19.0, current=18.0)
        # remaining=1.0 > landing_zone=0.1 → BOOST
        assert result == 19.0

    def test_large_step_boost(self):
        """18→21 (step=3.0, landing=0.1): BOOST returns target."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0)  # init
        result = _filter(m, target=21.0, current=18.0)
        # remaining=3.0 > landing_zone=0.1 → BOOST
        assert result == 21.0

    def test_boost_with_ema_running_internally(self):
        """During BOOST, filter_state progresses via EMA internally."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0)  # init at 19.0
        _filter(m, target=21.0, current=19.0)  # BOOST, but EMA runs

        alpha = _alpha(SP_TAU_SLOW)
        expected_state = alpha * 21.0 + (1.0 - alpha) * 19.0
        assert abs(m.filtered_setpoint - expected_state) < 1e-9


# ---------------------------------------------------------------------------
# LANDING phase — soft approach via EMA
# ---------------------------------------------------------------------------

class TestLandingPhase:
    """When remaining <= landing_zone, filter returns EMA output."""

    def test_landing_returns_ema(self):
        """When current is within landing_zone of target, return EMA state."""
        m = _make_manager()
        # Set filter_state to something below target
        m.filtered_setpoint = 18.8
        m._direction = "UP"
        # current=18.95, target=19.0: remaining=0.05 < landing_zone=0.1 → LANDING
        result = _filter(m, target=19.0, current=18.95)
        alpha = _alpha(SP_TAU_SLOW)
        expected = alpha * 19.0 + (1.0 - alpha) * 18.8
        assert abs(result - expected) < 1e-9

    def test_landing_reduces_p_error(self):
        """In LANDING, SP_for_P < target → P_error is reduced vs raw error."""
        m = _make_manager()
        m.filtered_setpoint = 18.85
        m._direction = "UP"
        target = 19.0
        current = 18.95
        result = _filter(m, target=target, current=current)
        # result should be < target (EMA lags behind)
        assert result < target
        # P_error = result - current should be less than target - current
        assert (result - current) < (target - current)


# ---------------------------------------------------------------------------
# BOOST → LANDING transition
# ---------------------------------------------------------------------------

class TestBoostToLandingTransition:
    """Verify smooth transition from BOOST to LANDING as temp approaches target."""

    def test_transition_sequence(self):
        """Simulate heating from 18→19 with landing_zone=0.1."""
        m = _make_manager()
        _filter(m, target=19.0, current=18.0)  # init + first cycle

        # Simulate temperature rising
        temps = [18.0, 18.3, 18.5, 18.7, 18.85, 18.92, 18.96]
        results = []
        for t in temps:
            r = _filter(m, target=19.0, current=t)
            results.append(r)

        # All BOOST cycles should return 19.0 (remaining > 0.1)
        for i, t in enumerate(temps):
            if 19.0 - t > LANDING_ZONE_TEST:
                assert results[i] == 19.0, f"Cycle {i}: expected BOOST (target=19.0), got {results[i]}"

        # LANDING cycles should return < 19.0
        for i, t in enumerate(temps):
            if 19.0 - t <= LANDING_ZONE_TEST:
                assert results[i] < 19.0, f"Cycle {i}: expected LANDING (<19.0), got {results[i]}"


# ---------------------------------------------------------------------------
# Instant Drop
# ---------------------------------------------------------------------------

class TestInstantDrop:
    def test_drop_returns_target_instantly(self):
        m = _make_manager()
        _filter(m, target=21.0, current=21.0)
        result = _filter(m, target=19.0, current=21.0)
        assert result == 19.0
        assert m.filtered_setpoint == 19.0
        assert m._direction == "DOWN"
        assert m._tau_f_prev == SP_TAU_FAST

    def test_instant_down_small_step(self):
        m = _make_manager()
        _filter(m, target=19.5, current=19.5)
        result = _filter(m, target=19.2, current=19.5)
        assert result == 19.2
        assert m._direction == "DOWN"


# ---------------------------------------------------------------------------
# Landing zone computation
# ---------------------------------------------------------------------------

class TestLandingZoneComputation:

    def test_landing_zone_from_model(self):
        """landing_zone = a * deadtime / 60."""
        m = _make_manager()
        # Init with filter_state below target so EMA has lag
        m.filtered_setpoint = 18.5

        # a=0.01, deadtime=600 → landing=0.1
        # current=18.91 → remaining=0.09 < 0.1 → LANDING
        result = _filter(m, target=19.0, current=18.91)
        assert result < 19.0  # LANDING (EMA from 18.5 toward 19.0)

        # current=18.89 → remaining=0.11 > 0.1 → BOOST
        m2 = _make_manager()
        m2.filtered_setpoint = 18.5
        result2 = _filter(m2, target=19.0, current=18.89)
        assert result2 == 19.0  # BOOST

    def test_landing_zone_clamped_min(self):
        """Very small a*deadtime gets clamped to SP_MIN_LANDING_ZONE."""
        m = _make_manager()
        # Init with filter_state below target so EMA has lag
        m.filtered_setpoint = 18.5
        # a*deadtime/60 = 0.0001*10/60 ≈ 0.00002 → clamped to MIN (0.05)
        # current=18.96 → remaining=0.04 < 0.05 → LANDING
        result = _filter(m, target=19.0, current=18.96, a=0.0001, deadtime=10.0)
        assert result < 19.0  # LANDING

    def test_landing_zone_clamped_max(self):
        """Very large a*deadtime gets clamped to SP_MAX_LANDING_ZONE."""
        m = _make_manager()
        _filter(m, target=25.0, current=20.0, a=0.1, deadtime=3600.0)
        # a*deadtime/60 = 0.1*3600/60 = 6.0 → clamped to MAX (2.0)
        # current=23.5 → remaining=1.5 < 2.0 → LANDING
        result = _filter(m, target=25.0, current=23.5, a=0.1, deadtime=3600.0)
        assert result < 25.0  # LANDING

        # current=22.5 → remaining=2.5 > 2.0 → BOOST
        m2 = _make_manager()
        _filter(m2, target=25.0, current=20.0, a=0.1, deadtime=3600.0)
        result2 = _filter(m2, target=25.0, current=22.5, a=0.1, deadtime=3600.0)
        assert result2 == 25.0  # BOOST


# ---------------------------------------------------------------------------
# EMA convergence over multiple cycles
# ---------------------------------------------------------------------------

class TestEMAConvergence:
    def test_ema_progresses_over_multiple_cycles(self):
        """Filter state converges toward target over multiple cycles."""
        m = _make_manager()
        m.filtered_setpoint = 19.6
        m._direction = "UP"
        target = 20.0
        current = 19.95  # within landing zone (remaining=0.05 < 0.1)
        state = 19.6
        for _ in range(10):
            result = _filter(m, target=target, current=current)
            alpha = _alpha(SP_TAU_SLOW)
            state = alpha * target + (1.0 - alpha) * state
            assert abs(result - state) < 1e-9
        assert result < target
        assert result > 19.6


# ---------------------------------------------------------------------------
# Target at or below current temp
# ---------------------------------------------------------------------------

class TestTargetAtOrBelowCurrent:
    def test_target_equals_current(self):
        m = _make_manager()
        _filter(m, target=19.0, current=19.0)
        result = _filter(m, target=19.0, current=19.0)
        assert result == 19.0

    def test_target_below_current_during_rise(self):
        """If current > target during a rise (overshoot), return target."""
        m = _make_manager()
        _filter(m, target=19.0, current=18.0)
        result = _filter(m, target=19.0, current=19.1)
        # remaining = -0.1 ≤ 0 → return target
        assert result == 19.0


# ---------------------------------------------------------------------------
# Disabled filter
# ---------------------------------------------------------------------------

class TestDisabledFilter:
    def test_passthrough_when_disabled(self):
        m = _make_manager(enabled=False)
        result = _filter(m, target=22.0, current=18.0)
        assert result == 22.0

    def test_passthrough_always_returns_target(self):
        m = _make_manager(enabled=False)
        for target in [19.0, 21.0, 17.5]:
            result = _filter(m, target=target, current=19.0)
            assert result == target


# ---------------------------------------------------------------------------
# State persistence (save / load)
# ---------------------------------------------------------------------------

class TestStatePersistence:
    def test_save_and_load_roundtrip(self):
        m = _make_manager()
        _filter(m, target=21.0, current=19.0)
        _filter(m, target=21.0, current=20.5)
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
        _filter(m, target=21.0, current=19.0)
        assert m.filtered_setpoint is not None

        m.reset()
        assert m.filtered_setpoint is None
        assert m._direction == "UP"
        assert m._tau_f_prev == SP_TAU_SLOW
        assert m.boost_active is False
