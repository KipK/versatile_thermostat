"""Tests for SmartPISetpointManager — State Machine Filter (Tracking vs Regulation)."""
import pytest

from custom_components.versatile_thermostat.smartpi.setpoint import SmartPISetpointManager
from custom_components.versatile_thermostat.smartpi.const import (
    SP_MIN_LANDING_ZONE,
    SP_MAX_LANDING_ZONE,
    SP_LANDING_ZONE_FACTOR,
    SP_LANDING_ZONE_MIN_P_FRACTION,
    SP_FILTER_DISABLE_THRESHOLD,
    SP_FILTER_ENABLE_THRESHOLD,
)

# Model parameters for tests
A_TEST = 0.01         # °C/min per duty
DEADTIME_TEST = 3000.0  # seconds → raw = 0.01 * 3000/60 = 0.5°C
# 0.5 * 1.6 = 0.8°C
LANDING_ZONE_TEST = A_TEST * DEADTIME_TEST / 60.0 * SP_LANDING_ZONE_FACTOR


def _make_manager(enabled: bool = True) -> SmartPISetpointManager:
    return SmartPISetpointManager(name="test", enabled=enabled)


def _filter(m, target, current, a=A_TEST, deadtime=DEADTIME_TEST):
    """Shorthand for filter_setpoint with default test model params."""
    return m.filter_setpoint(
        target_temp=target, current_temp=current,
        a=a, deadtime_cool_s=deadtime,
    )


def _expected_sp(current, target, landing_zone):
    """Expected landing value: quadratic with linear floor (mirrors filter formula)."""
    remaining = target - current
    p_error = (remaining * remaining) / landing_zone
    p_error = max(p_error, remaining * SP_LANDING_ZONE_MIN_P_FRACTION)
    return current + p_error


_quadratic = _expected_sp


# ---------------------------------------------------------------------------
# Bumpless transfer — initialisation
# ---------------------------------------------------------------------------

class TestBumplessTransfer:
    """On first call, filter_state must start from current_temp, not target_temp."""

    def test_init_from_current_temp_boost_phase(self):
        """First call with large step: BOOST phase returns target directly."""
        m = _make_manager()
        # current=18, target=19.5: remaining=1.5 > landing_zone(0.8) → BOOST
        result = _filter(m, target=19.5, current=18.0)
        assert result == 19.5  # BOOST: returns target

    def test_init_fallback_when_no_current_temp(self):
        m = _make_manager()
        result = _filter(m, target=22.0, current=None)
        assert result == 22.0

    def test_init_landing_phase(self):
        """First call within landing zone: returns quadratic value if step was large enough."""
        m = _make_manager()
        # current=18.5, target=19.0: remaining=0.5 < landing_zone(0.8). Step is 0.5 >= ENABLE_THRESHOLD
        result = _filter(m, target=19.0, current=18.5)
        expected = _quadratic(18.5, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9


# ---------------------------------------------------------------------------
# No current_temp — no update
# ---------------------------------------------------------------------------

class TestNoCurrentTemp:
    def test_no_update_when_current_temp_none(self):
        m = _make_manager()
        first = _filter(m, target=22.0, current=20.0)

        result = _filter(m, target=22.0, current=None)
        assert result == first


# ---------------------------------------------------------------------------
# BOOST phase — full power when far from target
# ---------------------------------------------------------------------------

class TestBoostPhase:
    """When remaining > landing_zone, filter returns target directly."""

    def test_small_step_boost(self):
        """18→19 (remaining=1.0, landing=0.8): BOOST returns target."""
        m = _make_manager()
        result = _filter(m, target=19.0, current=18.0)
        # remaining=1.0 > landing_zone=0.8 → BOOST
        assert result == 19.0

    def test_large_step_boost(self):
        """18→21 (remaining=3.0, landing=0.8): BOOST returns target."""
        m = _make_manager()
        result = _filter(m, target=21.0, current=18.0)
        # remaining=3.0 > landing_zone=0.8 → BOOST
        assert result == 21.0

    def test_boost_effective_setpoint(self):
        """During BOOST, effective_setpoint == target."""
        m = _make_manager()
        _filter(m, target=21.0, current=19.0)
        assert m.effective_setpoint == 21.0


# ---------------------------------------------------------------------------
# State Machine (Tracking vs Regulation / Stiffness)
# ---------------------------------------------------------------------------

class TestStateMachineStiffness:
    """Verify that filter completely deactivates near setpoint to restore P stiffness."""

    def test_deactivation_near_target(self):
        m = _make_manager()
        _filter(m, target=19.0, current=17.0) # Active tracking
        assert m.filter_active

        result = _filter(m, target=19.0, current=18.95) # remaining = 0.05 < DISABLE threshold
        assert m.filter_active is False
        assert result == 19.0 # P gets full transparent signal

    def test_small_disturbance_stiffness(self):
        """Regulation: small window opening does not reactivate the filter."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0) # lock-in
        assert m.filter_active is False

        # Temp drops to 18.7 (remaining = 0.3 < 0.5). Stiffness is preserved.
        result = _filter(m, target=19.0, current=18.7)
        assert m.filter_active is False
        assert result == 19.0

    def test_large_disturbance_reactivates(self):
        """Large temp drop (e.g. 0.6) reactivates filtering to prevent overshoot on recovery."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0) # lock-in

        result = _filter(m, target=19.0, current=18.4) # remaining=0.6 >= ENABLE
        assert m.filter_active is True
        # Since 0.6 <= lz (0.8), it's in landing zone immediately
        expected = _quadratic(18.4, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9


# ---------------------------------------------------------------------------
# LANDING phase — quadratic braking
# ---------------------------------------------------------------------------

class TestLandingPhase:
    """When remaining <= landing_zone and filter_active, filter returns quadratic braking value."""

    def test_landing_returns_quadratic(self):
        """When current is within landing_zone of target, return quadratic."""
        m = _make_manager()
        _filter(m, target=19.0, current=18.0) # activate
        # current=18.5, target=19.0: remaining=0.5 < landing_zone=0.8 → LANDING
        result = _filter(m, target=19.0, current=18.5)
        expected = _quadratic(18.5, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9

    def test_landing_reduces_p_error(self):
        """In LANDING, SP_for_P < target → P_error is reduced vs raw error."""
        m = _make_manager()
        target = 19.0
        current = 18.5
        _filter(m, target=target, current=17.0) # activate
        result = _filter(m, target=target, current=current)
        # result should be < target
        assert result < target
        # P_error = result - current should be less than target - current
        assert (result - current) < (target - current)

    def test_landing_continuous_at_boundary(self):
        """At the BOOST/LANDING boundary, quadratic gives exactly target."""
        m = _make_manager()
        # current = target - landing_zone → remaining == landing_zone
        current = 19.0 - LANDING_ZONE_TEST  # 18.2
        _filter(m, target=19.0, current=18.0) # activate
        result = _filter(m, target=19.0, current=current)
        assert abs(result - 19.0) < 1e-9

    def test_landing_at_target(self):
        """When current == target, remaining==0, SP_for_P == target."""
        m = _make_manager()
        result = _filter(m, target=19.0, current=19.0)
        assert result == 19.0

    def test_landing_power_profile(self):
        """Quadratic landing gives progressively decreasing P_error."""
        m = _make_manager()
        target = 19.0
        _filter(m, target=target, current=18.0) # activate
        # Test at several points in landing zone, all > 0.1 to stay active
        p_errors = []
        for current in [18.3, 18.5, 18.7, 18.8]:
            result = _filter(m, target=target, current=current)
            p_errors.append(result - current)
        # P_error should decrease monotonically
        for i in range(len(p_errors) - 1):
            assert p_errors[i] > p_errors[i + 1], \
                f"P_error not decreasing: {p_errors[i]:.6f} vs {p_errors[i+1]:.6f}"

    def test_landing_works_for_disturbance(self):
        """Quadratic landing works for disturbance recovery if large enough."""
        m = _make_manager()
        _filter(m, target=19.0, current=19.0)  # init at target
        result = _filter(m, target=19.0, current=18.4) # activates (remaining=0.6)
        expected = _quadratic(18.4, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9


# ---------------------------------------------------------------------------
# BOOST → LANDING transition
# ---------------------------------------------------------------------------

class TestBoostToLandingTransition:
    """Verify smooth transition from BOOST to LANDING as temp approaches target."""

    def test_transition_sequence(self):
        """Simulate heating from 17→19 with landing_zone=0.8."""
        m = _make_manager()

        # Simulate temperature rising
        temps = [17.0, 18.1, 18.3, 18.6, 18.8, 18.92, 18.96]
        results = []
        for t in temps:
            r = _filter(m, target=19.0, current=t)
            results.append(r)

        # BOOST cycles (remaining > 0.8)
        for i, t in enumerate(temps):
            if 19.0 - t > LANDING_ZONE_TEST:
                assert results[i] == 19.0, f"Cycle {i} (t={t}): expected BOOST, got {results[i]}"

        # LANDING cycles (0.1 < remaining <= 0.8)
        for i, t in enumerate(temps):
            remaining = 19.0 - t
            if remaining <= LANDING_ZONE_TEST and remaining > SP_FILTER_DISABLE_THRESHOLD:
                assert results[i] < 19.0, f"Cycle {i} (t={t}): expected LANDING (<19.0), got {results[i]}"

        # DEACTIVATED cycles (remaining <= 0.1)
        for i, t in enumerate(temps):
            remaining = 19.0 - t
            if remaining <= SP_FILTER_DISABLE_THRESHOLD:
                assert results[i] == 19.0, f"Cycle {i} (t={t}): expected DEACTIVATED (19.0), got {results[i]}"


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

    def test_instant_down_small_step(self):
        m = _make_manager()
        _filter(m, target=19.5, current=19.5)
        result = _filter(m, target=19.2, current=19.5)
        assert result == 19.2


# ---------------------------------------------------------------------------
# Landing zone computation
# ---------------------------------------------------------------------------

class TestLandingZoneComputation:

    def test_landing_zone_from_model(self):
        """landing_zone = a * deadtime / 60 * factor = 0.01*3000/60*1.6 = 0.8°C."""
        m = _make_manager()
        _filter(m, target=19.0, current=17.0) # activate
        # LANDING starts at 18.2
        # current=18.5 → remaining=0.5 < 0.8 → LANDING
        result = _filter(m, target=19.0, current=18.5)
        assert result < 19.0  # LANDING

    def test_landing_zone_clamped_min(self):
        """Very small a*deadtime gets clamped to SP_MIN_LANDING_ZONE (0.05)."""
        m = _make_manager()
        result = _filter(m, target=19.0, current=18.4, a=0.0001, deadtime=10.0)
        # landing zone is clamped to 0.05. However remaining=0.6 > 0.05, so it's BOOST.
        assert result == 19.0

    def test_landing_zone_clamped_max(self):
        """Very large a*deadtime gets clamped to SP_MAX_LANDING_ZONE."""
        m = _make_manager()
        # a*deadtime/60 = 0.1*3600/60 = 6.0 → clamped to MAX (2.0)
        _filter(m, target=25.0, current=15.0, a=0.1, deadtime=3600.0) # activate
        # current=23.5 → remaining=1.5 < 2.0 → LANDING
        result = _filter(m, target=25.0, current=23.5, a=0.1, deadtime=3600.0)
        assert result < 25.0  # LANDING


# ---------------------------------------------------------------------------
# Quadratic formula verification
# ---------------------------------------------------------------------------

class TestQuadraticFormula:
    def test_exact_quadratic_values(self):
        """Verify exact quadratic values at specific points."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.8
        _filter(m, target=target, current=17.0) # activate

        # remaining=0.4 → SP = 19.6 + 0.4²/0.8 = 19.6 + 0.16/0.8 = 19.6 + 0.2 = 19.8
        result = _filter(m, target=target, current=19.6)
        expected = 19.6 + (0.4 ** 2) / lz
        assert abs(result - expected) < 1e-9

    def test_quadratic_half_landing_zone(self):
        """At remaining = landing_zone/2, quadratic dominates over linear floor."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.8
        _filter(m, target=target, current=17.0) # activate

        current = target - lz / 2  # 19.6, remaining=0.4
        result = _filter(m, target=target, current=current)
        # expected p_error is 0.2
        assert abs((result - current) - 0.2) < 1e-9

    def test_linear_floor_near_setpoint(self):
        """Very close to setpoint, linear floor dominates over quadratic (prevents stall)."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.8
        _filter(m, target=target, current=17.0) # activate

        # remaining=0.15 < lz*fraction=0.8*0.3=0.24 → linear floor kicks in
        current = target - 0.15  # 19.85
        result = _filter(m, target=target, current=current)
        remaining = 0.15
        quadratic_p_error = remaining ** 2 / lz   # 0.028125
        linear_p_error = remaining * SP_LANDING_ZONE_MIN_P_FRACTION  # 0.045
        # Linear floor dominates
        assert linear_p_error > quadratic_p_error
        assert abs((result - current) - linear_p_error) < 1e-9
        assert (result - current) > quadratic_p_error


# ---------------------------------------------------------------------------
# Target at or below current temp
# ---------------------------------------------------------------------------

class TestTargetAtOrBelowCurrent:
    def test_target_equals_current(self):
        m = _make_manager()
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


# ---------------------------------------------------------------------------
# State persistence (save / load)
# ---------------------------------------------------------------------------

class TestStatePersistence:
    def test_save_and_load_roundtrip(self):
        m = _make_manager()
        _filter(m, target=21.0, current=19.0)
        saved = m.save_state()

        m2 = _make_manager()
        m2.load_state(saved)
        assert m2.filtered_setpoint == m.filtered_setpoint
        assert m2.filter_active == m.filter_active

    def test_load_state_empty(self):
        m = _make_manager()
        m.load_state({})
        assert m.filtered_setpoint is None

    def test_save_state_keys(self):
        m = _make_manager()
        saved = m.save_state()
        assert "filtered_setpoint" in saved
        assert "setpoint_filter_active" in saved


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

class TestReset:
    def test_reset_clears_state(self):
        m = _make_manager()
        _filter(m, target=21.0, current=19.0)
        m.reset()
        assert m.filtered_setpoint is None
        assert m.effective_setpoint is None
        assert m.filter_active is False
