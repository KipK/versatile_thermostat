"""Tests for SmartPISetpointManager — Dual-Track filter (BOOST + Quadratic Landing)."""
import pytest

from custom_components.versatile_thermostat.smartpi.setpoint import SmartPISetpointManager
from custom_components.versatile_thermostat.smartpi.const import (
    SP_MIN_LANDING_ZONE,
    SP_MAX_LANDING_ZONE,
    SP_LANDING_ZONE_FACTOR,
    SP_LANDING_ZONE_MIN_P_FRACTION,
)


# Model parameters for tests
A_TEST = 0.01         # °C/min per duty
DEADTIME_TEST = 600.0  # seconds → raw = 0.01 * 600/60 = 0.1°C
LANDING_ZONE_TEST = A_TEST * DEADTIME_TEST / 60.0 * SP_LANDING_ZONE_FACTOR  # 0.2°C


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


# Kept for backward compatibility in tests that pass explicit landing_zone
_quadratic = _expected_sp


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

    def test_init_fallback_when_no_current_temp(self):
        m = _make_manager()
        result = _filter(m, target=22.0, current=None)
        assert result == 22.0

    def test_init_landing_phase(self):
        """First call within landing zone: returns quadratic value."""
        m = _make_manager()
        # current=18.95, target=19.0: remaining=0.05 < landing_zone(0.1) → LANDING
        result = _filter(m, target=19.0, current=18.95)
        expected = _quadratic(18.95, 19.0, LANDING_ZONE_TEST)
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
        """18→19 (remaining=1.0, landing=0.1): BOOST returns target."""
        m = _make_manager()
        result = _filter(m, target=19.0, current=18.0)
        # remaining=1.0 > landing_zone=0.1 → BOOST
        assert result == 19.0

    def test_large_step_boost(self):
        """18→21 (remaining=3.0, landing=0.1): BOOST returns target."""
        m = _make_manager()
        result = _filter(m, target=21.0, current=18.0)
        # remaining=3.0 > landing_zone=0.1 → BOOST
        assert result == 21.0

    def test_boost_effective_setpoint(self):
        """During BOOST, effective_setpoint == target."""
        m = _make_manager()
        _filter(m, target=21.0, current=19.0)
        assert m.effective_setpoint == 21.0


# ---------------------------------------------------------------------------
# LANDING phase — quadratic braking
# ---------------------------------------------------------------------------

class TestLandingPhase:
    """When remaining <= landing_zone, filter returns quadratic braking value."""

    def test_landing_returns_quadratic(self):
        """When current is within landing_zone of target, return quadratic."""
        m = _make_manager()
        # current=18.95, target=19.0: remaining=0.05 < landing_zone=0.1 → LANDING
        result = _filter(m, target=19.0, current=18.95)
        expected = _quadratic(18.95, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9

    def test_landing_reduces_p_error(self):
        """In LANDING, SP_for_P < target → P_error is reduced vs raw error."""
        m = _make_manager()
        target = 19.0
        current = 18.95
        result = _filter(m, target=target, current=current)
        # result should be < target
        assert result < target
        # P_error = result - current should be less than target - current
        assert (result - current) < (target - current)

    def test_landing_continuous_at_boundary(self):
        """At the BOOST/LANDING boundary, quadratic gives exactly target."""
        m = _make_manager()
        # current = target - landing_zone → remaining == landing_zone
        current = 19.0 - LANDING_ZONE_TEST  # 18.9
        result = _filter(m, target=19.0, current=current)
        # remaining² / landing_zone = landing_zone² / landing_zone = landing_zone
        # SP_for_P = current + landing_zone = target
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
        lz = LANDING_ZONE_TEST  # 0.1
        # Test at several points in landing zone
        p_errors = []
        for current in [18.90, 18.93, 18.96, 18.99]:
            result = _filter(m, target=target, current=current)
            p_errors.append(result - current)
        # P_error should decrease monotonically
        for i in range(len(p_errors) - 1):
            assert p_errors[i] > p_errors[i + 1], \
                f"P_error not decreasing: {p_errors[i]:.6f} vs {p_errors[i+1]:.6f}"

    def test_landing_works_for_disturbance(self):
        """Quadratic landing works for disturbance recovery (e.g., window open)."""
        m = _make_manager()
        # Setpoint stays 19, temp dropped to 18.95 (within landing zone)
        _filter(m, target=19.0, current=19.0)  # init at target
        result = _filter(m, target=19.0, current=18.95)
        expected = _quadratic(18.95, 19.0, LANDING_ZONE_TEST)
        assert abs(result - expected) < 1e-9
        assert result < 19.0


# ---------------------------------------------------------------------------
# BOOST → LANDING transition
# ---------------------------------------------------------------------------

class TestBoostToLandingTransition:
    """Verify smooth transition from BOOST to LANDING as temp approaches target."""

    def test_transition_sequence(self):
        """Simulate heating from 18→19 with landing_zone=0.1."""
        m = _make_manager()

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
            if 19.0 - t <= LANDING_ZONE_TEST and 19.0 - t > 0:
                assert results[i] < 19.0, f"Cycle {i}: expected LANDING (<19.0), got {results[i]}"

    def test_continuity_at_boundary(self):
        """Transition is continuous: just inside/outside landing zone give close values."""
        m = _make_manager()
        boundary = 19.0 - LANDING_ZONE_TEST  # 18.9

        # Just outside: BOOST → 19.0
        r_boost = _filter(m, target=19.0, current=boundary - 0.001)
        assert r_boost == 19.0

        # Just inside: LANDING → close to 19.0
        r_land = _filter(m, target=19.0, current=boundary + 0.001)
        assert abs(r_land - 19.0) < 0.01  # nearly continuous


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
        """landing_zone = a * deadtime / 60 * factor = 0.01*600/60*2 = 0.2°C."""
        m = _make_manager()
        # landing_zone = 0.2°C → LANDING starts at 18.8
        # current=18.85 → remaining=0.15 < 0.2 → LANDING
        result = _filter(m, target=19.0, current=18.85)
        assert result < 19.0  # LANDING

        # current=18.79 → remaining=0.21 > 0.2 → BOOST
        m2 = _make_manager()
        result2 = _filter(m2, target=19.0, current=18.79)
        assert result2 == 19.0  # BOOST

    def test_landing_zone_clamped_min(self):
        """Very small a*deadtime gets clamped to SP_MIN_LANDING_ZONE."""
        m = _make_manager()
        # a*deadtime/60 = 0.0001*10/60 ≈ 0.00002 → clamped to MIN (0.05)
        # current=18.96 → remaining=0.04 < 0.05 → LANDING
        result = _filter(m, target=19.0, current=18.96, a=0.0001, deadtime=10.0)
        assert result < 19.0  # LANDING

    def test_landing_zone_clamped_max(self):
        """Very large a*deadtime gets clamped to SP_MAX_LANDING_ZONE."""
        m = _make_manager()
        # a*deadtime/60 = 0.1*3600/60 = 6.0 → clamped to MAX (2.0)
        # current=23.5 → remaining=1.5 < 2.0 → LANDING
        result = _filter(m, target=25.0, current=23.5, a=0.1, deadtime=3600.0)
        assert result < 25.0  # LANDING

        # current=22.5 → remaining=2.5 > 2.0 → BOOST
        m2 = _make_manager()
        result2 = _filter(m2, target=25.0, current=22.5, a=0.1, deadtime=3600.0)
        assert result2 == 25.0  # BOOST


# ---------------------------------------------------------------------------
# Quadratic formula verification
# ---------------------------------------------------------------------------

class TestQuadraticFormula:
    def test_exact_quadratic_values(self):
        """Verify exact quadratic values at specific points."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.1

        # remaining=0.08 → SP = 19.92 + 0.08²/0.1 = 19.92 + 0.064 = 19.984
        # P_error = 19.984 - 19.92 = 0.064
        result = _filter(m, target=target, current=19.92)
        expected = 19.92 + (0.08 ** 2) / lz
        assert abs(result - expected) < 1e-9

    def test_quadratic_half_landing_zone(self):
        """At remaining = landing_zone/2, quadratic still dominates over linear floor."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.2 (with factor=2)
        current = target - lz / 2  # 19.9, remaining=0.1
        result = _filter(m, target=target, current=current)
        remaining = target - current  # 0.1
        # quadratic=0.1²/0.2=0.05, linear_floor=0.1*0.3=0.03 → quadratic wins
        expected_p_error = remaining ** 2 / lz  # 0.05
        assert abs((result - current) - expected_p_error) < 1e-9

    def test_linear_floor_near_setpoint(self):
        """Very close to setpoint, linear floor dominates over quadratic (prevents stall)."""
        m = _make_manager()
        target = 20.0
        lz = LANDING_ZONE_TEST  # 0.2
        # remaining=0.04 < lz*fraction=0.06 → linear floor kicks in
        current = target - 0.04  # 19.96
        result = _filter(m, target=target, current=current)
        remaining = 0.04
        quadratic_p_error = remaining ** 2 / lz   # 0.008
        linear_p_error = remaining * SP_LANDING_ZONE_MIN_P_FRACTION  # 0.012
        # Linear floor dominates
        assert linear_p_error > quadratic_p_error
        assert abs((result - current) - linear_p_error) < 1e-9
        # P_error is significantly larger than pure quadratic → no stall
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

        saved = m.save_state()

        m2 = _make_manager()
        m2.load_state(saved)
        assert m2.filtered_setpoint == m.filtered_setpoint

    def test_load_state_empty(self):
        m = _make_manager()
        m.load_state({})
        assert m.filtered_setpoint is None

    def test_save_state_keys(self):
        m = _make_manager()
        saved = m.save_state()
        assert "filtered_setpoint" in saved
        assert "setpoint_boost_active" in saved


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
        assert m.effective_setpoint is None
        assert m.boost_active is False
