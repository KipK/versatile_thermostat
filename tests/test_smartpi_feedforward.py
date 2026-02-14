"""Tests for the Feed-Forward Gate module (smartpi/feedforward.py)."""

import pytest
from custom_components.versatile_thermostat.smartpi.feedforward import (
    apply_ff_gate,
    FFGateResult,
)


# ---------- Common kwargs for a "fully reliable" scenario ----------
RELIABLE_KWARGS = dict(
    ext_temp=5.0,
    Tin=19.0,
    a=0.020,          # °C/(min*%) — high enough so s_heat_net > 0
    b=0.0005,         # °C/(min*°C) — moderate loss coefficient
    learn_ok_count_a=20,
    tau_reliable=True,
    deadtime_heat_s=300.0,
    deadtime_heat_reliable=True,
    deadtime_cool_s=180.0,
    deadtime_cool_reliable=True,
    cycle_s=600.0,
)


# ================================================================
# Step 1 — Hard Gate Tests
# ================================================================


class TestHardGate:
    """Step 1: error < 0 => u_ff_eff = 0."""

    def test_above_setpoint_cuts_ff(self):
        result = apply_ff_gate(
            u_ff_raw=0.15,
            error=-0.5,
            **RELIABLE_KWARGS,
        )
        assert result.u_ff_eff == 0.0
        assert result.ff_reason == "ff_cut_above_setpoint"
        assert result.ff_scale is None

    def test_at_setpoint_exact_no_cut(self):
        """error == 0 should NOT trigger hard gate (error < 0 only)."""
        result = apply_ff_gate(
            u_ff_raw=0.10,
            error=0.0,
            enable_softgate=False,
            **RELIABLE_KWARGS,
        )
        assert result.u_ff_eff == 0.10
        assert result.ff_reason == "ff_none"

    def test_hard_gate_zero_raw(self):
        """Hard gate with u_ff_raw=0 should still report correct reason."""
        result = apply_ff_gate(
            u_ff_raw=0.0,
            error=-1.0,
            **RELIABLE_KWARGS,
        )
        assert result.u_ff_eff == 0.0
        assert result.ff_reason == "ff_cut_above_setpoint"


# ================================================================
# Step 3 — Fallback (softgate disabled) Tests
# ================================================================


class TestFallback:
    """Step 3: When softgate is disabled, FF passes through unchanged."""

    def test_softgate_disabled_passthrough(self):
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=False,
            **RELIABLE_KWARGS,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"
        assert result.ff_scale is None

    def test_default_softgate_is_disabled(self):
        """Default enable_softgate should be False."""
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            **RELIABLE_KWARGS,
        )
        assert result.ff_reason == "ff_none"


# ================================================================
# Step 2 — Soft Gate Tests
# ================================================================


class TestSoftGate:
    """Step 2: Deadtime-aware soft gate tests."""

    def test_missing_ext_temp_fallback(self):
        kwargs = {**RELIABLE_KWARGS, "ext_temp": None}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_unreliable_deadtime_heat_fallback(self):
        kwargs = {**RELIABLE_KWARGS, "deadtime_heat_reliable": False}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_unreliable_tau_fallback(self):
        kwargs = {**RELIABLE_KWARGS, "tau_reliable": False}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_low_learn_count_fallback(self):
        kwargs = {**RELIABLE_KWARGS, "learn_ok_count_a": 5}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_a_too_small_fallback(self):
        kwargs = {**RELIABLE_KWARGS, "a": 0.0}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_non_positive_net_slope_fallback(self):
        """When s_heat_net <= eps (a very small, b*deltaT large), fallback."""
        kwargs = {**RELIABLE_KWARGS, "a": 0.0001, "b": 0.01, "Tin": 20.0, "ext_temp": 0.0}
        # s_cool = 0.01 * 20 = 0.2, s_heat_net = 0.0001 - 0.2 < 0
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            enable_softgate=True,
            **kwargs,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_softgate_fallback_slope"

    def test_large_error_scale_near_one(self):
        """Far below setpoint => scale ~1, FF nearly unchanged."""
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=5.0,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        assert result.ff_reason == "ff_softgate_deadtime"
        assert result.ff_scale is not None
        assert result.ff_scale >= 0.99
        assert result.u_ff_eff == pytest.approx(0.20, abs=0.01)

    def test_small_error_scale_reduced(self):
        """Close to setpoint => scale < 1, FF reduced."""
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.05,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        assert result.ff_reason == "ff_softgate_deadtime"
        assert result.ff_scale is not None
        assert result.ff_scale < 1.0
        assert result.u_ff_eff < 0.20

    def test_zero_error_not_reached_because_hard_gate(self):
        """error=0 with softgate enabled: softgate requires error > 0, so fallback."""
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.0,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        # error == 0 => softgate condition (error > 0) not met => fallback
        assert result.ff_reason == "ff_none"
        assert result.u_ff_eff == 0.20

    def test_uses_deadtime_cool_when_reliable(self):
        """When deadtime_cool is reliable, L = deadtime_cool_s."""
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.5,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        assert result.H_inertia_s is not None
        # L = deadtime_cool_s (180) + cycle_s/2 (300) = 480
        assert result.H_inertia_s == pytest.approx(480.0)

    def test_uses_deadtime_heat_as_fallback(self):
        """When deadtime_cool is unreliable, L = deadtime_heat_s."""
        kwargs = {**RELIABLE_KWARGS, "deadtime_cool_reliable": False}
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.5,
            enable_softgate=True,
            **kwargs,
        )
        assert result.H_inertia_s is not None
        # L = deadtime_heat_s (300) + cycle_s/2 (300) = 600
        assert result.H_inertia_s == pytest.approx(600.0)

    def test_diagnostic_fields_populated(self):
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.5,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        assert result.ff_scale is not None
        assert result.H_inertia_s is not None
        assert result.d_inertia_deg is not None
        assert result.d_inertia_deg > 0


# ================================================================
# Invariant I1 — 0 <= u_ff_eff <= u_ff_raw
# ================================================================


class TestInvariantI1:
    """Invariant: 0 <= u_ff_eff <= u_ff_raw in all cases."""

    @pytest.mark.parametrize("error", [-2.0, -0.5, 0.0, 0.05, 0.5, 2.0, 5.0])
    @pytest.mark.parametrize("u_ff_raw", [0.0, 0.04, 0.20, 0.50, 1.0])
    def test_invariant_softgate_enabled(self, error, u_ff_raw):
        result = apply_ff_gate(
            u_ff_raw=u_ff_raw,
            error=error,
            enable_softgate=True,
            **RELIABLE_KWARGS,
        )
        assert 0.0 <= result.u_ff_eff <= u_ff_raw

    @pytest.mark.parametrize("error", [-2.0, -0.5, 0.0, 0.05, 0.5, 2.0, 5.0])
    @pytest.mark.parametrize("u_ff_raw", [0.0, 0.04, 0.20, 0.50, 1.0])
    def test_invariant_softgate_disabled(self, error, u_ff_raw):
        result = apply_ff_gate(
            u_ff_raw=u_ff_raw,
            error=error,
            enable_softgate=False,
            **RELIABLE_KWARGS,
        )
        assert 0.0 <= result.u_ff_eff <= u_ff_raw
