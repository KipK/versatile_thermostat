"""Tests for the Feed-Forward Gate module (smartpi/feedforward.py)."""

import pytest
from custom_components.versatile_thermostat.smartpi.feedforward import (
    apply_ff_gate,
    FFGateResult,
)


# ================================================================
# Step 1 — Hard Gate Tests
# ================================================================


class TestHardGate:
    """Step 1: error < -near_band_above_deg => u_ff_eff = 0."""

    def test_above_setpoint_cuts_ff(self):
        result = apply_ff_gate(
            u_ff_raw=0.15,
            error=-0.5,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.0
        assert result.ff_reason == "ff_cut_above_setpoint"

    def test_at_setpoint_exact_no_cut(self):
        """error == 0 should NOT trigger hard gate."""
        result = apply_ff_gate(
            u_ff_raw=0.10,
            error=0.0,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.10
        assert result.ff_reason == "ff_none"

    def test_in_near_band_above_no_cut(self):
        """error in (-near_band_above_deg, 0) should NOT trigger hard gate."""
        result = apply_ff_gate(
            u_ff_raw=0.10,
            error=-0.1,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.10
        assert result.ff_reason == "ff_none"

    def test_hard_gate_zero_raw(self):
        """Hard gate with u_ff_raw=0 should still report correct reason."""
        result = apply_ff_gate(
            u_ff_raw=0.0,
            error=-1.0,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.0
        assert result.ff_reason == "ff_cut_above_setpoint"


# ================================================================
# Step 2 — Fallback Tests
# ================================================================


class TestFallback:
    """Step 2: Fallback (FF passes through unchanged)."""

    def test_fallback_positive_error(self):
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=1.0,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"

    def test_fallback_small_error(self):
        result = apply_ff_gate(
            u_ff_raw=0.20,
            error=0.05,
            near_band_above_deg=0.3,
        )
        assert result.u_ff_eff == 0.20
        assert result.ff_reason == "ff_none"


# ================================================================
# Invariant I1 — 0 <= u_ff_eff <= u_ff_raw
# ================================================================


class TestInvariantI1:
    """Invariant: 0 <= u_ff_eff <= u_ff_raw in all cases."""

    @pytest.mark.parametrize("error", [-2.0, -0.5, 0.0, 0.05, 0.5, 2.0, 5.0])
    @pytest.mark.parametrize("u_ff_raw", [0.0, 0.04, 0.20, 0.50, 1.0])
    def test_invariant(self, error, u_ff_raw):
        result = apply_ff_gate(
            u_ff_raw=u_ff_raw,
            error=error,
            near_band_above_deg=0.3,
        )
        assert 0.0 <= result.u_ff_eff <= u_ff_raw
