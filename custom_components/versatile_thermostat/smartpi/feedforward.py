"""
Feed-Forward Gate for Smart-PI Algorithm.

Implements a simple gating mechanism on the raw feed-forward signal:
  - Step 1 (Hard Gate): Cuts FF to zero when indoor temperature is above setpoint.
  - Step 2 (Fallback): Passes FF through unchanged when no gate applies.

Note: The previous "Soft Gate" (Stage 2) has been removed as it was counter-productive
(reducing FF near setpoint caused static offset).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class FFGateResult:
    """Result of the feed-forward gate computation."""

    u_ff_eff: float
    ff_reason: str


def apply_ff_gate(
    *,
    u_ff_raw: float,
    error: float,
) -> FFGateResult:
    """Apply feed-forward gating (hard gate only).

    Pure function — no side effects, no dependency on SmartPI instance.

    Args:
        u_ff_raw: Raw FF value (already includes warmup scaling).
        error: SP - Tin (positive = below setpoint).

    Returns:
        FFGateResult with effective FF value.
    """

    # --- Step 1: Hard Gate — above setpoint ---
    if error < 0:
        return FFGateResult(
            u_ff_eff=0.0,
            ff_reason="ff_cut_above_setpoint",
        )

    # --- Step 2: Default fallback ---
    return FFGateResult(
        u_ff_eff=u_ff_raw,
        ff_reason="ff_none",
    )
