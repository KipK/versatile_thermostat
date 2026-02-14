"""
Feed-Forward Gate for Smart-PI Algorithm.

Implements a two-stage gating mechanism on the raw feed-forward signal:
  - Step 1 (Hard Gate): Cuts FF to zero when indoor temperature is above setpoint.
  - Step 2 (Soft Gate): Progressively attenuates FF as temperature approaches setpoint,
    using deadtime-based inertia anticipation (optional, disabled by default).
  - Step 3 (Fallback): Passes FF through unchanged when no gate applies.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass

from .const import (
    clamp,
    ENABLE_FF_SOFTGATE,
    FF_SOFTGATE_D_MIN_C,
    FF_SOFTGATE_D_MAX_C,
    FF_SOFTGATE_MIN_LEARN_OK_A,
    FF_SOFTGATE_A_EPS,
    FF_SOFTGATE_S_NET_EPS,
)

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class FFGateResult:
    """Result of the feed-forward gate computation."""

    u_ff_eff: float
    ff_reason: str
    ff_scale: float | None = None
    H_inertia_s: float | None = None
    d_inertia_deg: float | None = None


def apply_ff_gate(
    *,
    u_ff_raw: float,
    error: float,
    ext_temp: float | None,
    Tin: float,
    a: float,
    b: float,
    learn_ok_count_a: int,
    tau_reliable: bool,
    deadtime_heat_s: float | None,
    deadtime_heat_reliable: bool,
    deadtime_cool_s: float | None,
    deadtime_cool_reliable: bool,
    cycle_s: float,
    enable_softgate: bool = ENABLE_FF_SOFTGATE,
) -> FFGateResult:
    """Apply feed-forward gating (hard gate + optional soft gate).

    Pure function — no side effects, no dependency on SmartPI instance.

    Args:
        u_ff_raw: Raw FF value (already includes warmup scaling).
        error: SP - Tin (positive = below setpoint).
        ext_temp: Outdoor temperature (None if unavailable).
        Tin: Indoor temperature.
        a: Learned heating efficacy (°C/(min*%)).
        b: Learned loss coefficient (°C/(min*°C)).
        learn_ok_count_a: Number of successful 'a' learning updates.
        tau_reliable: Whether tau estimation is considered reliable.
        deadtime_heat_s: Estimated heating dead time (seconds), or None.
        deadtime_heat_reliable: Whether heating dead time is reliable.
        deadtime_cool_s: Estimated cooling dead time (seconds), or None.
        deadtime_cool_reliable: Whether cooling dead time is reliable.
        cycle_s: Cycle duration in seconds.
        enable_softgate: Whether the deadtime-aware soft gate is enabled.

    Returns:
        FFGateResult with effective FF value and diagnostic fields.
    """

    # --- Step 1: Hard Gate — above setpoint ---
    if error < 0:
        return FFGateResult(
            u_ff_eff=0.0,
            ff_reason="ff_cut_above_setpoint",
        )

    # --- Step 2: Soft Gate — deadtime-aware approach (optional) ---
    if (
        enable_softgate
        and error > 0
        and ext_temp is not None
        and deadtime_heat_reliable
        and tau_reliable
        and learn_ok_count_a >= FF_SOFTGATE_MIN_LEARN_OK_A
        and a > FF_SOFTGATE_A_EPS
    ):
        # 2.1 — Inertia horizon
        if deadtime_cool_reliable and deadtime_cool_s is not None:
            L = deadtime_cool_s
        else:
            L = deadtime_heat_s  # type: ignore[assignment]

        H_inertia_s = L + cycle_s / 2.0

        # 2.2 — Net slope estimation
        delta_T = max(Tin - ext_temp, 0.0)
        s_cool = b * delta_T           # °C/min — cooling rate
        s_heat_net = a - s_cool        # °C/min — net heating rate

        if s_heat_net <= FF_SOFTGATE_S_NET_EPS:
            return FFGateResult(
                u_ff_eff=u_ff_raw,
                ff_reason="ff_softgate_fallback_slope",
            )

        s_heat_s = s_heat_net / 60.0   # °C/s

        # 2.3 — Anticipation distance
        d_inertia_deg = clamp(
            s_heat_s * H_inertia_s,
            FF_SOFTGATE_D_MIN_C,
            FF_SOFTGATE_D_MAX_C,
        )

        # 2.4 — Attenuation factor
        scale = clamp(error / d_inertia_deg, 0.0, 1.0)
        u_ff_eff = u_ff_raw * scale

        # 2.5 — Safety clamp (invariant I1)
        u_ff_eff = clamp(u_ff_eff, 0.0, u_ff_raw)

        return FFGateResult(
            u_ff_eff=u_ff_eff,
            ff_reason="ff_softgate_deadtime",
            ff_scale=scale,
            H_inertia_s=H_inertia_s,
            d_inertia_deg=d_inertia_deg,
        )

    # --- Step 3: Default fallback ---
    return FFGateResult(
        u_ff_eff=u_ff_raw,
        ff_reason="ff_none",
    )
