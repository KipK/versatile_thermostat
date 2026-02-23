"""Setpoint filtering for Smart-PI: Saturation Guard + asymmetric first-order low-pass."""
from __future__ import annotations

import logging
from typing import Optional

from .const import (
    SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN,
    SP_TAU_SLOW,
    SP_TAU_FAST,
    SP_SATURATION_THRESHOLD,
    SP_SETPOINT_JUMP_THRESHOLD,
    SP_HYST,
)
from ..vtherm_hvac_mode import VThermHvacMode

_LOGGER = logging.getLogger(__name__)


class SmartPISetpointManager:
    """
    Setpoint Management for Smart-PI.

    Responsible for:
    1. Setpoint filtering — Saturation Guard + asymmetric first-order low-pass filter.
       - BYPASS mode  : |SP_brut - y| >= SATURATION_THRESHOLD  →  SP_for_P = SP_brut
       - FILTER mode  : |SP_brut - y| <  SATURATION_THRESHOLD  →  SP_for_P tracks via EMA
    2. Setpoint boost detection (detecting manual overrides).
    """

    def __init__(self, name: str, enabled: bool = True):
        self._name = name
        self.enabled = enabled

        # Filter state (spec: filter_state)
        self.filtered_setpoint: Optional[float] = None

        # Direction tracking with hysteresis
        self._direction: str = "UP"
        self._tau_f_prev: float = SP_TAU_SLOW

        # Boost state
        self.boost_active: bool = False
        self.prev_setpoint_for_boost: Optional[float] = None

    def reset(self):
        """Reset internal state."""
        self.filtered_setpoint = None
        self._direction = "UP"
        self._tau_f_prev = SP_TAU_SLOW
        self.boost_active = False
        self.prev_setpoint_for_boost = None

    def load_state(self, state: dict):
        """Load state from persistence."""
        if not state:
            return

        fs = state.get("filtered_setpoint")
        if fs is not None:
            self.filtered_setpoint = float(fs)

        direction = state.get("direction")
        if direction in ("UP", "DOWN"):
            self._direction = direction

        tau_f_prev = state.get("tau_f_prev")
        if tau_f_prev is not None:
            self._tau_f_prev = float(tau_f_prev)

        self.boost_active = bool(state.get("setpoint_boost_active", False))

        ps = state.get("prev_setpoint_for_boost")
        if ps is not None:
            self.prev_setpoint_for_boost = float(ps)

    def save_state(self) -> dict:
        """Save state for persistence."""
        return {
            "filtered_setpoint": self.filtered_setpoint,
            "direction": self._direction,
            "tau_f_prev": self._tau_f_prev,
            "setpoint_boost_active": self.boost_active,
            "prev_setpoint_for_boost": self.prev_setpoint_for_boost,
        }

    def filter_setpoint(
        self,
        target_temp: float,
        current_temp: float | None,
        dt_min: float,
    ) -> float:
        """
        Apply Saturation Guard + asymmetric first-order low-pass filter to setpoint.

        Returns SP_for_P — the filtered setpoint for the proportional term.
        The integrator must always use SP_brut (target_temp), not this return value.

        Args:
            target_temp:  Raw setpoint (SP_brut).
            current_temp: Measured temperature. If None, no update is performed.
            dt_min:       Elapsed time since last call, in minutes.
        """
        if not self.enabled:
            self.filtered_setpoint = target_temp
            return target_temp

        # Bumpless transfer on first call (spec §6.5): initialise filter state from
        # current temperature so the first step starts without a discontinuity.
        # Do NOT return early — continue immediately into the Saturation Guard so
        # the first calculate() call already produces a meaningful SP_for_P.
        if self.filtered_setpoint is None:
            self.filtered_setpoint = current_temp if current_temp is not None else target_temp
            self._direction = "UP"
            self._tau_f_prev = SP_TAU_SLOW

        # No temperature measurement — keep current filter state
        if current_temp is None:
            return self.filtered_setpoint

        dt_s = dt_min * 60.0  # convert minutes to seconds

        # ── Step 1: True error ──
        delta_sp = target_temp - current_temp

        # ── BYPASS mode: large physical error OR filter state lags behind new setpoint ──
        # The second condition catches setpoint steps (e.g. +0.5 °C from a converged state)
        # where filter_state ≈ old_setpoint < current_temp: without bypass, error_p would
        # be near-zero or negative, making the proportional term unable to drive the system.
        if (abs(delta_sp) >= SP_SATURATION_THRESHOLD or
                abs(target_temp - self.filtered_setpoint) >= SP_SETPOINT_JUMP_THRESHOLD):
            self.filtered_setpoint = target_temp
            return target_temp

        # ── FILTER mode: asymmetric EMA with direction hysteresis ──
        filter_state = self.filtered_setpoint

        if target_temp >= filter_state + SP_HYST:
            self._direction = "UP"
        elif target_temp <= filter_state - SP_HYST:
            self._direction = "DOWN"
        # else: keep previous direction (hysteresis)

        tau_f = SP_TAU_SLOW if self._direction == "UP" else SP_TAU_FAST
        alpha = dt_s / (tau_f + dt_s)
        self.filtered_setpoint = alpha * target_temp + (1.0 - alpha) * filter_state
        self._tau_f_prev = tau_f

        return self.filtered_setpoint

    def update_boost_state(  # pylint: disable=unused-argument
        self, target_temp: float, error: float, hvac_mode: VThermHvacMode
    ) -> bool:
        """Check and update boost state based on setpoint changes."""
        if self.prev_setpoint_for_boost is None:
            self.prev_setpoint_for_boost = target_temp

        sp_delta = target_temp - self.prev_setpoint_for_boost

        # Activate boost on significant change
        if abs(sp_delta) >= SETPOINT_BOOST_THRESHOLD:
            self.boost_active = True
            self.prev_setpoint_for_boost = target_temp
            _LOGGER.debug("%s - Boost activate: delta=%.2f", self._name, sp_delta)
        elif abs(sp_delta) > 0.01:
            # Just track
            self.prev_setpoint_for_boost = target_temp
            self.boost_active = False

        # Deactivate boost when error is small
        if self.boost_active and abs(error) < SETPOINT_BOOST_ERROR_MIN:
            self.boost_active = False
            _LOGGER.debug("%s - Boost deactivate: error=%.3f", self._name, abs(error))

        return self.boost_active
