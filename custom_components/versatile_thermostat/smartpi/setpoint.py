"""Setpoint filtering for Smart-PI: Dual-Track (BOOST + Quadratic Landing)."""
from __future__ import annotations

import logging
from typing import Optional

from .const import (
    SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN,
    SP_MIN_LANDING_ZONE,
    SP_MAX_LANDING_ZONE,
    SP_LANDING_ZONE_FACTOR,
    SP_LANDING_ZONE_MIN_P_FRACTION,
)
from ..vtherm_hvac_mode import VThermHvacMode

_LOGGER = logging.getLogger(__name__)


class SmartPISetpointManager:
    """
    Setpoint Management for Smart-PI.

    Responsible for:
    1. Setpoint filtering — Dual-Track filter (BOOST + Quadratic Landing).
       - BOOST phase  : current_temp far from target → SP_for_P = target (full power)
       - LANDING phase: current_temp within landing zone →
         SP_for_P = current + remaining²/landing_zone (quadratic braking)
       Stateless in LANDING: depends only on current_temp and target.
       Works for both setpoint changes AND disturbance recovery.
    2. Setpoint boost detection (detecting manual overrides).
    """

    def __init__(self, name: str, enabled: bool = True):
        self._name = name
        self.enabled = enabled

        # Tracks last known target for drop detection
        self.filtered_setpoint: Optional[float] = None
        # Actual SP_for_P value returned to the controller (for diagnostics)
        self.effective_setpoint: Optional[float] = None

        # Boost state
        self.boost_active: bool = False
        self.prev_setpoint_for_boost: Optional[float] = None

    def reset(self):
        """Reset internal state."""
        self.filtered_setpoint = None
        self.effective_setpoint = None
        self.boost_active = False
        self.prev_setpoint_for_boost = None

    def load_state(self, state: dict):
        """Load state from persistence."""
        if not state:
            return

        fs = state.get("filtered_setpoint")
        if fs is not None:
            self.filtered_setpoint = float(fs)

        self.boost_active = bool(state.get("setpoint_boost_active", False))

        ps = state.get("prev_setpoint_for_boost")
        if ps is not None:
            self.prev_setpoint_for_boost = float(ps)

    def save_state(self) -> dict:
        """Save state for persistence."""
        return {
            "filtered_setpoint": self.filtered_setpoint,
            "setpoint_boost_active": self.boost_active,
            "prev_setpoint_for_boost": self.prev_setpoint_for_boost,
        }

    def filter_setpoint(
        self,
        target_temp: float,
        current_temp: float | None,
        a: float = 0.0,
        deadtime_cool_s: float = 0.0,
    ) -> float:
        """
        Dual-Track setpoint filter: BOOST (full power) + Quadratic Landing.

        Returns SP_for_P — the filtered setpoint for the proportional term.
        The integrator must always use SP_brut (target_temp), not this return value.

        Phase logic based on distance between current_temp and target:
        - BOOST   (remaining > landing_zone): return target → full P power
        - LANDING (remaining ≤ landing_zone): return current + remaining²/landing_zone
          → quadratic braking, stateless, continuous with BOOST at boundary

        Args:
            target_temp:      Raw setpoint (SP_brut).
            current_temp:     Measured temperature. If None, no update is performed.
            a:                Heating gain from ABEstimator (°C/min per duty).
            deadtime_cool_s:  Cooling dead time in seconds.
        """
        if not self.enabled:
            self.filtered_setpoint = target_temp
            self.effective_setpoint = target_temp
            return target_temp

        # First call: initialise filter state
        if self.filtered_setpoint is None:
            self.filtered_setpoint = current_temp if current_temp is not None else target_temp

        # No temperature measurement — keep current state
        if current_temp is None:
            return self.effective_setpoint if self.effective_setpoint is not None else target_temp

        # ── Drop: Instantaneous for energy savings ──
        if target_temp < self.filtered_setpoint:
            self.filtered_setpoint = target_temp
            self.effective_setpoint = target_temp
            return target_temp

        # Track target for drop detection
        self.filtered_setpoint = target_temp

        # ── Rise: Dual-Track (BOOST + Quadratic Landing) ──
        remaining = target_temp - current_temp
        if remaining <= 0:
            self.effective_setpoint = target_temp
            return target_temp

        # Landing zone: temperature rise expected during deadtime at full power,
        # multiplied by SP_LANDING_ZONE_FACTOR to start braking earlier and
        # prevent overshoot from thermal inertia.
        landing_zone = a * deadtime_cool_s / 60.0 * SP_LANDING_ZONE_FACTOR
        landing_zone = max(SP_MIN_LANDING_ZONE, min(landing_zone, SP_MAX_LANDING_ZONE))

        if remaining > landing_zone:
            # BOOST: full power — return target directly
            self.effective_setpoint = target_temp
            return target_temp

        # LANDING: quadratic braking with linear floor
        # Quadratic alone gives P_error → 0 near target (remaining² → 0 fast).
        # The linear floor SP_LANDING_ZONE_MIN_P_FRACTION ensures the P term
        # keeps contributing during the final approach, preventing stalling.
        p_error = (remaining * remaining) / landing_zone
        p_error = max(p_error, remaining * SP_LANDING_ZONE_MIN_P_FRACTION)
        sp_for_p = current_temp + p_error
        self.effective_setpoint = sp_for_p
        return sp_for_p

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
