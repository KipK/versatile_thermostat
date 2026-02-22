from __future__ import annotations

import logging
import math
from typing import Optional

from .const import (
    SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN,
    SP_TAU_SLOW,
    SP_TAU_FAST,
    SP_BAND
)
from ..vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_HEAT, VThermHvacMode_COOL

_LOGGER = logging.getLogger(__name__)

class SmartPISetpointManager:
    """
    Setpoint Management for Smart-PI.
    
    Responsible for:
    1. Setpoint smoothing (Asymmetric EMA Filter).
    2. Setpoint boost detection (detecting manual overrides).
    3. Tracking setpoint changes.
    """

    def __init__(self, name: str, enabled: bool = True):
        self._name = name
        self.enabled = enabled
        
        # Filter state
        self.filtered_setpoint: Optional[float] = None
        self.last_raw_setpoint: Optional[float] = None
        self.initial_temp_for_filter: Optional[float] = None
        
        # Boost state
        self.boost_active: bool = False
        self.prev_setpoint_for_boost: Optional[float] = None

    def reset(self):
        """Reset internal state."""
        self.filtered_setpoint = None
        self.last_raw_setpoint = None
        self.initial_temp_for_filter = None
        self.boost_active = False
        self.prev_setpoint_for_boost = None

    def load_state(self, state: dict):
        """Load state from persistence."""
        if not state:
            return
            
        fs = state.get("filtered_setpoint")
        if fs is not None:
            self.filtered_setpoint = float(fs)
        
        lrs = state.get("last_raw_setpoint")
        if lrs is not None:
            self.last_raw_setpoint = float(lrs)
        
        it = state.get("initial_temp_for_filter")
        if it is not None:
            self.initial_temp_for_filter = float(it)
        
        self.boost_active = bool(state.get("setpoint_boost_active", False))
        
        ps = state.get("prev_setpoint_for_boost")
        if ps is not None:
            self.prev_setpoint_for_boost = float(ps)

    def save_state(self) -> dict:
        """Save state for persistence."""
        return {
            "filtered_setpoint": self.filtered_setpoint,
            "last_raw_setpoint": self.last_raw_setpoint,
            "initial_temp_for_filter": self.initial_temp_for_filter,
            "setpoint_boost_active": self.boost_active,
            "prev_setpoint_for_boost": self.prev_setpoint_for_boost,
        }

    def filter_setpoint(
        self,
        target_temp: float,
        current_temp: float | None,
        hvac_mode: VThermHvacMode,
        dt_min: float,
        advance_ema: bool = True
    ) -> float:
        """
        Apply asymmetric EMA filter to setpoint with midpoint activation.
        """
        if not self.enabled:
            self.filtered_setpoint = target_temp
            self.last_raw_setpoint = target_temp
            return target_temp

        # First call or no previous setpoint: initialize
        if self.filtered_setpoint is None or self.last_raw_setpoint is None:
            self.filtered_setpoint = target_temp
            self.last_raw_setpoint = target_temp
            self.initial_temp_for_filter = None
            return target_temp

        # Detect if the RAW setpoint has changed since last cycle
        setpoint_changed = abs(target_temp - self.last_raw_setpoint) > 0.01

        if setpoint_changed:
            # Setpoint just changed - determine direction and action
            should_filter = False
            if hvac_mode == VThermHvacMode_HEAT:
                should_filter = target_temp > self.last_raw_setpoint
            elif hvac_mode == VThermHvacMode_COOL:
                should_filter = target_temp < self.last_raw_setpoint
            
            # Update last raw setpoint
            self.last_raw_setpoint = target_temp

            if not should_filter:
                # Instant transition (energy saving)
                self.filtered_setpoint = target_temp
                self.initial_temp_for_filter = None
                return target_temp

            # Record initial temperature for midpoint calculation
            if current_temp is not None:
                self.initial_temp_for_filter = current_temp
            
            # Initially follow raw setpoint, filter activates at midpoint
            self.filtered_setpoint = target_temp
            return target_temp

        # Setpoint unchanged - check logic
        if self.initial_temp_for_filter is None or current_temp is None:
            self.filtered_setpoint = target_temp
            return target_temp

        # Calculate midpoint
        midpoint = (self.initial_temp_for_filter + target_temp) / 2.0

        # Check if we have reached the midpoint
        reached_midpoint = False
        if hvac_mode == VThermHvacMode_HEAT:
            reached_midpoint = current_temp >= midpoint
        elif hvac_mode == VThermHvacMode_COOL:
            reached_midpoint = current_temp <= midpoint

        if not reached_midpoint:
            self.filtered_setpoint = target_temp
            return target_temp

        # On first midpoint arrival, filtered_setpoint was held at target_temp.
        # Initialize it to current_temp so the EMA sees the actual remaining gap.
        if abs(self.filtered_setpoint - target_temp) < 0.01:
            self.filtered_setpoint = current_temp

        # Apply EMA
        gap = abs(target_temp - self.filtered_setpoint)
        if gap <= 0.02:
            self.filtered_setpoint = target_temp
            self.initial_temp_for_filter = None
            return target_temp

        if advance_ema:
             # Use band to interpolate between fast and slow time constants
            w = min(gap / SP_BAND, 1.0)
            tau = SP_TAU_SLOW + (SP_TAU_FAST - SP_TAU_SLOW) * w
            
            # Robust alpha calculation
            alpha = 1.0 - math.exp(-max(dt_min, 0.001) / max(tau, 1.0))
            
            self.filtered_setpoint = alpha * target_temp + (1 - alpha) * self.filtered_setpoint

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
