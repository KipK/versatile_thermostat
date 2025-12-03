"""Auto TPI Manager implementing TPI algorithm."""

import logging
import json
import os
import math
from datetime import datetime
from typing import Optional
from dataclasses import dataclass, asdict

import asyncio
from typing import Callable

from homeassistant.core import HomeAssistant
from homeassistant.helpers.event import async_call_later

from .const import (
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 6


@dataclass
class AutoTpiState:
    """Persistent state for Auto TPI algorithm."""
    # Learning coefficients (heat)
    coeff_indoor_heat: float = 0.1
    coeff_outdoor_heat: float = 0.01
    coeff_indoor_autolearn: int = 1  # Counter
    coeff_outdoor_autolearn: int = 0

    # Learning coefficients for Cool
    coeff_indoor_cool: float = 0.1
    coeff_outdoor_cool: float = 0.01
    coeff_indoor_cool_autolearn: int = 1
    coeff_outdoor_cool_autolearn: int = 0
    
    # Offsets.
    offset: float = 0.0
    
    # Previous cycle state (Snapshot for learning)
    last_power: float = 0.0
    last_order: float = 0.0
    last_temp_in: float = 0.0
    last_temp_out: float = 0.0
    last_state: str = 'stop'  # 'heat', 'cool', 'stop'
    previous_state: str = 'stop' # State of the previous cycle
    last_on_temp_in: float = 0.0 # Temp at the end of ON time
    last_update_date: Optional[datetime] = None
    last_heater_stop_time: Optional[datetime] = None # When heater stopped
    
    # Cycle management
    cycle_start_date: Optional[datetime] = None  # Start of current cycle
    cycle_active: bool = False
    current_cycle_cold_factor: float = 0.0 # 1.0 = cold, 0.0 = hot
    
    # Management
    consecutive_failures: int = 0
    autolearn_enabled: bool = False
    last_learning_status: str = "startup"
    total_cycles: int = 0  # Total number of TPI cycles

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        d = data.copy()
        # Date conversion from ISO format
        for date_field in ["last_update_date", "cycle_start_date", "last_heater_stop_time"]:
            if d.get(date_field):
                try:
                    d[date_field] = datetime.fromisoformat(d[date_field])
                except (ValueError, TypeError):
                    d[date_field] = None
        
        # Create instance with defaults first
        instance = cls()
        
        # Filter unknown fields and update only valid ones
        valid_fields = {k for k in cls.__annotations__}
        for key, value in d.items():
            if key in valid_fields:
                setattr(instance, key, value)
        
        return instance


class AutoTpiManager:
    """Auto TPI Manager implementing TPI algorithm."""

    def __init__(self, hass: HomeAssistant, unique_id: str, name: str, cycle_min: int,
                 tpi_threshold_low: float = 0.0, tpi_threshold_high: float = 0.0,
                 coef_int: float = 0.6, coef_ext: float = 0.04,
                 heater_heating_time: int = 0, heater_cooling_time: int = 0):
        self._hass = hass
        self._unique_id = unique_id
        self._name = name
        self._cycle_min = cycle_min
        self._tpi_threshold_low = tpi_threshold_low
        self._tpi_threshold_high = tpi_threshold_high
        self._heater_heating_time = heater_heating_time
        self._heater_cooling_time = heater_cooling_time

        self._storage_path = hass.config.path(
            f".storage/versatile_thermostat_{unique_id}_auto_tpi_v2.json"
        )
        self._default_coef_int = coef_int if coef_int is not None else 0.6
        self._default_coef_ext = coef_ext if coef_ext is not None else 0.04

        self.state = AutoTpiState(
            coeff_indoor_heat=self._default_coef_int,
            coeff_outdoor_heat=self._default_coef_ext,
            coeff_indoor_cool=self._default_coef_int,
            coeff_outdoor_cool=self._default_coef_ext
        )
        self._calculated_params = {}

        # Transient state
        self._current_temp_in: float = 0.0
        self._current_temp_out: float = 0.0
        self._current_target_temp: float = 0.0
        self._current_hvac_action: str = 'stop'
        self._current_hvac_mode: str = 'heat' # 'heat' or 'cool' (or 'off' etc)
        self._last_cycle_power_efficiency: float = 1.0
        self._current_cycle_params: dict = None

        self._timer_remove_callback: Callable[[], None] | None = None
        self._timer_capture_remove_callback: Callable[[], None] | None = None
        # data_provider: async function that returns a dict with:
        # on_time_sec, off_time_sec, on_percent, hvac_mode
        self._data_provider: Callable[[], dict] | None = None
        # event_sender: async function that sends events to thermostat
        self._event_sender: Callable[[dict], None] | None = None

    async def async_save_data(self):
        """Save data."""
        await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Sync save."""
        tmp_path = f"{self._storage_path}.tmp"
        try:
            # Helper for datetime serialization
            def json_serial(obj):
                if isinstance(obj, datetime):
                    return obj.isoformat()
                raise TypeError ("Type %s not serializable" % type(obj))

            data = {
                "version": STORAGE_VERSION,
                "state": self.state.to_dict()
            }
            
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            with open(tmp_path, 'w') as f:
                json.dump(data, f, indent=2, default=json_serial)
            
            # Atomic replace
            os.replace(tmp_path, self._storage_path)

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Save error: %s", self._name, e)
            if os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except Exception:
                    pass

    async def async_load_data(self):
        """Load data."""
        await self._hass.async_add_executor_job(self._load_data_sync)
        await self.calculate()

    def _load_data_sync(self):
        """Sync load."""
        if not os.path.exists(self._storage_path):
            return

        try:
            with open(self._storage_path, 'r') as f:
                data = json.load(f)
                
            version = data.get("version", 0)
            if version >= STORAGE_VERSION:
                state_data = data.get("state", {})
                self.state = AutoTpiState.from_dict(state_data)
                
                # If no learning has been done yet, force the configured defaults
                if self.state.total_cycles == 0:
                     _LOGGER.info("%s - Auto TPI: No learning cycles yet. Enforcing configured coefficients.", self._name)
                     self.state.coeff_indoor_heat = self._default_coef_int
                     self.state.coeff_outdoor_heat = self._default_coef_ext
                     self.state.coeff_indoor_cool = self._default_coef_int
                     self.state.coeff_outdoor_cool = self._default_coef_ext
                     
                _LOGGER.info("%s - Auto TPI: State loaded. Cycles: %d, Indoor learn count: %d",
                            self._name, self.state.total_cycles, self.state.coeff_indoor_autolearn)
            else:
                _LOGGER.info("%s - Auto TPI: Old storage version %d. Resetting to new structure.",
                             self._unique_id, version)
                self.state = AutoTpiState()

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Load error: %s. Resetting.", self._name, e)
            self.state = AutoTpiState(
                coeff_indoor_heat=self._default_coef_int,
                coeff_outdoor_heat=self._default_coef_ext,
                coeff_indoor_cool=self._default_coef_int,
                coeff_outdoor_cool=self._default_coef_ext
            )

    async def update(self, room_temp: float, ext_temp: float,
                    power_percent: float, target_temp: float, hvac_action: str,
                    hvac_mode: str, humidity: Optional[float] = None) -> float:
        """Update state with new data.
        
        This method is called at each control_heating cycle.
        It updates the transient state used for power calculation and future learning.
        
        Returns the calculated power for validation/indication.
        """
        
        # Convert hvac_action to 'heat', 'cool', or 'stop'
        current_state_str = 'stop'
        if hvac_action == 'heating':
            current_state_str = 'heat'
        elif hvac_action == 'cooling':
            current_state_str = 'cool'

        # Store current values for later use in cycle callbacks
        self._current_temp_in = room_temp if room_temp is not None else 0.0
        self._current_temp_out = ext_temp if ext_temp is not None else 0.0
        self._current_target_temp = target_temp if target_temp is not None else 0.0
        self._current_hvac_action = current_state_str
        self._current_hvac_mode = hvac_mode
        
        # Calculate and return power
        # Use hvac_mode to force direction if action is idle (stop)
        # If action is 'heat' or 'cool', use it. If 'stop', check hvac_mode.
        calc_state_str = current_state_str
        if current_state_str == 'stop':
            if hvac_mode == 'cool':
                calc_state_str = 'cool'
            elif hvac_mode == 'heat':
                calc_state_str = 'heat'

        return self.calculate_power(target_temp, room_temp, ext_temp, calc_state_str)

    async def calculate(self) -> Optional[dict]:
        """Return the current calculated TPI parameters."""
        # Return current coefficients for the thermostat to use
        params = {}
        
        # Use hvac_mode to determine which coefficients to return
        # This prevents flapping when switching between heating/cooling actions while in the same mode (e.g. idle)
        # Note: hvac_mode usually comes from VThermHvacMode (heat, cool, off, auto...)
        
        is_cool_mode = self._current_hvac_mode == 'cool' or self._current_hvac_action == 'cool'
        
        if is_cool_mode:
            params[CONF_TPI_COEF_INT] = self.state.coeff_indoor_cool
            params[CONF_TPI_COEF_EXT] = self.state.coeff_outdoor_cool
        else:
            params[CONF_TPI_COEF_INT] = self.state.coeff_indoor_heat
            params[CONF_TPI_COEF_EXT] = self.state.coeff_outdoor_heat
            
        self._calculated_params = params
        return params

    def _should_learn(self) -> bool:
        """Check if learning should be performed."""
        if not self.state.autolearn_enabled:
            return False

        # Power conditions: 0 < last_power < 100
        if not (0 < self.state.last_power < 100):
            return False
            
        # Failures check
        if self.state.consecutive_failures >= 3:
            return False
        
        # 1. First Cycle Exclusion
        if self.state.previous_state == 'stop':
            _LOGGER.debug("%s - Auto TPI: Not learning - First cycle (previous state was stop)", self._name)
            return False
        if self.state.last_order == 0:
            _LOGGER.debug("%s - Auto TPI: Not learning - Last order is 0", self._name)
            return False
            
        # 2. Mild Weather Exclusion (Safe Ratio)
        # Avoid division by small numbers or learning when delta is too small to be significant
        delta_out = self.state.last_order - self._current_temp_out
        if abs(delta_out) < 2.0:
             _LOGGER.debug("%s - Auto TPI: Not learning - Delta out too small (< 2.0)", self._name)
             return False

        return True

    def _get_no_learn_reason(self) -> str:
        """Get reason why learning is not happening."""
        if not self.state.autolearn_enabled:
            return "learning_disabled"
        if not (0 < self.state.last_power < 100):
            return f"power_out_of_range({self.state.last_power:.1f}%)"
        if self.state.consecutive_failures >= 3:
            return f"too_many_failures({self.state.consecutive_failures})"
        return "unknown"

    async def _perform_learning(self, current_temp_in: float, current_temp_out: float):
        """Execute the learning logic based on previous state and current observations."""
        
        is_heat = self.state.last_state == 'heat'
        is_cool = self.state.last_state == 'cool'
        
        if not (is_heat or is_cool):
            self.state.last_learning_status = "not_heating_or_cooling"
            _LOGGER.debug("%s - Auto TPI: Not learning - system was in %s mode",
                         self._name, self.state.last_state)
            return

        target_temp = self.state.last_order

        # 1. Mandatory Condition: Heating Logic Updates
        # We allow learning even if target is reached (overshoot) to adjust coefficients down
        
        # 3. Ratio Validation
        # Validate the calculated ratio: ratio = (target_temp - indoor_temp) / (target_temp - outdoor_temp)
        # We use current temps (end of cycle) for this validation to be consistent with the gap check
        delta_in = target_temp - current_temp_in
        delta_out = target_temp - current_temp_out

        if abs(delta_out) < 0.1:
            self.state.last_learning_status = "delta_out_too_small"
            _LOGGER.debug("%s - Auto TPI: Not learning - delta_out too small", self._name)
            return

        ratio = delta_in / delta_out

        # Allow negative ratio (overshoot) but bound it to avoid anomalies
        if not (-5 < ratio < 10):
            mode_name = "heat" if is_heat else "cool"
            self.state.last_learning_status = f"ratio_out_of_range_{mode_name}({ratio:.2f})"
            _LOGGER.debug("%s - Auto TPI: Not learning - ratio %.2f out of range (-5 to 10) for %s",
                          self._name, ratio, mode_name)
            return

        # Calculate deltas based on direction
        if is_heat:
            temp_progress = current_temp_in - self.state.last_temp_in
            target_diff = self.state.last_order - self.state.last_temp_in
            outdoor_condition = current_temp_out < self.state.last_order
        else:  # Cool
            temp_progress = self.state.last_temp_in - current_temp_in
            target_diff = self.state.last_temp_in - self.state.last_order
            outdoor_condition = current_temp_out > self.state.last_order

        learned = False
        
        # Priority 1: Indoor Coefficient
        if temp_progress > 0 and target_diff > 0:
            if self._learn_indoor(target_diff, temp_progress, self._last_cycle_power_efficiency, is_cool):
                self.state.last_learning_status = f"learned_indoor_{'cool' if is_cool else 'heat'}"
                learned = True
            
        # Priority 2: Outdoor Coefficient
        # Fallback if Priority 1 failed (e.g., real_rise too small) or wasn't applicable
        if not learned and outdoor_condition:
            if self._learn_outdoor(current_temp_in, current_temp_out, is_cool):
                self.state.last_learning_status = f"learned_outdoor_{'cool' if is_cool else 'heat'}"
                learned = True
        
        if not learned:
            if not outdoor_condition and not (temp_progress > 0 and target_diff > 0):
                self.state.last_learning_status = f"no_valid_conditions(progress={temp_progress:.2f},target_diff={target_diff:.2f})"
                _LOGGER.debug("%s - Auto TPI: No valid learning conditions. Temp progress: %.3f, Target diff: %.3f",
                             self._name, temp_progress, target_diff)
            # Else status was set by learn_indoor failure
            _LOGGER.debug("%s - Auto TPI: No valid learning conditions. Temp progress: %.3f, Target diff: %.3f",
                         self._name, temp_progress, target_diff)

        if learned:
            _LOGGER.info("%s - Auto TPI: Learning successful - %s",
                        self._name, self.state.last_learning_status)

    def _learn_indoor(self, delta_theoretical: float, delta_real: float, efficiency: float = 1.0, is_cool: bool = False) -> bool:
        """Learn indoor coefficient."""
        
        # 3. Correct Delta Calculation
        # We want to use the rise during the ON period, not the full cycle drift.
        # However, delta_real passed here comes from _perform_learning which uses current_temp - last_temp.
        # We need to recalculate it using last_on_temp_in if available.
        
        if is_cool:
             # For cooling, we expect temp to drop during ON time
             real_rise = self.state.last_temp_in - self.state.last_on_temp_in
        else:
             # For heating, we expect temp to rise during ON time
             real_rise = self.state.last_on_temp_in - self.state.last_temp_in
             
        # Fallback if last_on_temp_in wasn't captured properly (e.g. restart)
        if self.state.last_on_temp_in == 0.0:
             real_rise = delta_real
             _LOGGER.debug("%s - Auto TPI: last_on_temp_in missing, falling back to full cycle delta", self._name)

        if real_rise <= 0.01: # Minimal rise required (0.01 to account for float precision/small sensors)
            _LOGGER.debug("%s - Auto TPI: Cannot learn indoor - real_rise %.3f <= 0.01. Will try outdoor learning.", self._name, real_rise)
            self.state.last_learning_status = "real_rise_too_small"
            return False

        # Adjust theoretical delta by the efficiency of the power delivered
        # If efficiency was 50% (due to rampup time), we expect only 50% of the result.
        # So we compare real progress against (theoretical * efficiency)
        adjusted_theoretical = delta_theoretical * efficiency
        
        if adjusted_theoretical <= 0:
             _LOGGER.warning("%s - Auto TPI: Cannot learn indoor - adjusted_theoretical <= 0 (eff=%.2f)",
                             self._name, efficiency)
             self.state.last_learning_status = "adjusted_theoretical_lte_0"
             return False

        ratio = adjusted_theoretical / real_rise
        current_coeff = self.state.coeff_indoor_cool if is_cool else self.state.coeff_indoor_heat
        coeff_new = current_coeff * ratio
        
        # Validate coefficient - reject only truly invalid values (non-finite or <= 0)
        if not math.isfinite(coeff_new) or coeff_new <= 0:
            _LOGGER.warning("%s - Auto TPI: Invalid new indoor coeff: %.3f (non-finite or <= 0), skipping",
                           self._name, coeff_new)
            self.state.last_learning_status = "invalid_indoor_coeff"
            return False
        
        # 4. Cap Coefficient
        MAX_COEFF = 0.6
        if coeff_new > MAX_COEFF:
            _LOGGER.info("%s - Auto TPI: Calculated indoor coeff %.3f > %.1f, capping to %.1f before averaging",
                        self._name, coeff_new, MAX_COEFF, MAX_COEFF)
            coeff_new = MAX_COEFF
            
        # 5. EMA Smoothing (20% weight)
        # new_avg = (old_avg * 0.8) + (new_sample * 0.2)
        old_coeff = self.state.coeff_indoor_cool if is_cool else self.state.coeff_indoor_heat
        avg_coeff = (old_coeff * 0.8) + (coeff_new * 0.2)

        # Update counters just for stats/logging, no longer used for weighting
        count = self.state.coeff_indoor_cool_autolearn if is_cool else self.state.coeff_indoor_autolearn
        new_count = min(count + 1, 50)
        
        if is_cool:
            self.state.coeff_indoor_cool = avg_coeff
            self.state.coeff_indoor_cool_autolearn = new_count
        else:
            self.state.coeff_indoor_heat = avg_coeff
            self.state.coeff_indoor_autolearn = new_count
        
        _LOGGER.info(
            "%s - Auto TPI: Learn indoor (%s). Old: %.3f, New calculated: %.3f (rise=%.3f), Averaged: %.3f (count: %d)",
            self._name, 'cool' if is_cool else 'heat', old_coeff, coeff_new, real_rise, avg_coeff, new_count
        )
        return True

    def _learn_outdoor(self, current_temp_in: float, current_temp_out: float, is_cool: bool = False) -> bool:
        """Learn outdoor coefficient."""
        gap_in = self.state.last_order - current_temp_in
        gap_out = self.state.last_order - current_temp_out
        
        if gap_out == 0:
            _LOGGER.debug("%s - Auto TPI: Cannot learn outdoor - gap_out is 0", self._name)
            self.state.last_learning_status = "gap_out_is_zero"
            return False

        ratio_influence = gap_in / gap_out
        current_indoor = self.state.coeff_indoor_cool if is_cool else self.state.coeff_indoor_heat
        current_outdoor = self.state.coeff_outdoor_cool if is_cool else self.state.coeff_outdoor_heat
        
        contribution = current_indoor * ratio_influence
        coeff_new = contribution + current_outdoor

        # Validate coefficient - reject only truly invalid values (non-finite or <= 0)
        # For values > 1.0, cap them instead of rejecting (like Jeedom does for lower bound)
        if not math.isfinite(coeff_new) or coeff_new <= 0:
            _LOGGER.warning("%s - Auto TPI: Invalid new outdoor coeff: %.3f (non-finite or <= 0), skipping",
                           self._name, coeff_new)
            self.state.last_learning_status = "invalid_outdoor_coeff"
            return False
        
        # Cap coefficient at 1.0 before averaging (normalized units)
        if coeff_new > 1.0:
            _LOGGER.info("%s - Auto TPI: Calculated outdoor coeff %.3f > 1.0, capping to 1.0 before averaging",
                        self._name, coeff_new)
            coeff_new = 1.0

        count = self.state.coeff_outdoor_cool_autolearn if is_cool else self.state.coeff_outdoor_autolearn
        old_coeff = current_outdoor
        
        avg_coeff = (
            (old_coeff * count + coeff_new) / (count + 1)
        )
        new_count = min(count + 1, 50)
        
        if is_cool:
            self.state.coeff_outdoor_cool = avg_coeff
            self.state.coeff_outdoor_cool_autolearn = new_count
        else:
            self.state.coeff_outdoor_heat = avg_coeff
            self.state.coeff_outdoor_autolearn = new_count
        
        _LOGGER.info(
            "%s - Auto TPI: Learn outdoor (%s). Old: %.3f, New calculated: %.3f, Averaged: %.3f (count: %d)",
            self._name, 'cool' if is_cool else 'heat', old_coeff, coeff_new, avg_coeff, new_count
        )
        return True

    def _detect_failures(self, current_temp_in: float):
        """Detect system failures."""
        OFFSET_FAILURE = 1.0
        MIN_LEARN_FOR_DETECTION = 25
        
        failure_detected = False
        
        if (self.state.last_state == 'heat' and
            current_temp_in < self.state.last_order - OFFSET_FAILURE and
            current_temp_in < self.state.last_temp_in and
            self.state.coeff_indoor_autolearn > MIN_LEARN_FOR_DETECTION):
            failure_detected = True
            _LOGGER.warning("%s - Auto TPI: Failure detected in HEAT mode", self._name)
            
        elif (self.state.last_state == 'cool' and
              current_temp_in > self.state.last_order + OFFSET_FAILURE and
              current_temp_in > self.state.last_temp_in and
              self.state.coeff_indoor_autolearn > MIN_LEARN_FOR_DETECTION):
            failure_detected = True
            _LOGGER.warning("%s - Auto TPI: Failure detected in COOL mode", self._name)
            
        if failure_detected:
            self.state.consecutive_failures += 1
            if self.state.consecutive_failures >= 3:
                self.state.autolearn_enabled = False
                _LOGGER.error("%s - Auto TPI: Learning disabled due to %d consecutive failures.", 
                             self._name, self.state.consecutive_failures)
        else:
            self.state.consecutive_failures = 0

    def _is_valid_coeff(self, coeff: float) -> bool:
        """Check if coefficient is valid."""
        return math.isfinite(coeff) and 0 < coeff <= 1.0
        
    def calculate_power(self, setpoint: float, temp_in: float, temp_out: float, state_str: str) -> float:
        """Calculate power using TPI formula."""
        if setpoint is None or temp_in is None or temp_out is None:
            return 0.0

        direction = 1 if state_str == 'heat' else -1
        delta_in = setpoint - temp_in
        delta_out = setpoint - temp_out
        
        if state_str == 'cool':
            coeff_int = self.state.coeff_indoor_cool
            coeff_ext = self.state.coeff_outdoor_cool
        else:
            coeff_int = self.state.coeff_indoor_heat
            coeff_ext = self.state.coeff_outdoor_heat
            
        offset = self.state.offset
        # Jeedom has an offset you can dynamically feed to the thermostat.
        # We keep it here for future addition if needed, it's not used yet.
        power = (direction * delta_in * coeff_int) + (direction * delta_out * coeff_ext) + offset
        return max(0.0, min(100.0, power))

    async def on_cycle_started(self, on_time_sec: float, off_time_sec: float,
                             on_percent: float, hvac_mode: str):
        """Called when a TPI cycle starts."""
        # Detect if previous cycle was interrupted
        if self.state.cycle_active:
            _LOGGER.info("%s - Auto TPI: Previous cycle was interrupted (not completed). Discarding it.", self._name)
            # You could add specific logic here if needed (stats, etc)
        
        # Cancel any pending capture timer
        if self._timer_capture_remove_callback:
            self._timer_capture_remove_callback()
            self._timer_capture_remove_callback = None

        self.state.cycle_active = True
        
        _LOGGER.debug("%s - Auto TPI: Cycle started. On: %.0fs, Off: %.0fs (%.1f%%), Mode: %s",
                     self._name, on_time_sec, off_time_sec, on_percent * 100, hvac_mode)
        
        now = datetime.now()
        
        # Snapshot current state for learning at the end of the cycle
        self.state.last_temp_in = self._current_temp_in
        self.state.last_temp_out = self._current_temp_out
        self.state.last_order = self._current_target_temp
        self.state.last_power = on_percent if on_percent is not None else 0.0
        self.state.last_on_temp_in = 0.0 # Reset
        
        # Save previous state before updating last_state (for first cycle detection)
        self.state.previous_state = self.state.last_state

        # Map VThermHvacMode/HVACMode to internal state string
        # hvac_mode is expected to be VThermHvacMode or string representation
        mode_str = str(hvac_mode)
        if mode_str == 'heat' or mode_str == 'heating':
            self.state.last_state = 'heat'
        elif mode_str == 'cool' or mode_str == 'cooling':
            self.state.last_state = 'cool'
        else:
            self.state.last_state = 'stop'
            
        self.state.cycle_start_date = now
        self.state.last_update_date = now
        
        # Schedule capture of temperature at the end of the ON pulse
        if on_time_sec > 0:
            self._timer_capture_remove_callback = async_call_later(
                self._hass,
                on_time_sec,
                self._capture_end_of_on_temp
            )

        # Calculate cold factor for this cycle
        self.state.current_cycle_cold_factor = 0.0
        if self._heater_cooling_time > 0 and self.state.last_heater_stop_time:
            elapsed_off = (now - self.state.last_heater_stop_time).total_seconds() / 60.0
            if elapsed_off >= 0:
                self.state.current_cycle_cold_factor = max(0.0, elapsed_off / self._heater_cooling_time)
                _LOGGER.debug("%s - Auto TPI: Cold factor calc: elapsed_off=%.1f min, cooling_time=%.1f min, factor=%.2f",
                              self._name, elapsed_off, self._heater_cooling_time, self.state.current_cycle_cold_factor)
        
        await self.async_save_data()

    async def on_cycle_completed(self, on_time_sec: float, off_time_sec: float, hvac_mode: str):
        """Called when a TPI cycle completes."""
        if not self.state.cycle_active:
            _LOGGER.debug("%s - Auto TPI: Cycle completed but no cycle active. Ignoring.", self._name)
            return

        self.state.cycle_active = False

        elapsed_minutes = (on_time_sec + off_time_sec) / 60
        on_time_minutes = on_time_sec / 60.0
        self.state.total_cycles += 1
        
        # Update last_heater_stop_time if we were heating
        if self.state.last_state == 'heat':
            self.state.last_heater_stop_time = datetime.now()

        # Calculate Power Efficiency based on Heating time and Cold Factor
        # If the heater was ON for 10 min but heating_time is 5 min, only 5 min were effective heating
        # But if cold_factor is 0.5 (half warm), heating_time is effectively 2.5 min
        self._last_cycle_power_efficiency = 1.0
        effective_heating_time = self._heater_heating_time * self.state.current_cycle_cold_factor
        
        if effective_heating_time > 0 and on_time_minutes > 0:
            effective_time = max(0.0, on_time_minutes - effective_heating_time)
            self._last_cycle_power_efficiency = effective_time / on_time_minutes
            
            _LOGGER.debug("%s - Auto TPI: Power Efficiency calc: on_time=%.1f min, heating_time=%.1f, cold_factor=%.2f, eff_heating_time=%.1f, eff=%.2f",
                          self._name, on_time_minutes, self._heater_heating_time, self.state.current_cycle_cold_factor, effective_heating_time, self._last_cycle_power_efficiency)
        
        _LOGGER.info("%s - Auto TPI: Cycle #%d completed after %.1f minutes (efficiency: %.2f)",
                    self._name, self.state.total_cycles, elapsed_minutes, self._last_cycle_power_efficiency)
        
        # Attempt learning
        # We also check if the cycle was significant enough (on_time > effective_heating_time)
        is_significant_cycle = True
        if effective_heating_time > 0 and on_time_minutes <= effective_heating_time:
            is_significant_cycle = False
            _LOGGER.debug("%s - Auto TPI: Cycle ignored for learning - ON time (%.1f) <= Eff. Heating Time (%.1f)",
                          self._name, on_time_minutes, effective_heating_time)

        if self._should_learn() and is_significant_cycle:
            _LOGGER.info("%s - Auto TPI: Attempting to learn from cycle data", self._name)
            await self._perform_learning(self._current_temp_in, self._current_temp_out)
        else:
            reason = self._get_no_learn_reason()
            if not is_significant_cycle:
                reason = "on_time_too_short_vs_heating_time"
                
            _LOGGER.debug("%s - Auto TPI: Not learning this cycle: %s", self._name, reason)
            self.state.last_learning_status = reason
            
        # Check for failures
        self._detect_failures(self._current_temp_in)
        
        await self.async_save_data()
    
    def get_calculated_params(self) -> dict:
        return self._calculated_params
    
    @property
    def learning_active(self) -> bool:
        return self.state.autolearn_enabled
    
    @property
    def int_cycles(self) -> int:
        """Number of learning cycles completed for internal coefficient"""
        is_cool_mode = self._current_hvac_mode == 'cool' or self._current_hvac_action == 'cool'
        if is_cool_mode:
            return self.state.coeff_indoor_cool_autolearn
        return self.state.coeff_indoor_autolearn

    @property
    def ext_cycles(self) -> int:
        """Number of learning cycles completed for external coefficient"""
        is_cool_mode = self._current_hvac_mode == 'cool' or self._current_hvac_action == 'cool'
        if is_cool_mode:
            return self.state.coeff_outdoor_cool_autolearn
        return self.state.coeff_outdoor_autolearn
    
    @property
    def heating_cycles_count(self) -> int:
        """Number of total TPI cycles"""
        return self.state.total_cycles
    
    @property
    def time_constant(self) -> float:
        """Thermal time constant in hours"""
        if self.state.coeff_indoor_heat > 0:
            return round(1.0 / self.state.coeff_indoor_heat, 2)
        return 0.0
    
    @property
    def confidence(self) -> float:
        """Confidence level in the learned model (0.0 to 1.0)"""
        # We consider stability reached when both coefficients have 50 cycles
        int_cycles = self.int_cycles
        ext_cycles = self.ext_cycles
        
        if int_cycles == 0 and ext_cycles == 0:
            return 0.0
            
        # Average of progress for both
        confidence_int = min(int_cycles / 50.0, 1.0)
        confidence_ext = min(ext_cycles / 50.0, 1.0)
        
        cycle_confidence = (confidence_int + confidence_ext) / 2.0
        
        if self.state.consecutive_failures > 0:
            failure_penalty = min(self.state.consecutive_failures * 0.15, 0.6)
            cycle_confidence = max(0.2, cycle_confidence - failure_penalty)
        
        return round(cycle_confidence, 2)

    async def start_learning(self, coef_int: float = None, coef_ext: float = None):
        """Start learning, optionally resetting coefficients to configured values."""
        _LOGGER.info("%s - Auto TPI: Starting learning with coef_int=%.3f, coef_ext=%.3f",
                    self._name, coef_int or self.state.coeff_indoor_heat, coef_ext or self.state.coeff_outdoor_heat)
        
        if coef_int is not None:
            self.state.coeff_indoor_heat = coef_int
            self.state.coeff_indoor_autolearn = 1
        if coef_ext is not None:
            self.state.coeff_outdoor_heat = coef_ext
            self.state.coeff_outdoor_autolearn = 0
        
        # Reset all learning data for fresh start
        self.state.last_power = 0.0
        self.state.last_order = 0.0
        self.state.last_temp_in = 0.0
        self.state.last_temp_out = 0.0
        self.state.last_state = 'stop'
        self.state.last_update_date = None
        self.state.last_heater_stop_time = None
        self.state.total_cycles = 0
        self.state.consecutive_failures = 0
        self.state.last_learning_status = "learning_started"
        self.state.autolearn_enabled = True
        self.state.cycle_start_date = datetime.now()
        self.state.cycle_active = False
        
        await self.async_save_data()

    async def stop_learning(self):
        _LOGGER.info("%s - Auto TPI: Stopping learning", self._name)
        self.state.autolearn_enabled = False
        self.state.last_learning_status = "learning_stopped"
        await self.async_save_data()
        
    async def reset_learning_data(self):
        _LOGGER.info("%s - Auto TPI: Resetting all learning data", self._name)
        self.state = AutoTpiState()
        self.state.cycle_active = False
        await self.async_save_data()

    async def start_cycle_loop(self, data_provider: Callable[[], dict], event_sender: Callable[[dict], None]):
        """Start the TPI cycle loop."""
        _LOGGER.debug("%s - Auto TPI: Starting cycle loop", self._name)
        self._data_provider = data_provider
        self._event_sender = event_sender

        # Stop existing timer if any
        if self._timer_remove_callback:
            self._timer_remove_callback()
            self._timer_remove_callback = None

        # Execute immediately
        await self._tick()

    async def _capture_end_of_on_temp(self, _):
        """Called when the ON period ends (heater turns off)."""
        self.state.last_on_temp_in = self._current_temp_in
        _LOGGER.debug("%s - Auto TPI: Captured last_on_temp_in: %.3f", self._name, self.state.last_on_temp_in)
        self._timer_capture_remove_callback = None

    def stop_cycle_loop(self):
        """Stop the TPI cycle loop."""
        _LOGGER.debug("%s - Auto TPI: Stopping cycle loop", self._name)
        if self._timer_remove_callback:
            self._timer_remove_callback()
            self._timer_remove_callback = None
        if self._timer_capture_remove_callback:
            self._timer_capture_remove_callback()
            self._timer_capture_remove_callback = None

        self._data_provider = None
        self._event_sender = None

    def _schedule_next_timer(self):
        """Schedule the next timer."""
        # Ensure we don't have multiple timers
        if self._timer_remove_callback:
            self._timer_remove_callback()
            
        self._timer_remove_callback = async_call_later(
            self._hass, self._cycle_min * 60, self._on_timer_fired
        )

    async def _on_timer_fired(self, _):
        """Called when timer fires."""
        await self._tick()

    async def _tick(self):
        """Perform a tick of the cycle loop."""
        if not self._data_provider:
            return

        now = datetime.now()
        
        # 1. Handle previous cycle completion
        if self.state.cycle_start_date is not None and self._current_cycle_params is not None:
            elapsed_minutes = (now - self.state.cycle_start_date).total_seconds() / 60
            expected_duration = self._cycle_min
            tolerance = max(expected_duration * 0.1, 1.0)

            if abs(elapsed_minutes - expected_duration) <= tolerance:
                _LOGGER.debug(
                    "%s - Cycle validation success: duration=%.1fmin (expected=%.1fmin). Triggering learning.",
                    self._name, elapsed_minutes, expected_duration
                )
                # Use stored parameters from the PREVIOUS cycle
                prev_params = self._current_cycle_params
                await self.on_cycle_completed(
                    on_time_sec=prev_params.get("on_time_sec", 0),
                    off_time_sec=prev_params.get("off_time_sec", 0),
                    hvac_mode=prev_params.get("hvac_mode", "stop")
                )
            else:
                _LOGGER.debug(
                    "%s - Cycle validation failed: duration=%.1fmin (expected=%.1fmin, tolerance=%.1fmin). Skipping learning.",
                    self._name, elapsed_minutes, expected_duration, tolerance
                )
            
            # Reset previous cycle tracking
            self._current_cycle_params = None

        # 2. Get fresh data from thermostat
        try:
            if asyncio.iscoroutinefunction(self._data_provider):
                params = await self._data_provider()
            else:
                params = self._data_provider()
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Error getting data from thermostat: %s", self._name, e)
            # Retry later ?
            self._schedule_next_timer()
            return

        if not params:
            _LOGGER.warning("%s - Auto TPI: No data received from thermostat", self._name)
            self._schedule_next_timer()
            return
            
        self._current_cycle_params = params
        on_time = params.get("on_time_sec", 0)
        off_time = params.get("off_time_sec", 0)
        on_percent = params.get("on_percent", 0)
        hvac_mode = params.get("hvac_mode", "stop")

        # 3. Notify start of cycle
        await self.on_cycle_started(on_time, off_time, on_percent, hvac_mode)
        
        # 4. Notify thermostat to apply changes
        if self._event_sender:
            try:
                if asyncio.iscoroutinefunction(self._event_sender):
                    await self._event_sender(params)
                else:
                    self._event_sender(params)
            except Exception as e:
                _LOGGER.error("%s - Auto TPI: Error sending event to thermostat: %s", self._name, e)

        # 5. Schedule next tick
        self._schedule_next_timer()
