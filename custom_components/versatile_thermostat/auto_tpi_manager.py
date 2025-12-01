"""Auto TPI Manager implementing TPI algorithm."""

import logging
import json
import os
import math
from datetime import datetime
from typing import Optional
from dataclasses import dataclass, asdict

from homeassistant.core import HomeAssistant

from .const import (
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 5


@dataclass
class AutoTpiState:
    """Persistent state for Auto TPI algorithm."""
    # Learning coefficients (Unified)
    coeff_indoor: float = 0.4
    coeff_outdoor: float = 0.05
    coeff_indoor_autolearn: int = 1  # Counter
    coeff_outdoor_autolearn: int = 0
    
    # Offsets.
    offset: float = 0.0
    
    # Previous cycle state (Snapshot for learning)
    last_power: float = 0.0
    last_order: float = 0.0
    last_temp_in: float = 0.0
    last_temp_out: float = 0.0
    last_state: str = 'stop'  # 'heat', 'cool', 'stop'
    last_update_date: Optional[datetime] = None
    
    # Cycle management
    cycle_start_date: Optional[datetime] = None  # Start of current cycle
    cycle_active: bool = False
    
    # Management
    consecutive_failures: int = 0
    autolearn_enabled: bool = True
    last_learning_status: str = "startup"
    total_cycles: int = 0  # Total number of TPI cycles

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        d = data.copy()
        # Date conversion from ISO format
        for date_field in ["last_update_date", "cycle_start_date"]:
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
                 heater_rampup: int = 0):
        self._hass = hass
        self._unique_id = unique_id
        self._name = name
        self._cycle_min = cycle_min
        self._tpi_threshold_low = tpi_threshold_low
        self._tpi_threshold_high = tpi_threshold_high
        self._heater_rampup = heater_rampup

        self._storage_path = hass.config.path(
            f".storage/versatile_thermostat_{unique_id}_auto_tpi_v2.json"
        )

        self.state = AutoTpiState()
        if coef_int is not None:
            self.state.coeff_indoor = coef_int
        if coef_ext is not None:
            self.state.coeff_outdoor = coef_ext
        self._calculated_params = {}

        # Transient state
        self._current_temp_in: float = 0.0
        self._current_temp_out: float = 0.0
        self._current_target_temp: float = 0.0
        self._current_hvac_action: str = 'stop'
        self._last_cycle_power_efficiency: float = 1.0

    async def async_save_data(self):
        """Save data."""
        await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Sync save."""
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
            with open(self._storage_path, 'w') as f:
                json.dump(data, f, indent=2, default=json_serial)
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Save error: %s", self._name, e)

    async def async_load_data(self):
        """Load data."""
        await self._hass.async_add_executor_job(self._load_data_sync)

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
                _LOGGER.info("%s - Auto TPI: State loaded. Cycles: %d, Indoor learn count: %d", 
                            self._name, self.state.total_cycles, self.state.coeff_indoor_autolearn)
            else:
                _LOGGER.info("%s - Auto TPI: Old storage version %d. Resetting to new structure.",
                             self._unique_id, version)
                self.state = AutoTpiState()

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Load error: %s. Resetting.", self._name, e)
            self.state = AutoTpiState()

    async def update(self, room_temp: float, ext_temp: float,
                    power_percent: float, target_temp: float, hvac_action: str,
                    humidity: Optional[float] = None) -> float:
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
        
        # Calculate and return power
        return self.calculate_power(target_temp, room_temp, ext_temp, current_state_str)

    async def calculate(self) -> Optional[dict]:
        """Return the current calculated TPI parameters."""
        # Return current coefficients for the thermostat to use
        params = {}
        params[CONF_TPI_COEF_INT] = self.state.coeff_indoor
        params[CONF_TPI_COEF_EXT] = self.state.coeff_outdoor
            
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
            self._learn_indoor(target_diff, temp_progress, self._last_cycle_power_efficiency)
            self.state.last_learning_status = "learned_indoor"
            learned = True
            
        # Priority 2: Outdoor Coefficient
        elif outdoor_condition:
            self._learn_outdoor(current_temp_in, current_temp_out)
            self.state.last_learning_status = "learned_outdoor"
            learned = True
        else:
            self.state.last_learning_status = f"no_valid_conditions(progress={temp_progress:.2f},target_diff={target_diff:.2f})"
            _LOGGER.debug("%s - Auto TPI: No valid learning conditions. Temp progress: %.3f, Target diff: %.3f",
                         self._name, temp_progress, target_diff)

        if learned:
            _LOGGER.info("%s - Auto TPI: Learning successful - %s",
                        self._name, self.state.last_learning_status)

    def _learn_indoor(self, delta_theoretical: float, delta_real: float, efficiency: float = 1.0):
        """Learn indoor coefficient."""
        if delta_real <= 0:
            _LOGGER.warning("%s - Auto TPI: Cannot learn indoor - delta_real <= 0", self._name)
            return

        # Adjust theoretical delta by the efficiency of the power delivered
        # If efficiency was 50% (due to rampup time), we expect only 50% of the result.
        # So we compare real progress against (theoretical * efficiency)
        adjusted_theoretical = delta_theoretical * efficiency
        
        if adjusted_theoretical <= 0:
             _LOGGER.warning("%s - Auto TPI: Cannot learn indoor - adjusted_theoretical <= 0 (eff=%.2f)",
                             self._name, efficiency)
             return

        ratio = adjusted_theoretical / delta_real
        coeff_new = self.state.coeff_indoor * ratio
        
        # Validate coefficient - reject only truly invalid values (non-finite or <= 0)
        # For values > 1.0, cap them instead of rejecting (like Jeedom does for lower bound)
        if not math.isfinite(coeff_new) or coeff_new <= 0:
            _LOGGER.warning("%s - Auto TPI: Invalid new indoor coeff: %.3f (non-finite or <= 0), skipping",
                           self._name, coeff_new)
            return
        
        # Cap coefficient at 1.0 before averaging (normalized units)
        if coeff_new > 1.0:
            _LOGGER.info("%s - Auto TPI: Calculated indoor coeff %.3f > 1.0, capping to 1.0 before averaging", 
                        self._name, coeff_new)
            coeff_new = 1.0
            
        # Weighted average
        count = self.state.coeff_indoor_autolearn
        old_coeff = self.state.coeff_indoor
        self.state.coeff_indoor = (
            (self.state.coeff_indoor * count + coeff_new) / (count + 1)
        )
        # Ensure the averaged result is also valid (defense in depth)
        if self.state.coeff_indoor > 1.0:
             self.state.coeff_indoor = 1.0

        self.state.coeff_indoor_autolearn = min(count + 1, 50)
        
        _LOGGER.info(
            "%s - Auto TPI: Learn indoor. Old: %.3f, New calculated: %.3f, Averaged: %.3f (count: %d)",
            self._name, old_coeff, coeff_new, self.state.coeff_indoor, self.state.coeff_indoor_autolearn
        )

    def _learn_outdoor(self, current_temp_in: float, current_temp_out: float):
        """Learn outdoor coefficient."""
        gap_in = self.state.last_order - current_temp_in
        gap_out = self.state.last_order - current_temp_out
        
        if gap_out == 0:
            _LOGGER.debug("%s - Auto TPI: Cannot learn outdoor - gap_out is 0", self._name)
            return

        ratio_influence = gap_in / gap_out
        contribution = self.state.coeff_indoor * ratio_influence
        coeff_new = contribution + self.state.coeff_outdoor

        # Validate coefficient - reject only truly invalid values (non-finite or <= 0)
        # For values > 1.0, cap them instead of rejecting (like Jeedom does for lower bound)
        if not math.isfinite(coeff_new) or coeff_new <= 0:
            _LOGGER.warning("%s - Auto TPI: Invalid new outdoor coeff: %.3f (non-finite or <= 0), skipping", 
                           self._name, coeff_new)
            return
        
        # Cap coefficient at 1.0 before averaging (normalized units)
        if coeff_new > 1.0:
            _LOGGER.info("%s - Auto TPI: Calculated outdoor coeff %.3f > 1.0, capping to 1.0 before averaging", 
                        self._name, coeff_new)
            coeff_new = 1.0

        count = self.state.coeff_outdoor_autolearn
        old_coeff = self.state.coeff_outdoor
        self.state.coeff_outdoor = (
            (self.state.coeff_outdoor * count + coeff_new) / (count + 1)
        )
        self.state.coeff_outdoor_autolearn = min(count + 1, 50)
        
        _LOGGER.info(
            "%s - Auto TPI: Learn outdoor. Old: %.3f, New calculated: %.3f, Averaged: %.3f (count: %d)",
            self._name, old_coeff, coeff_new, self.state.coeff_outdoor, self.state.coeff_outdoor_autolearn
        )

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
        
        coeff_int = self.state.coeff_indoor
        coeff_ext = self.state.coeff_outdoor
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

        self.state.cycle_active = True
        
        _LOGGER.debug("%s - Auto TPI: Cycle started. On: %.0fs, Off: %.0fs (%.1f%%), Mode: %s",
                     self._name, on_time_sec, off_time_sec, on_percent * 100, hvac_mode)
        
        now = datetime.now()
        
        # Snapshot current state for learning at the end of the cycle
        self.state.last_temp_in = self._current_temp_in
        self.state.last_temp_out = self._current_temp_out
        self.state.last_order = self._current_target_temp
        self.state.last_power = on_percent if on_percent is not None else 0.0
        
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
        
        await self.async_save_data()

    async def on_cycle_completed(self, on_time_sec: float, off_time_sec: float, hvac_mode: str):
        """Called when a TPI cycle completes."""
        self.state.cycle_active = False

        elapsed_minutes = (on_time_sec + off_time_sec) / 60
        on_time_minutes = on_time_sec / 60.0
        self.state.total_cycles += 1
        
        # Calculate Power Efficiency based on RampUp time
        # If the heater was ON for 10 min but rampup is 5 min, only 5 min were effective heating
        self._last_cycle_power_efficiency = 1.0
        if self._heater_rampup > 0 and on_time_minutes > 0:
            effective_time = max(0.0, on_time_minutes - self._heater_rampup)
            self._last_cycle_power_efficiency = effective_time / on_time_minutes
            
            _LOGGER.debug("%s - Auto TPI: Power Efficiency calc: on_time=%.1f min, rampup=%.1f min, eff=%.2f",
                          self._name, on_time_minutes, self._heater_rampup, self._last_cycle_power_efficiency)
        
        _LOGGER.info("%s - Auto TPI: Cycle #%d completed after %.1f minutes (efficiency: %.2f)",
                    self._name, self.state.total_cycles, elapsed_minutes, self._last_cycle_power_efficiency)
        
        # Attempt learning
        # We also check if the cycle was significant enough (on_time > rampup)
        is_significant_cycle = True
        if self._heater_rampup > 0 and on_time_minutes <= self._heater_rampup:
            is_significant_cycle = False
            _LOGGER.debug("%s - Auto TPI: Cycle ignored for learning - ON time (%.1f) <= RampUp (%.1f)",
                          self._name, on_time_minutes, self._heater_rampup)

        if self._should_learn() and is_significant_cycle:
            _LOGGER.info("%s - Auto TPI: Attempting to learn from cycle data", self._name)
            await self._perform_learning(self._current_temp_in, self._current_temp_out)
        else:
            reason = self._get_no_learn_reason()
            if not is_significant_cycle:
                reason = "on_time_too_short_vs_rampup"
                
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
    def data_points(self) -> int:
        """Number of learning cycles completed (indoor coefficient updates)"""
        return self.state.coeff_indoor_autolearn
    
    @property
    def heating_cycles_count(self) -> int:
        """Number of total TPI cycles"""
        return self.state.total_cycles
    
    @property
    def time_constant(self) -> float:
        """Thermal time constant in hours"""
        if self.state.coeff_indoor > 0:
            return round(1.0 / self.state.coeff_indoor, 2)
        return 0.0
    
    @property
    def confidence(self) -> float:
        """Confidence level in the learned model (0.0 to 1.0)"""
        if self.state.coeff_indoor_autolearn == 0:
            return 0.0
        
        cycle_confidence = min(self.state.coeff_indoor_autolearn / 50.0, 1.0)
        
        if self.state.consecutive_failures > 0:
            failure_penalty = min(self.state.consecutive_failures * 0.15, 0.6)
            cycle_confidence = max(0.2, cycle_confidence - failure_penalty)
        
        return round(cycle_confidence, 2)

    async def start_learning(self, coef_int: float = None, coef_ext: float = None):
        """Start learning, optionally resetting coefficients to configured values."""
        _LOGGER.info("%s - Auto TPI: Starting learning with coef_int=%.3f, coef_ext=%.3f",
                    self._name, coef_int or self.state.coeff_indoor, coef_ext or self.state.coeff_outdoor)
        
        if coef_int is not None:
            self.state.coeff_indoor = coef_int
            self.state.coeff_indoor_autolearn = 1
        if coef_ext is not None:
            self.state.coeff_outdoor = coef_ext
            self.state.coeff_outdoor_autolearn = 0
        
        # Reset all learning data for fresh start
        self.state.last_power = 0.0
        self.state.last_order = 0.0
        self.state.last_temp_in = 0.0
        self.state.last_temp_out = 0.0
        self.state.last_state = 'stop'
        self.state.last_update_date = None
        self.state.total_cycles = 0
        self.state.consecutive_failures = 0
        self.state.last_learning_status = "learning_started"
        self.state.autolearn_enabled = True
        self.state.cycle_start_date = datetime.now()
        
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
