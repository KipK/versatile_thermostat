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

STORAGE_VERSION = 8  # Restore max_capacity into AutoTpiState


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

    # Max Capacity (physical power of the system)
    max_capacity_heat: float = 0.0
    max_capacity_cool: float = 0.0

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
                 heater_heating_time: int = 0, heater_cooling_time: int = 0,
                 calculation_method: str = "ema", ema_alpha: float = 0.2,
                 avg_initial_weight: int = 1, max_coef_int: float = 0.6,
                 heating_rate: float = 1.0, cooling_rate: float = 1.0,
                 use_capacity_as_rate: bool = False, ema_decay_rate: float = 0.1):
        self._hass = hass
        self._unique_id = unique_id
        self._name = name
        self._cycle_min = cycle_min
        self._tpi_threshold_low = tpi_threshold_low
        self._tpi_threshold_high = tpi_threshold_high
        self._heater_heating_time = heater_heating_time
        self._heater_cooling_time = heater_cooling_time

        self._calculation_method = calculation_method
        self._ema_alpha = ema_alpha
        self._avg_initial_weight = avg_initial_weight
        self._max_coef_int = max_coef_int
        self._heating_rate = heating_rate
        self._cooling_rate = cooling_rate
        self._use_capacity_as_rate = use_capacity_as_rate
        self._ema_decay_rate = ema_decay_rate

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
        self._save_lock = asyncio.Lock()

        self._timer_remove_callback: Callable[[], None] | None = None
        self._timer_capture_remove_callback: Callable[[], None] | None = None
        # data_provider: async function that returns a dict with:
        # on_time_sec, off_time_sec, on_percent, hvac_mode
        self._data_provider: Callable[[], dict] | None = None
        # event_sender: async function that sends events to thermostat
        self._event_sender: Callable[[dict], None] | None = None

    async def async_save_data(self):
        """Save data."""
        async with self._save_lock:
            _LOGGER.debug("%s - Auto TPI: requesting save data", self._name)
            await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Sync save."""
        _LOGGER.debug("%s - Auto TPI: starting sync save", self._name)
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
            _LOGGER.debug("%s - Auto TPI: sync save completed successfully", self._name)

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
                     # Initialize counters with the configured weight
                     self.state.coeff_indoor_autolearn = self._avg_initial_weight
                     self.state.coeff_indoor_cool_autolearn = self._avg_initial_weight
                     self.state.coeff_outdoor_autolearn = 0
                     self.state.coeff_outdoor_cool_autolearn = 0
                     
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

    def _get_adaptive_alpha(self, cycle_count: int) -> float:
        """Calculate adaptive alpha for EMA."""
        # α(n) = α₀ / (1 + k·n)
        alpha = self._ema_alpha / (1 + self._ema_decay_rate * cycle_count)
        return alpha

    def _should_learn(self) -> bool:
        """Check if learning should be performed."""
        if not self.state.autolearn_enabled:
            return False

        # Power conditions: 0 < last_power < 0.99
        # If power is >= 0.99 (99%), we don't learn coefficients but we detect max capacity (handled elsewhere)
        if not (0 < self.state.last_power < 0.99):
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

        # NEW: Natural drift exclusion - check temperature at CYCLE START
        # If temp was already past setpoint at cycle START, this is passive drift, not active regulation
        is_heat = self.state.last_state == 'heat'
        is_cool = self.state.last_state == 'cool'

        if is_heat and self.state.last_temp_in > self.state.last_order:
            _LOGGER.debug("%s - Auto TPI: Not learning - Passive cooling at cycle start (T_in %.2f > Target %.2f)",
                        self._name, self.state.last_temp_in, self.state.last_order)
            return False

        if is_cool and self.state.last_temp_in < self.state.last_order:
            _LOGGER.debug("%s - Auto TPI: Not learning - Passive heating at cycle start (T_in %.2f < Target %.2f)",
                        self._name, self.state.last_temp_in, self.state.last_order)
            return False

        return True

    def _get_no_learn_reason(self) -> str:
        """Get reason why learning is not happening."""
        if not self.state.autolearn_enabled:
            return "learning_disabled"
        if not (0 < self.state.last_power < 0.99):
            return f"power_out_of_range({self.state.last_power * 100:.1f}%)"
        if self.state.consecutive_failures >= 3:
            return f"too_many_failures({self.state.consecutive_failures})"
        
        # NEW: Check passive drift conditions
        is_heat = self.state.last_state == 'heat'
        is_cool = self.state.last_state == 'cool'
        if is_heat and self.state.last_temp_in > self.state.last_order:
            return f"passive_cooling(T_in={self.state.last_temp_in:.1f}>Target={self.state.last_order:.1f})"
        if is_cool and self.state.last_temp_in < self.state.last_order:
            return f"passive_heating(T_in={self.state.last_temp_in:.1f}<Target={self.state.last_order:.1f})"
            
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
        # We only learn coefficient if power is not saturated (0 < power < 0.99)
        # OR if we have overshoot (which allows reducing coeff even if saturated previously)
        # But here we stick to standard range for coeff learning to avoid windup at 100%.
        if 0 < self.state.last_power < 0.99:
            if temp_progress > 0 and target_diff > 0:
                if self._learn_indoor(target_diff, temp_progress, self._last_cycle_power_efficiency, is_cool):
                    self.state.last_learning_status = f"learned_indoor_{'cool' if is_cool else 'heat'}"
                    learned = True
        else:
             _LOGGER.debug("%s - Auto TPI: Skipping indoor coeff learning because power is saturated (%.1f%%)",
                           self._name, self.state.last_power * 100)
            
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

        # Capacity-Based Learning Logic
        # We aim to close the full temperature gap (delta_theoretical),
        # but capped by the physical capacity of the system.
        
        # 1. Define Reference Capacity (in °C/h)
        if self._use_capacity_as_rate:
            # Use auto-detected capacity (stored as Reference Capacity in °C/h)
            ref_capacity_h = self.state.max_capacity_cool if is_cool else self.state.max_capacity_heat
        else:
            # Use manual capacity from config (now in °C/h)
            ref_capacity_h = self._cooling_rate if is_cool else self._heating_rate

        # If no capacity defined, skip learning for this cycle
        if ref_capacity_h <= 0:
            _LOGGER.debug("%s - Auto TPI: Cannot learn indoor - no capacity defined (use_capacity_as_rate=%s, manual_rate=%.2f)",
                          self._name, self._use_capacity_as_rate,
                          self._cooling_rate if is_cool else self._heating_rate)
            self.state.last_learning_status = "no_capacity_defined"
            return False

        # 2. Calculate Effective Capacity with thermal losses (same logic as auto-detected)
        if is_cool:
            k_ext = self.state.coeff_outdoor_cool
            delta_t = self._current_temp_out - self._current_temp_in
        else:
            k_ext = self.state.coeff_outdoor_heat
            delta_t = self._current_temp_in - self._current_temp_out

        loss_factor = k_ext * max(0.0, delta_t)
        loss_factor = min(loss_factor, 0.95)  # Prevent going negative

        effective_capacity_h = ref_capacity_h * (1.0 - loss_factor)
        
        # 3. Calculate Max Achievable Rise in this cycle (°C)
        cycle_duration_h = self._cycle_min / 60.0
        max_achievable_rise = effective_capacity_h * cycle_duration_h * efficiency
        
        _LOGGER.debug("%s - Auto TPI: Capacity calc: ref=%.3f °C/h, loss=%.2f, eff=%.3f °C/h, max_rise=%.3f °C (cycle=%.1f min, eff=%.2f)",
                      self._name, ref_capacity_h, loss_factor, effective_capacity_h, max_achievable_rise, self._cycle_min, efficiency)

        # 4. Calculate adjusted_theoretical: aim for full gap, capped by capacity
        adjusted_theoretical = min(delta_theoretical, max_achievable_rise)
        
        if max_achievable_rise < delta_theoretical:
            mode_str = "cooling" if is_cool else "heating"
            _LOGGER.debug("%s - Auto TPI: Target rise clamped from %.3f to %.3f (Max %s Capacity)",
                          self._name, delta_theoretical, max_achievable_rise, mode_str)

        if adjusted_theoretical <= 0:
             _LOGGER.warning("%s - Auto TPI: Cannot learn indoor - adjusted_theoretical <= 0 (max_rise=%.3f, target_diff=%.3f)",
                             self._name, max_achievable_rise, delta_theoretical)
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
        MAX_COEFF = self._max_coef_int
        if coeff_new > MAX_COEFF:
            _LOGGER.info("%s - Auto TPI: Calculated indoor coeff %.3f > %.1f, capping to %.1f before averaging",
                        self._name, coeff_new, MAX_COEFF, MAX_COEFF)
            coeff_new = MAX_COEFF
            
        old_coeff = self.state.coeff_indoor_cool if is_cool else self.state.coeff_indoor_heat
        count = self.state.coeff_indoor_cool_autolearn if is_cool else self.state.coeff_indoor_autolearn
        
        # 5. Calculation Method
        if self._calculation_method == "average":
            # Weighted average
            # avg_coeff = ((old_coeff * count + coeff_new) / (count + 1))
            # We must use the current count (not incremented) as weight for old_coeff
            
            # If count is 0 (should not happen for valid state), treat as 1
            weight_old = max(count, 1)
            
            avg_coeff = ((old_coeff * weight_old) + coeff_new) / (weight_old + 1)
            _LOGGER.debug("%s - Auto TPI: Weighted Average: old=%.3f (weight=%d), new=%.3f, result=%.3f",
                          self._name, old_coeff, weight_old, coeff_new, avg_coeff)

        else: # EMA
            # EMA Smoothing (20% weight by default)
            # new_avg = (old_avg * (1 - alpha)) + (new_sample * alpha)
            alpha = self._get_adaptive_alpha(count)
            avg_coeff = (old_coeff * (1.0 - alpha)) + (coeff_new * alpha)
            _LOGGER.debug("%s - Auto TPI: EMA: old=%.3f, new=%.3f, alpha=%.2f (count=%d), result=%.3f",
                          self._name, old_coeff, coeff_new, alpha, count, avg_coeff)

        # Update counters
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

        # NEW: Validate gap_in sign matches expected regulation direction
        # In heat mode, gap_in should be >= 0 (T_in <= Target, meaning we need to heat)
        # In cool mode, gap_in should be <= 0 (T_in >= Target, meaning we need to cool)
        # If sign is wrong, we're in overshoot/undershoot territory - skip outdoor learning
        if is_cool and gap_in > 0:
            _LOGGER.debug("%s - Auto TPI: Cannot learn outdoor - Overcooled state (gap_in=%.2f > 0)",
                        self._name, gap_in)
            self.state.last_learning_status = "overcooled_gap"
            return False

        if not is_cool and gap_in < 0:
            _LOGGER.debug("%s - Auto TPI: Cannot learn outdoor - Overheated state (gap_in=%.2f < 0)",
                        self._name, gap_in)
            self.state.last_learning_status = "overheated_gap"
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
        
        # 5. Calculation Method
        if self._calculation_method == "average":
            # Weighted average
            avg_coeff = ((old_coeff * count) + coeff_new) / (count + 1)
            _LOGGER.debug("%s - Auto TPI: Outdoor Weighted Average: old=%.3f (weight=%d), new=%.3f, result=%.3f",
                          self._name, old_coeff, count, coeff_new, avg_coeff)

        else: # EMA
            # EMA Smoothing
            alpha = self._get_adaptive_alpha(count)
            avg_coeff = (old_coeff * (1.0 - alpha)) + (coeff_new * alpha)
            _LOGGER.debug("%s - Auto TPI: Outdoor EMA: old=%.3f, new=%.3f, alpha=%.2f (count=%d), result=%.3f",
                          self._name, old_coeff, coeff_new, alpha, count, avg_coeff)
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
        
        if self.learning_active:
            _LOGGER.info("%s - Auto TPI: Cycle #%d completed after %.1f minutes (efficiency: %.2f)",
                        self._name, self.state.total_cycles, elapsed_minutes, self._last_cycle_power_efficiency)
        else:
            _LOGGER.debug("%s - Auto TPI: Cycle #%d completed after %.1f minutes (efficiency: %.2f)",
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

        # Max Capacity detection (Run always, even if learning is disabled)
        _LOGGER.info("%s - Auto TPI: Invoking Max Capacity detection", self._name)
        await self._detect_max_capacity(on_time_minutes, self._current_temp_in)

        await self.async_save_data()

    async def _detect_max_capacity(self, on_time_minutes: float, current_temp_in: float):
        """Detect max capacity if we were at 100%."""
        _LOGGER.info("%s - Auto TPI: Checking Max Capacity. last_power=%.3f, last_state=%s, current_temp_in=%.2f, last_temp_in=%.2f",
                     self._name, self.state.last_power, self.state.last_state, current_temp_in, self.state.last_temp_in)

        if self.state.last_power < 0.99:
            _LOGGER.info("%s - Auto TPI: Max Capacity check skipped - Power < 0.99 (%.3f)", self._name, self.state.last_power)
            return

        is_heat = self.state.last_state == 'heat'
        is_cool = self.state.last_state == 'cool'

        if not (is_heat or is_cool):
            _LOGGER.info("%s - Auto TPI: Max Capacity check skipped - Not in heat or cool mode (%s)", self._name, self.state.last_state)
            return

        # Ignore heating capacity detection if heater was cold
        if is_heat and self.state.current_cycle_cold_factor > 0.2:
            _LOGGER.info("%s - Auto TPI: Max Capacity check skipped - Heater was cold (factor %.2f > 0.2)",
                         self._name, self.state.current_cycle_cold_factor)
            return

        # Calculate real temperature change per hour
        # We use current_temp_in (end of cycle) - last_temp_in (start of cycle)
        # Note: In cooling, temp should drop, so we take absolute diff or invert
        temp_diff = current_temp_in - self.state.last_temp_in
        if is_cool:
            temp_diff = -temp_diff
        
        _LOGGER.info("%s - Auto TPI: Max Capacity Check - Temp diff: %.3f (is_cool=%s)", self._name, temp_diff, is_cool)

        if temp_diff <= 0.01:
            _LOGGER.info("%s - Auto TPI: Max Capacity check skipped - Temp diff too small (%.3f <= 0.01)", self._name, temp_diff)
            return

        # Convert to °C/hour
        # If cycle is 10 min, we multiply by 6
        cycle_duration_h = self._cycle_min / 60.0
        if cycle_duration_h <= 0:
            return

        measured_capacity = temp_diff / cycle_duration_h

        # Normalize to Reference Capacity (Adiabatic)
        # Power_Net = Power_Input (100%) - Power_Loss
        # Power_Loss = K_ext * (Tin - Tout)
        # Capacity_Ref = Capacity_Meas / (1 - Loss)
        if is_cool:
            k_ext = self.state.coeff_outdoor_cool
            delta_t_loss = self.state.last_temp_out - self.state.last_temp_in
        else:
            k_ext = self.state.coeff_outdoor_heat
            delta_t_loss = self.state.last_temp_in - self.state.last_temp_out

        loss_factor = k_ext * max(0.0, delta_t_loss)
        # Safety clamp: avoid division by zero or extreme inflation
        loss_factor = min(loss_factor, 0.8)

        measured_ref_capacity = measured_capacity / (1.0 - loss_factor)

        _LOGGER.debug("%s - Auto TPI: Capacity Normalization: Meas=%.3f, K_ext=%.3f, dT=%.1f, Loss=%.2f, Ref=%.3f",
                      self._name, measured_capacity, k_ext, delta_t_loss, loss_factor, measured_ref_capacity)

        # EMA Smoothing (alpha 0.3)
        alpha = 0.3

        if is_cool:
            old_capacity = self.state.max_capacity_cool
            if old_capacity == 0.0:
                new_capacity = measured_ref_capacity
            else:
                new_capacity = (old_capacity * (1.0 - alpha)) + (measured_ref_capacity * alpha)

            self.state.max_capacity_cool = new_capacity
            _LOGGER.info("%s - Auto TPI: New Max Cooling Ref Capacity detected: %.3f °C/h (was: %.3f, measured_ref: %.3f)",
                         self._name, new_capacity, old_capacity, measured_ref_capacity)
        else:
            old_capacity = self.state.max_capacity_heat
            if old_capacity == 0.0:
                new_capacity = measured_ref_capacity
            else:
                new_capacity = (old_capacity * (1.0 - alpha)) + (measured_ref_capacity * alpha)

            self.state.max_capacity_heat = new_capacity
            _LOGGER.info("%s - Auto TPI: New Max Heating Ref Capacity detected: %.3f °C/h (was: %.3f, measured_ref: %.3f)",
                         self._name, new_capacity, old_capacity, measured_ref_capacity)
    
    def get_calculated_params(self) -> dict:
        return self._calculated_params
    
    @property
    def learning_active(self) -> bool:
        return self.state.autolearn_enabled
    
    @property
    def int_cycles(self) -> int:
        """Number of ACTUAL learning cycles completed for internal coefficient"""
        is_cool_mode = self._current_hvac_mode == 'cool' or self._current_hvac_action == 'cool'
        if is_cool_mode:
            return max(0, self.state.coeff_indoor_cool_autolearn - self._avg_initial_weight)
        return max(0, self.state.coeff_indoor_autolearn - self._avg_initial_weight)

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
        
        # Update coefficients if provided
        if coef_int is not None:
            self.state.coeff_indoor_heat = coef_int
            self.state.coeff_indoor_cool = coef_int  # Also reset cool coefficient
        if coef_ext is not None:
            self.state.coeff_outdoor_heat = coef_ext
            self.state.coeff_outdoor_cool = coef_ext  # Also reset cool coefficient
        
        # ALWAYS reset ALL counters (moved outside the if blocks)
        self.state.coeff_indoor_autolearn = self._avg_initial_weight
        self.state.coeff_outdoor_autolearn = 0
        self.state.coeff_indoor_cool_autolearn = self._avg_initial_weight
        self.state.coeff_outdoor_cool_autolearn = 0
        
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
