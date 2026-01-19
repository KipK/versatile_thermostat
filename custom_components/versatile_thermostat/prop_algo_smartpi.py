# pylint: disable=line-too-long, abstract-method
"""Smart PI algorithm handler for ThermostatProp."""

import logging
from typing import Any, TYPE_CHECKING
from homeassistant.helpers.storage import Store
from homeassistant.exceptions import ServiceValidationError

from .smartpi_algorithm import SmartPI
from .const import (
    CONF_CYCLE_MIN,
    CONF_MINIMAL_ACTIVATION_DELAY,
    CONF_MINIMAL_DEACTIVATION_DELAY,
    CONF_MAX_ON_PERCENT,
    CONF_SMART_PI_DEADBAND,
    CONF_SMART_PI_AGGRESSIVENESS,
    CONF_SMART_PI_USE_SETPOINT_FILTER,
    EventType,
)
from .vtherm_hvac_mode import VThermHvacMode_OFF
from .commons import write_event_log

if TYPE_CHECKING:
    from .thermostat_prop import ThermostatProp

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
STORAGE_KEY = "versatile_thermostat.smartpi.{}"

class SmartPIHandler:
    """Handler for SmartPI-specific logic."""

    def __init__(self, thermostat: "ThermostatProp"):
        """Initialize handler with parent thermostat reference."""
        self._thermostat = thermostat
        self._store: Store | None = None
        # State for learning
        self._last_temp = None
        self._last_ext_temp = None
        self._last_time = None
        self._last_on_percent = 0.0

    def init_algorithm(self):
        """Initialize SmartPI algorithm."""
        t = self._thermostat
        entry = t._entry_infos

        # Initialize storage
        self._store = Store(t._hass, STORAGE_VERSION, STORAGE_KEY.format(t.unique_id))

        # Read config (mirroring how it was done for TPI, but for SmartPI params)
        cycle_min = entry.get(CONF_CYCLE_MIN, 5)
        minimal_activation_delay = entry.get(CONF_MINIMAL_ACTIVATION_DELAY, 0)
        minimal_deactivation_delay = entry.get(CONF_MINIMAL_DEACTIVATION_DELAY, 0)
        max_on_percent = entry.get(CONF_MAX_ON_PERCENT, 1.0)
        
        # SmartPI specific
        deadband = entry.get(CONF_SMART_PI_DEADBAND, 0.05)
        aggressiveness = entry.get(CONF_SMART_PI_AGGRESSIVENESS, 0.5)
        use_setpoint_filter = entry.get(CONF_SMART_PI_USE_SETPOINT_FILTER, True)

        # Create SmartPI instance
        # Note: saved_state is loaded asynchronously later
        t._prop_algorithm = SmartPI(
            cycle_min=cycle_min,
            minimal_activation_delay=minimal_activation_delay,
            minimal_deactivation_delay=minimal_deactivation_delay,
            name=t.name,
            max_on_percent=max_on_percent,
            deadband_c=deadband,
            aggressiveness=aggressiveness,
            use_setpoint_filter=use_setpoint_filter,
        )
        
        _LOGGER.info("%s - SmartPI Algorithm initialized", t)

    async def async_added_to_hass(self):
        """Load persistent data."""
        t = self._thermostat
        if self._store:
            try:
                data = await self._store.async_load()
                if data and t._prop_algorithm:
                    t._prop_algorithm.load_state(data)
                    _LOGGER.debug("%s - SmartPI state loaded", t)
            except Exception as e:
                _LOGGER.error("%s - Failed to load SmartPI state: %s", t, e)

    async def async_startup(self):
        """Startup actions."""
        # SmartPI doesn't need a specific background loop like AutoTPI, 
        # but we might want to ensure everything is ready.
        pass

    def remove(self):
        """Cleanup and save state on removal."""
        t = self._thermostat
        if self._store and t._prop_algorithm:
            try:
                data = t._prop_algorithm.save_state()
                t.hass.async_create_task(self._store.async_save(data))
                _LOGGER.debug("%s - SmartPI state saved", t)
            except Exception as e:
                _LOGGER.error("%s - Failed to save SmartPI state: %s", t, e)

    async def control_heating(self, timestamp=None, force=False):
        """Control heating using SmartPI."""
        t = self._thermostat
        from datetime import datetime
        import time

        if t._prop_algorithm:
             # Learning update
             # We need current values
             current_temp = t._cur_temp
             current_ext_temp = t._cur_ext_temp
             now = time.time()
             
             # Trigger learning only on cycle timer (timestamp is not None)
             if timestamp is not None and (
                 current_temp is not None 
                 and self._last_temp is not None 
                 and self._last_time is not None
                 # Ensure we have a valid previous power (default 0.0)
             ):
                 dt_m = (now - self._last_time) / 60.0
                 # Only learn if we are heating? SmartPI handles this check (hvac_mode check)
                 # We simply pass data
                 t._prop_algorithm.update_learning(
                     current_temp=current_temp,
                     ext_current_temp=current_ext_temp if current_ext_temp is not None else 0.0, # fallback
                     previous_temp=self._last_temp,
                     previous_power=self._last_on_percent,
                     hvac_mode=t.vtherm_hvac_mode,
                     cycle_dt=dt_m,
                     ext_previous_temp=self._last_ext_temp
                 )

             # Calculate uses current temp, ext temp, etc.
             t._prop_algorithm.calculate(
                target_temp=t.target_temperature,
                current_temp=t._cur_temp,
                ext_current_temp=t._cur_ext_temp,
                slope=t.last_temperature_slope,
                hvac_mode=t.vtherm_hvac_mode,
            )
             
             # Update last state (only if we just learned/completed a cycle)
             # This ensures dt matches the cycle duration next time
             if timestamp is not None and current_temp is not None:
                 self._last_temp = current_temp
                 self._last_time = now
                 self._last_ext_temp = current_ext_temp
                 self._last_on_percent = t._prop_algorithm.on_percent
             elif self._last_time is None and current_temp is not None:
                 # Initialize on first run
                 self._last_temp = current_temp
                 self._last_time = now
                 self._last_ext_temp = current_ext_temp
                 self._last_on_percent = t._prop_algorithm.on_percent

        # Stop here if we are off
        if t.vtherm_hvac_mode == VThermHvacMode_OFF:
            _LOGGER.debug("%s - End of cycle (HVAC_MODE_OFF)", t)
            if t.is_device_active:
                await t.async_underlying_entity_turn_off()
        else:
            for under in t._underlyings:
                await under.start_cycle(
                    t.vtherm_hvac_mode,
                    t._prop_algorithm.on_time_sec if t._prop_algorithm else None,
                    t._prop_algorithm.off_time_sec if t._prop_algorithm else None,
                    t._prop_algorithm.on_percent if t._prop_algorithm else None,
                    force,
                )

    async def on_state_changed(self):
        """Handle state changes."""
        # SmartPI executes on calculate(), which is triggered by thermostat update_events.
        # No special loop start/stop needed unless we want periodic recalc?
        # The prompt mentioned "SmartPi Call Frequency Verification" in history.
        # If we need periodic recalc, we might need a timer here.
        # But base ThermostatProp typically triggers recalculate on temp change.
        pass

    def update_attributes(self):
        """Add SmartPI-specific attributes."""
        t = self._thermostat
        if t._prop_algorithm and isinstance(t._prop_algorithm, SmartPI):
            algo = t._prop_algorithm
            t._attr_extra_state_attributes["specific_states"]["smart_pi"] = {
                "a": algo.a,
                "b": algo.b,
                "tau_min": algo.tau_min,
                "tau_reliable": algo.tau_reliable,
                "learn_ok_count": algo.learn_ok_count,
                "learn_ok_count_a": algo.learn_ok_count_a,
                "learn_ok_count_b": algo.learn_ok_count_b,
                "learn_skip_count": algo.learn_skip_count,
                "learn_last_reason": algo.learn_last_reason,
                "learning_start_dt": algo.learning_start_dt,
                "Kp": algo.kp,
                "Ki": algo.ki,
                "integral_error": algo.integral_error,
                "i_mode": algo.i_mode,
                "sat": algo.sat,
                "error": algo.error,
                "error_p": algo.error_p,
                "error_filtered": algo.error_filtered,
                "setpoint_weight_b": algo.setpoint_weight_b,
                "near_band_deg": algo.near_band_deg,
                "kp_near_factor": algo.kp_near_factor,
                "ki_near_factor": algo.ki_near_factor,
                "sign_flip_leak": algo.sign_flip_leak,
                "sign_flip_active": algo.sign_flip_active,
                "u_ff": algo.u_ff,
                "u_pi": algo.u_pi,
                "ff_warmup_ok_count": algo.ff_warmup_ok_count,
                "ff_warmup_cycles": algo.ff_warmup_cycles,
                "ff_scale_unreliable_max": algo.ff_scale_unreliable_max,
                "cycles_since_reset": algo.cycles_since_reset,
                "on_percent": algo.on_percent,
                "on_time_sec": algo.on_time_sec,
                "off_time_sec": algo.off_time_sec,
                "cycle_min": algo._cycle_min,
                "filtered_setpoint": algo.filtered_setpoint,
                "learning_resume_ts": algo.learning_resume_ts,
                "u_cmd": algo.u_cmd,
                "u_limited": algo.u_limited,
                "u_applied": algo.u_applied,
                "aw_du": algo.aw_du,
                "forced_by_timing": algo.forced_by_timing,
                "in_deadband": algo.in_deadband,
                "setpoint_boost_active": algo.setpoint_boost_active,
            }

    async def service_reset_smart_pi_learning(self):
        """Reset learning data."""
        t = self._thermostat
        if t._prop_algorithm and isinstance(t._prop_algorithm, SmartPI):
             t._prop_algorithm.reset_learning()
             write_event_log(_LOGGER, t, "SmartPI learning reset")
