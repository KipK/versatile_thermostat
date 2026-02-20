# pylint: disable=line-too-long, abstract-method
"""Smart PI algorithm handler for ThermostatProp."""

import logging
import time
from typing import TYPE_CHECKING
from homeassistant.util import slugify
from homeassistant.helpers.storage import Store
from homeassistant.helpers.event import async_track_time_interval
from datetime import timedelta, datetime

from .prop_algo_smartpi import SmartPI
from .smartpi.const import (
    SMARTPI_RECALC_INTERVAL_SEC,
    SmartPIPhase,
    SmartPICalibrationPhase,
    SmartPICalibrationResult,
    NEAR_BAND_HYSTERESIS_C,
)
from .const import (
    CONF_MINIMAL_ACTIVATION_DELAY,
    CONF_MINIMAL_DEACTIVATION_DELAY,
    CONF_MAX_ON_PERCENT,
    CONF_SMART_PI_DEADBAND,
    CONF_SMART_PI_USE_SETPOINT_FILTER,
    CONF_SMART_PI_HYSTERESIS_ON,
    CONF_SMART_PI_HYSTERESIS_OFF,
    CONF_SMART_PI_DEBUG,
    EventType,
)
from .vtherm_hvac_mode import VThermHvacMode_OFF, VThermHvacMode_HEAT, VThermHvacMode_COOL
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
        # Track calibration state for completion detection
        self._prev_is_calibrating: bool = False

    def init_algorithm(self):
        """Initialize SmartPI algorithm."""
        t = self._thermostat
        entry = t.entry_infos

        # Initialize storage with slugified name to allow retrieval if re-created
        # STORAGE_KEY is "versatile_thermostat.smartpi.{}"
        safe_name = slugify(t.name)
        self._store = Store(t.hass, STORAGE_VERSION, STORAGE_KEY.format(safe_name))

        # Use the thermostat's cycle_min property directly for consistency
        # (base_thermostat reads CONF_CYCLE_MIN in __init__)
        cycle_min = t.cycle_min
        minimal_activation_delay = entry.get(CONF_MINIMAL_ACTIVATION_DELAY, 0)
        minimal_deactivation_delay = entry.get(CONF_MINIMAL_DEACTIVATION_DELAY, 0)
        max_on_percent = entry.get(CONF_MAX_ON_PERCENT, 1.0)

        # Update thermostat attributes directly (like TPIHandler)
        t.minimal_activation_delay = minimal_activation_delay
        t.minimal_deactivation_delay = minimal_deactivation_delay

        # SmartPI specific
        deadband = entry.get(CONF_SMART_PI_DEADBAND, 0.05)
        use_setpoint_filter = entry.get(CONF_SMART_PI_USE_SETPOINT_FILTER, True)
        hyst_on = entry.get(CONF_SMART_PI_HYSTERESIS_ON, 0.3)
        hyst_off = entry.get(CONF_SMART_PI_HYSTERESIS_OFF, 0.5)
        debug_mode = entry.get(CONF_SMART_PI_DEBUG, False)

        # Create SmartPI instance
        # Note: saved_state is loaded asynchronously later
        t.prop_algorithm = SmartPI(
            hass=t.hass,
            cycle_min=cycle_min,
            minimal_activation_delay=minimal_activation_delay,
            minimal_deactivation_delay=minimal_deactivation_delay,
            name=t.name,
            max_on_percent=max_on_percent,
            deadband_c=deadband,
            use_setpoint_filter=use_setpoint_filter,
            hysteresis_on=hyst_on,
            hysteresis_off=hyst_off,
            debug_mode=debug_mode,
        )

        _LOGGER.info("%s - SmartPI Algorithm initialized", t)

    async def async_added_to_hass(self):
        """Load persistent data."""
        t = self._thermostat
        if self._store:
            try:
                data = await self._store.async_load()
                if data and t.prop_algorithm:
                    t.prop_algorithm.load_state(data)
                    _LOGGER.debug("%s - SmartPI state loaded", t)
            except Exception as e:
                _LOGGER.error("%s - Failed to load SmartPI state: %s", t, e)

    async def async_startup(self):
        """Startup actions."""
        # Initialize the cycle start state if possible to enable first-cycle learning
        t = self._thermostat
        if t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
            # Check availability of sensors
            if t.current_temperature is not None and t.current_outdoor_temperature is not None:
                # Use restored u_applied if available, else 0
                # CycleManager handles initialization automatically in process_cycle
                _LOGGER.debug("%s - SmartPI startup: ready for cycle management", t)

        # Check if we need to start the periodic recalculation timer
        await self.on_state_changed()

    async def _async_save(self):
        """Save SmartPI state to storage."""
        t = self._thermostat
        if self._store and t.prop_algorithm:
            try:
                data = t.prop_algorithm.save_state()
                t.hass.async_create_task(self._store.async_save(data))
                _LOGGER.debug("%s - SmartPI state saved", t)
            except Exception as e:
                _LOGGER.error("%s - Failed to save SmartPI state: %s", t, e)

    def remove(self):
        """Cleanup and save state on removal."""
        t = self._thermostat
        if self._store and t.prop_algorithm:
            # We can't await here easily, but we schedule save
            t.hass.async_create_task(self._async_save())

        self._stop_recalc_timer()

    async def control_heating(self, timestamp=None, force=False):
        """Control heating using SmartPI."""
        t = self._thermostat
        from datetime import datetime
        from .timing_utils import calculate_cycle_times
        from .smartpi.guards import GuardAction

        if t.prop_algorithm:
            # Learning update
            current_temp = t.current_temperature

            # --- Guard Cut ---
            algo = t.prop_algorithm
            guard_cut_action = algo.guards.check_guard_cut(
                current_temp=current_temp,
                target_temp=t.target_temperature,
                near_band_above=algo._near_band_above_deg,
                in_near_band=algo.in_near_band,
                is_device_active=any(under.is_device_active for under in t.underlyings),
                hvac_mode=t.vtherm_hvac_mode,
                is_calibration=(algo.phase == SmartPIPhase.CALIBRATION)
            )

            if guard_cut_action == GuardAction.CUT_TRIGGER:
                force = True

            # --- Guard Kick ---
            guard_kick_action = algo.guards.check_guard_kick(
                current_temp=current_temp,
                target_temp=t.target_temperature,
                near_band_below=algo._near_band_below_deg,
                in_near_band=algo.in_near_band,
                on_percent=algo.on_percent,
                hvac_mode=t.vtherm_hvac_mode,
                is_calibration=(algo.phase == SmartPIPhase.CALIBRATION)
            )

            if guard_kick_action == GuardAction.KICK_TRIGGER:
                force = True

            # Calculate uses current temp, ext temp, etc.
            # If guard_cut is active, calculate() will set on_percent=0.
            t.prop_algorithm.calculate(
                target_temp=t.target_temperature,
                current_temp=t.current_temperature,
                ext_current_temp=t.current_outdoor_temperature,
                slope=t.last_temperature_slope,
                hvac_mode=t.vtherm_hvac_mode,
                power_shedding=t.power_manager.is_overpowering_detected if t.power_manager else False,
            )

            # Trigger learning only on cycle timer (timestamp is not None)
            # And do not learn if we are OFF (window open, etc.)
            if timestamp is not None and current_temp is not None and t.vtherm_hvac_mode != VThermHvacMode_OFF:
                async def _data_provider():
                    # 1. Get requested percentage from algorithm
                    requested_on_percent = t.prop_algorithm.on_percent

                    # 2. Calculate timing with constraints
                    on_time_sec, off_time_sec, forced_by_timing = calculate_cycle_times(
                        requested_on_percent,
                        t.cycle_min,
                        t.minimal_activation_delay,
                        t.minimal_deactivation_delay
                    )

                    # 3. Derive realized percentage
                    realized_on_percent = on_time_sec / (t.cycle_min * 60)

                    # 4. Notify algorithm of realized result for closed-loop anti-windup/tracking
                    # We calculate dt_min here as it's needed for anti-windup
                    dt_min = 0.0
                    if timestamp:
                        ts = timestamp.timestamp() if isinstance(timestamp, datetime) else timestamp
                        if t.prop_algorithm._last_calculate_time:
                            dt_min = (ts - t.prop_algorithm._last_calculate_time) / 60.0

                    if hasattr(t.prop_algorithm, "update_realized_power"):
                        t.prop_algorithm.update_realized_power(realized_on_percent, forced_by_timing, dt_min)

                    return {
                        "temp_in": t.current_temperature,
                        "temp_ext": t.current_outdoor_temperature,
                        "timestamp": timestamp,
                        "on_percent": realized_on_percent, # Use realized percent for learning
                        "on_time_sec": on_time_sec,
                        "off_time_sec": off_time_sec,
                        "hvac_mode": t.vtherm_hvac_mode
                    }

                async def _event_sender(params):
                    # Events are applied by the handler below (lines 165+)
                    pass

                await t.prop_algorithm.process_cycle(timestamp, _data_provider, _event_sender, force)

        # Stop here if we are off
        if t.vtherm_hvac_mode == VThermHvacMode_OFF:
            _LOGGER.debug("%s - End of cycle (HVAC_MODE_OFF)", t)
            t._on_time_sec = 0
            t._off_time_sec = int(t.cycle_min * 60)
            if t.is_device_active:
                await t.async_underlying_entity_turn_off()
        else:
            # Calculate timing for underlying entities
            if t.prop_algorithm:
                on_percent = t.prop_algorithm.on_percent
                on_time_sec, off_time_sec, _ = calculate_cycle_times(
                    on_percent,
                    t.cycle_min,
                    t.minimal_activation_delay,
                    t.minimal_deactivation_delay
                )
            else:
                on_time_sec = None
                off_time_sec = None
                on_percent = None

            # Check if on_percent has changed
            new_on_percent = t.prop_algorithm.on_percent
            on_percent_changed = abs(new_on_percent - self._last_on_percent) > 0.001
            self._last_on_percent = new_on_percent

            # Store on/off times on thermostat for sensors and attributes
            t._on_time_sec = on_time_sec
            t._off_time_sec = off_time_sec

            for under in t.underlyings:
                await under.start_cycle(
                    t.vtherm_hvac_mode,
                    on_time_sec,
                    off_time_sec,
                    on_percent,
                    force or (t.prop_algorithm.phase == SmartPIPhase.CALIBRATION) or (t.prop_algorithm.phase == SmartPIPhase.HYSTERESIS and on_percent_changed),
                )

        # Save state after cycle to persist learning data
        await self._async_save()

        # --- AutoCalibTrigger: hourly check ---
        if t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
            algo = t.prop_algorithm
            now_wall = time.time()

            # Detect calibration completion (CALIBRATING -> IDLE transition)
            currently_calibrating = algo.calibration_state != SmartPICalibrationPhase.IDLE
            if self._prev_is_calibrating and not currently_calibrating:
                # Calibration just ended
                ac_event = algo.autocalib.on_calibration_complete(
                    now_wall=now_wall,
                    algo=algo,
                    result=algo.calibration_mgr.calibration_result,
                )
                if ac_event is not None:
                    t.hass.bus.async_fire(ac_event.event_type, ac_event.payload)
                    _LOGGER.info(
                        "%s - AutoCalib event fired: %s", t.name, ac_event.event_type
                    )
            self._prev_is_calibrating = currently_calibrating

            # Hourly stagnation check
            ac_event = algo.autocalib.check_hourly(
                now_wall=now_wall,
                algo=algo,
                ext_temp=t.current_outdoor_temperature,
                current_temp=t.current_temperature,
            )
            if ac_event is not None:
                t.hass.bus.async_fire(ac_event.event_type, ac_event.payload)
                _LOGGER.info(
                    "%s - AutoCalib event fired: %s", t.name, ac_event.event_type
                )
                if ac_event.should_trigger_calibration:
                    # AutoCalibTrigger decided to start a calibration
                    algo.calibration_mgr.request_calibration(phase=algo.phase)
                    _LOGGER.warning(
                        "%s - AutoCalib: calibration requested by supervisor", t.name
                    )

    async def on_state_changed(self):
        """Handle state changes."""
        t = self._thermostat
        if t.vtherm_hvac_mode in [VThermHvacMode_HEAT, VThermHvacMode_COOL]:
            # Check if we're resuming from OFF (timer was stopped)
            timer_was_stopped = getattr(t, "_smartpi_recalc_timer_remove", None) is None

            self._start_recalc_timer()

            # When resuming from OFF state (e.g., window close), reset the cycle start state
            # to prevent using stale timestamps. CycleManager will re-init on next process_cycle.
            if timer_was_stopped and t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
                # Force CycleManager to re-initialize by clearing start date
                # Note: accessing _cycle_start_date as it is an internal state of CycleManager
                t.prop_algorithm._cycle_start_date = None
                t.prop_algorithm._reset_learning_window()
                _LOGGER.debug("%s - SmartPI resumed from OFF: cycle and learning window reset", t.name)
        else:
            self._stop_recalc_timer()
            # Cancel any ongoing calibration when switching to OFF/SLEEP mode
            await self._cancel_calibration_if_active()

    def _start_recalc_timer(self):
        """Start the periodic recalculation timer."""
        t = self._thermostat
        if getattr(t, "_smartpi_recalc_timer_remove", None):
            return

        async def _recalc_callback(now):
            _LOGGER.debug("%s - SmartPI periodic calculation trigger", t)
            # Force control_heating but without timestamp to avoid triggering learning
            # The learning should only be triggered by the cycle manager (every cycle_min)
            # automatic recalculation is done inside control_heating
            await t.async_control_heating(timestamp=None)

        t._smartpi_recalc_timer_remove = async_track_time_interval(
            t.hass,
            _recalc_callback,
            timedelta(seconds=SMARTPI_RECALC_INTERVAL_SEC)
        )
        _LOGGER.debug("%s - SmartPI calc timer started", t)

    def _stop_recalc_timer(self):
        """Stop the periodic recalculation timer."""
        t = self._thermostat
        remove_callback = getattr(t, "_smartpi_recalc_timer_remove", None)
        if remove_callback:
            remove_callback()
            t._smartpi_recalc_timer_remove = None
            _LOGGER.debug("%s - SmartPI calc timer stopped", t)

    async def _cancel_calibration_if_active(self):
        """Cancel any ongoing calibration (manual or auto) when HVAC is turned off."""
        t = self._thermostat
        algo = t.prop_algorithm
        if not algo or not isinstance(algo, SmartPI):
            return

        if not algo.calibration_mgr.is_calibrating:
            return

        _LOGGER.info("%s - HVAC OFF: canceling ongoing calibration", t.name)

        # Reset the calibration manager
        algo.calibration_mgr.reset()

        # Notify autocalib trigger about the cancellation
        now_wall = time.time()
        event = algo.autocalib.on_calibration_complete(now_wall, algo, SmartPICalibrationResult.CANCELLED)

        # Fire event if autocalib returns one
        if event:
            t.hass.bus.fire(EventType.SMART_PI_EVENT.value, {"entity_id": t.entity_id, "type": event.event_type, "data": event.data or {}})

        # Reset tracking state
        self._prev_is_calibrating = False

        # Update attributes and save state
        self.update_attributes()
        t.async_write_ha_state()
        await self._async_save()

    def update_attributes(self):
        """Add SmartPI-specific attributes."""
        t = self._thermostat
        if t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
            algo = t.prop_algorithm
            # Retrieve diagnostics from algorithm (base)
            diag_data = algo.get_diagnostics()

            # Merge Handler-specific formatting (e.g. timestamps from Handler/Manager)
            # Note: algo.get_diagnostics() already returns ISO strings for timestamps managed by algo
            # We override or add only what is specific to the handler context if needed.

            # We want to ensure specific formatting for `last_calibration_time` if it's not already in ISO
            # algo.get_diagnostics() returns it as ISO string if available in `smartpi/diagnostics.py`,
            # but let's double check `prop_handler` was doing it manually.
            # In `diagnostics.py`: "last_calibration_time": ... isoformat() ...
            # So we don't need to re-do it here.

            t._attr_extra_state_attributes["specific_states"]["smart_pi"] = diag_data

            # Add to configuration dict for consistency with TPI
            t._attr_extra_state_attributes["configuration"].update({
                "minimal_activation_delay_sec": t.minimal_activation_delay,
                "minimal_deactivation_delay_sec": t.minimal_deactivation_delay,
            })

    async def service_reset_smart_pi_learning(self):
        """Reset learning data."""
        t = self._thermostat
        if t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
            t.prop_algorithm.reset_learning()
            t.hass.bus.fire(EventType.SMART_PI_EVENT.value, {
                 "entity_id": t.entity_id,
                 "type": "learning_reset"
             })
            write_event_log(_LOGGER, t, "SmartPI learning reset")
            self.update_attributes()
            t.async_write_ha_state()
            await self._async_save()

    async def service_force_smartpi_calibration(self):
        """Force calibration."""
        t = self._thermostat
        if t.prop_algorithm and isinstance(t.prop_algorithm, SmartPI):
            event = t.prop_algorithm.force_calibration()
            t.hass.bus.fire(EventType.SMART_PI_EVENT.value, {
                 "entity_id": t.entity_id,
                 "type": "force_calibration"
             })
            if event is not None:
                t.hass.bus.fire(event.event_type, event.payload)

            write_event_log(_LOGGER, t, "SmartPI forced calibration triggered")
            # Force immediate recalculation to update state
            await self.control_heating(force=True)
            self.update_attributes()
            t.async_write_ha_state()
            await self._async_save()

            _LOGGER.debug("%s - AutoCalib: manual calibration triggered, will check exit on completion", t.name)
