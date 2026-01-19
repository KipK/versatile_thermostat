# tests/test_smartpi_multi_switch_bug.py

import pytest
import logging
from unittest.mock import patch, ANY, call
from datetime import datetime, timedelta

from custom_components.versatile_thermostat.base_thermostat import BaseThermostat
from custom_components.versatile_thermostat.const import *
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT
from .commons import *

logging.getLogger().setLevel(logging.DEBUG)
_LOGGER = logging.getLogger(__name__)

@pytest.mark.asyncio
async def test_smartpi_multi_switch_hysteresis_fix(hass: HomeAssistant, skip_hass_states_is_state):
    """
    Test that the bug where periodic recalculations in Smart-PI Hysteresis mode
    reset the start delay for underlying switches is fixed.
    """
    tz = get_tz(hass)
    now = datetime.now(tz=tz)

    # 1. Configure Thermostat with Smart-PI and 2 switches
    # Cycle is 10 min. 2 switches -> delay 0 and delay 5 min (300s).
    # Recalculation is every 60s. 300s > 60s, so the second switch should be affected.
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="SmartPIMultiSwitch",
        unique_id="uniqueId",
        data={
            CONF_NAME: "SmartPIMultiSwitch",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 10,
            CONF_TEMP_MIN: 15,
            CONF_TEMP_MAX: 30,
            CONF_USE_WINDOW_FEATURE: False,
            CONF_USE_MOTION_FEATURE: False,
            CONF_USE_POWER_FEATURE: False,
            CONF_USE_PRESENCE_FEATURE: False,
            CONF_HEATER: "switch.mock_switch1",
            CONF_HEATER_2: "switch.mock_switch2",
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_SMART_PI,
            CONF_SMART_PI_DEADBAND: 0.05, # Ensure we are not in deadband too easily
        },
    )

    entity: BaseThermostat = await create_thermostat(hass, entry, "climate.smartpimultiswitch")
    assert entity
    assert entity.nb_underlying_entities == 2
    
    # Verify delays
    # switch1: 0s
    # switch2: 10*60/2 = 300s
    assert entity.underlying_entity(0).initial_delay_sec == 0
    assert entity.underlying_entity(1).initial_delay_sec == 300

    # 2. Set mode to HEAT and target temp to trigger heating (Hysteresis check)
    # Target 20, current 15 -> Error 5 -> should be ON 100% in Hysteresis
    await entity.async_set_hvac_mode(VThermHvacMode_HEAT)
    await entity.async_set_temperature(temperature=20)
    
    with patch("custom_components.versatile_thermostat.base_thermostat.BaseThermostat.send_event"):
        await send_temperature_change_event(entity, 15, now)
        await send_ext_temperature_change_event(entity, 5, now)

    # force calculation to ensure phase is HYSTERESIS
    assert entity.prop_algorithm.phase == "Hysteresis"
    assert entity.on_percent == 1.0

    # Mock call_later to capture tasks
    # We want to check if call_later is called repeatedly for switch 2
    
    # We simulate periodic calls to control_heating(timestamp=None)
    # This happens every 60s in real life via SMARTPI_RECALC_INTERVAL_SEC
    
    with patch("custom_components.versatile_thermostat.underlyings.UnderlyingSwitch.call_later") as mock_call_later, \
         patch("custom_components.versatile_thermostat.underlyings.UnderlyingSwitch.turn_on") as mock_turn_on:
        
        # Initial call (already done via temp change technically, but let's be sure)
        # Note: temp change triggered control_heating(timestamp=now)
        # Periodic calls trigger control_heating(timestamp=None)
        
        # We manually call control_heating(timestamp=None) multiple times simulating time passing
        for i in range(5): # 5 minutes passing
            _LOGGER.info(f"--- Minute {i} ---")
            await entity.async_control_heating(timestamp=None)
            
            # Check if switch 2 had its cycle cancelled and restarted
            # underlying_entity(1) -> switch2
            # It should call call_later with delay 300 EACH TIME if the bug exists
            # Because force=True causes _cancel_cycle() then call_later()
            pass
            
        # If bug exists, call_later is called at least 5 times for switch 2 with delay ~300
        # And importantly, the previous task is cancelled.
        
        # Let's count calls to call_later for the second switch (delay=300)
        # mock_call_later(hass, delay, callback)
        
        _LOGGER.info(f"All calls: {mock_call_later.call_args_list}")
        calls_switch_2 = [
            c for c in mock_call_later.call_args_list
            if len(c.args) > 1 and c.args[1] == 300
        ]
        
        _LOGGER.info(f"Calls to call_later for switch 2 (delay 300): {len(calls_switch_2)}")
        
        # With the fix, we expect call_later NOT to be called repeatedly
        # Ideally only once (start_cycle initial)
        # In this test setup, async_control_heating(timestamp=None) might trigger ONE call if logic is weird,
        # but definitely not every minute.
        assert len(calls_switch_2) <= 1, f"Bug present: call_later called {len(calls_switch_2)} times for switch 2"

