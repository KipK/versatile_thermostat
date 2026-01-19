""" Test the SmartPI periodic recalculation timer """
import logging
from datetime import datetime, timedelta
from unittest.mock import patch

import pytest
from homeassistant.core import HomeAssistant
from homeassistant.components.climate import HVACMode
from homeassistant.util import dt as dt_util

# Correct import for firing time changed events in tests
from pytest_homeassistant_custom_component.common import async_fire_time_changed, MockConfigEntry

from custom_components.versatile_thermostat.base_thermostat import BaseThermostat
from custom_components.versatile_thermostat.const import (
    DOMAIN,
    CONF_NAME,
    CONF_THERMOSTAT_TYPE,
    CONF_THERMOSTAT_SWITCH,
    CONF_TEMP_SENSOR,
    CONF_EXTERNAL_TEMP_SENSOR,
    CONF_CYCLE_MIN,
    CONF_TEMP_MIN,
    CONF_TEMP_MAX,
    CONF_USE_WINDOW_FEATURE,
    CONF_USE_MOTION_FEATURE,
    CONF_USE_POWER_FEATURE,
    CONF_USE_PRESENCE_FEATURE,
    CONF_UNDERLYING_LIST,
    CONF_PROP_FUNCTION,
    PROPORTIONAL_FUNCTION_SMART_PI,
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
    CONF_MINIMAL_ACTIVATION_DELAY,
    CONF_MINIMAL_DEACTIVATION_DELAY,
)
from custom_components.versatile_thermostat.smartpi_algorithm import SMARTPI_RECALC_INTERVAL_SEC

from .commons import create_thermostat

@pytest.mark.parametrize("expected_lingering_tasks", [True])
@pytest.mark.parametrize("expected_lingering_timers", [True])
async def test_smartpi_periodic_recalc(
    hass: HomeAssistant, skip_hass_states_is_state: None, skip_turn_on_off_heater
):
    """Test that SmartPI recalculates periodically independently of sensor updates."""
    
    # 1. Setup VTherm with SmartPI
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="SmartPIRecalcTest",
        unique_id="smartpi_recalc_test",
        data={
            CONF_NAME: "SmartPIRecalcTest",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 10, # 10 min cycle
            CONF_TEMP_MIN: 15,
            CONF_TEMP_MAX: 30,
            CONF_USE_WINDOW_FEATURE: False,
            CONF_USE_MOTION_FEATURE: False,
            CONF_USE_POWER_FEATURE: False,
            CONF_USE_PRESENCE_FEATURE: False,
            CONF_UNDERLYING_LIST: ["switch.mock_switch"],
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_SMART_PI,
            CONF_TPI_COEF_INT: 0.6,
            CONF_TPI_COEF_EXT: 0.01,
            CONF_MINIMAL_ACTIVATION_DELAY: 30,
            CONF_MINIMAL_DEACTIVATION_DELAY: 0,
        },
    )

    entity: BaseThermostat = await create_thermostat(
        hass, entry, "climate.smartpirecalctest"
    )
    assert entity
    assert entity._prop_algorithm
    assert entity._proportional_function == PROPORTIONAL_FUNCTION_SMART_PI

    # Spy on recalculate method
    with patch.object(entity, 'recalculate', wraps=entity.recalculate) as mock_recalculate:
        
        # 2. Set to HEAT mode to start the timer
        print(f"DEBUG: Entity type: {type(entity)}")
        print(f"DEBUG: MRO: {type(entity).mro()}")
        if hasattr(entity.update_states, '__func__'):
            print(f"DEBUG: update_states qualname: {entity.update_states.__func__.__qualname__}")
        else:
             print(f"DEBUG: update_states: {entity.update_states}")
        
        await entity.async_set_hvac_mode(HVACMode.HEAT)
        await hass.async_block_till_done()

        # Timer should be started
        assert entity._smartpi_recalc_timer_remove is not None, \
            f"Timer not started. Mode={entity.vtherm_hvac_mode}, " \
            f"PropFunc={entity._proportional_function}, " \
            f"AutoTpiMgr={entity._auto_tpi_manager}, " \
            f"SmartPI={entity._prop_algorithm}"
        
        # Reset mock to ignore initial calls during startup/mode change
        mock_recalculate.reset_mock()
        
        # 3. Advance time by SMARTPI_RECALC_INTERVAL_SEC + 1 second
        # We need to fire time change event
        future = dt_util.utcnow() + timedelta(seconds=SMARTPI_RECALC_INTERVAL_SEC + 1)
        async_fire_time_changed(hass, future)
        await hass.async_block_till_done()
        
        # 4. Verify recalculate was called
        assert mock_recalculate.called, "recalculate() should be called by the periodic timer"
        assert mock_recalculate.call_count >= 1
        
        # Reset for next check
        mock_recalculate.reset_mock()
        
        # 5. Advance time again
        future = future + timedelta(seconds=SMARTPI_RECALC_INTERVAL_SEC + 1)
        async_fire_time_changed(hass, future)
        await hass.async_block_till_done()
        
        assert mock_recalculate.called, "recalculate() should be called again"
        
        # 6. Turn OFF and verify timer is stopped
        await entity.async_set_hvac_mode(HVACMode.OFF)
        await hass.async_block_till_done()
        
        assert entity._smartpi_recalc_timer_remove is None
        
        # Reset mock
        mock_recalculate.reset_mock()
        
        # 7. Advance time and verify NO call
        future = future + timedelta(seconds=SMARTPI_RECALC_INTERVAL_SEC + 1)
        async_fire_time_changed(hass, future)
        await hass.async_block_till_done()
        
        assert not mock_recalculate.called, "recalculate() should NOT be called when OFF"

    entity.remove_thermostat()
