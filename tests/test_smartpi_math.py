"""Test SmartPI learning logic and frequency."""
import logging
from datetime import timedelta, datetime
from unittest.mock import patch, MagicMock
import time

import pytest
from homeassistant.core import HomeAssistant
from homeassistant.util import dt as dt_util

from custom_components.versatile_thermostat.const import (
    CONF_CYCLE_MIN,
    CONF_PROP_FUNCTION,
    PROPORTIONAL_FUNCTION_SMART_PI,
    CONF_EXTERNAL_TEMP_SENSOR,
    CONF_TEMP_SENSOR,
)
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI

from .commons import create_thermostat, send_temperature_change_event

from pytest_homeassistant_custom_component.common import MockConfigEntry
from custom_components.versatile_thermostat.const import (
    DOMAIN, 
    CONF_NAME, 
    CONF_THERMOSTAT_TYPE
)

@pytest.fixture
async def smartpi_thermostat(hass: HomeAssistant):
    """Create a SmartPI thermostat fixture."""
    entry_data = {
        CONF_NAME: "Test SmartPI",
        CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_SMART_PI,
        CONF_CYCLE_MIN: 5,
        CONF_EXTERNAL_TEMP_SENSOR: "sensor.external_temp",
        CONF_TEMP_SENSOR: "sensor.indoor_temp",
        CONF_THERMOSTAT_TYPE: "thermostat_over_switch",  # Missing in orig dict
        "thermostat_over_switch": "switch.test_heater",
        # Minimal defaults
        "minimal_activation_delay": 0,
        "minimal_deactivation_delay": 0,
    }
    
    mock_entry = MockConfigEntry(
        domain=DOMAIN,
        title="Test SmartPI",
        data=entry_data,
        unique_id="test_smartpi_uid"
    )
    
    # Create the underlying switch
    from homeassistant.components.switch import DOMAIN as SWITCH_DOMAIN
    from tests.commons import MockSwitch, register_mock_entity
    
    mock_switch = MockSwitch(hass, "test_heater", "Test Heater")
    mock_switch.entity_id = "switch.test_heater"
    await register_mock_entity(hass, mock_switch, SWITCH_DOMAIN)
    
    entity = await create_thermostat(hass, mock_entry, "climate.test_smartpi")
    if entity is None:
        # Retry search
        from tests.commons import search_entity
        from homeassistant.components.climate import DOMAIN as CLIMATE_DOMAIN
        for _ in range(10):
            await hass.async_block_till_done()
            entity = search_entity(hass, "climate.test_smartpi", CLIMATE_DOMAIN)
            if entity:
                break
    return entity



async def test_smartpi_math_with_mocked_time(hass: HomeAssistant):
    """Test math with controlled time."""
    with patch('time.time') as mock_time:
        mock_time.return_value = 1000.0
        
        algo = SmartPI(
            cycle_min=10, 
            name="test_pi",
            minimal_activation_delay=0,
            minimal_deactivation_delay=0
        )
        algo.est = MagicMock()
        
        # Start cycle
        # u=0.0 (Heat Loss scenario)
        algo.start_new_cycle(u_applied=0.0, temp_in=20.0, temp_ext=0.0)
        
        # Advance time by 10 mins (600s)
        mock_time.return_value = 1600.0
        
        # End cycle: Temp dropped to 19.0
        # dT = -1.0, dt = 10min -> dT/dt = -0.1
        # delta = 20.0 - 0.0 = 20.0
        algo.update_learning(
            current_temp=19.0,
            ext_current_temp=0.0,
            hvac_mode=1 # HEAT
        )
        
        # Should learn B
        # u < 0.05, delta > 0.5
        algo.est.learn.assert_called_once()
        call_args = algo.est.learn.call_args[1]
        assert call_args['u'] == 0.0
        assert abs(call_args['dT_int_per_min'] - (-0.1)) < 0.001
        
        # Reset
        algo.est.reset_mock()
        
        # Case 2: High power, heating up -> Learn A
        # Setup next cycle manually implies start_new_cycle was called implicitly by update_learning
        # We need to simulate that 'calculate' ran and set u_applied=1.0 for this cycle
        algo._cycle_start_state["u_applied"] = 1.0 # Simulate Calculate setting it
        algo._cycle_start_state["temp_in"] = 19.0
        algo._cycle_start_state["temp_ext"] = 0.0
        algo._cycle_start_state["time"] = 1600.0
        
        # Advance time
        mock_time.return_value = 2200.0 # +10min
        
        # End cycle: Temp rose to 20.0
        # dT = +1.0, dT/dt = 0.1
        algo.update_learning(
            current_temp=20.0,
            ext_current_temp=0.0,
            hvac_mode=1
        )
        
        algo.est.learn.assert_called_once()
        call_args = algo.est.learn.call_args[1]
        assert call_args['u'] == 1.0
        assert abs(call_args['dT_int_per_min'] - 0.1) < 0.001
