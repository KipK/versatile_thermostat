""" Test the TPI integration in BaseThermostat """
import logging
import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from homeassistant.core import HomeAssistant

from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_HEAT, VThermHvacMode_OFF
from custom_components.versatile_thermostat.base_thermostat import BaseThermostat
from custom_components.versatile_thermostat.prop_algorithm import PROPORTIONAL_FUNCTION_TPI
from .commons import *  # pylint: disable=wildcard-import, unused-wildcard-import

_LOGGER = logging.getLogger(__name__)

@pytest.mark.parametrize("expected_lingering_timers", [True])
async def test_auto_tpi_enable_update_config(
    hass: HomeAssistant, skip_hass_states_is_state: None
):
    """Test that auto_tpi_enable_update_config is respected"""
    
    # Register a mock switch to avoid ServiceNotFound
    mock_switch = MockSwitch(hass, "mock_switch", "Mock Switch")
    await register_mock_entity(hass, mock_switch, "switch")

    # Case 1: auto_tpi_enable_update_config is False (default)
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="TheOverSwitchMockName3",
        unique_id="uniqueId3",
        data={
            CONF_NAME: "TheOverSwitchMockName3",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 5,
            CONF_HEATER: "switch.mock_switch",
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_TPI,
            CONF_TPI_COEF_INT: 0.3,
            CONF_TPI_COEF_EXT: 0.01,
            CONF_AUTO_TPI_ENABLE_NOTIFICATION: False,
            # CONF_AUTO_TPI_ENABLE_UPDATE_CONFIG: False # Default
        },
    )

    entity: BaseThermostat = await create_thermostat(
        hass, entry, "climate.theoverswitchmockname3"
    )
    
    # Mock AutoTpiManager to return calculated params
    entity._auto_tpi_manager = MagicMock()
    entity._auto_tpi_manager.async_load_data = AsyncMock()
    entity._auto_tpi_manager.calculate = AsyncMock()
    entity._auto_tpi_manager.update = AsyncMock()
    entity._auto_tpi_manager.get_calculated_params.return_value = {
        CONF_TPI_COEF_INT: 0.5,
        CONF_TPI_COEF_EXT: 0.02
    }
    entity._auto_tpi_manager.learning_active = True
    # Add attributes expected by sensors
    entity._auto_tpi_manager.data_points = 0
    entity._auto_tpi_manager.min_data_points = 10
    entity._auto_tpi_manager.heating_cycles_count = 0
    entity._auto_tpi_manager.learning_quality = "insufficient_data"
    entity._auto_tpi_manager.time_constant = 0.0
    entity._auto_tpi_manager.confidence = 0.0
    
    # Simulate async_added_to_hass
    await entity.async_added_to_hass()
    
    # Verify params NOT updated
    assert entity._tpi_coef_int == 0.3
    assert entity._tpi_coef_ext == 0.01
    
    # Case 2: auto_tpi_enable_update_config is True
    _LOGGER.info("DEBUG: CONF_AUTO_TPI_ENABLE_UPDATE_CONFIG constant value: %s", CONF_AUTO_TPI_ENABLE_UPDATE_CONFIG)
    
    entry2 = MockConfigEntry(
        domain=DOMAIN,
        title="TheOverSwitchMockName4",
        unique_id="uniqueId4",
        data={
            CONF_NAME: "TheOverSwitchMockName4",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 5,
            CONF_HEATER: "switch.mock_switch",
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_TPI,
            CONF_TPI_COEF_INT: 0.3,
            CONF_TPI_COEF_EXT: 0.01,
            CONF_AUTO_TPI_ENABLE_UPDATE_CONFIG: True,
            CONF_AUTO_TPI_ENABLE_NOTIFICATION: False
        },
    )
    _LOGGER.info("DEBUG: entry2.data: %s", entry2.data)

    entity2: BaseThermostat = await create_thermostat(
        hass, entry2, "climate.theoverswitchmockname4"
    )
    
    # Mock AutoTpiManager to return calculated params
    entity2._auto_tpi_manager = MagicMock()
    entity2._auto_tpi_manager.async_load_data = AsyncMock()
    entity2._auto_tpi_manager.calculate = AsyncMock()
    entity2._auto_tpi_manager.update = AsyncMock()
    entity2._auto_tpi_manager.get_calculated_params.return_value = {
        CONF_TPI_COEF_INT: 0.5,
        CONF_TPI_COEF_EXT: 0.02
    }
    entity2._auto_tpi_manager.learning_active = True
    # Add attributes expected by sensors
    entity2._auto_tpi_manager.data_points = 0
    entity2._auto_tpi_manager.min_data_points = 10
    entity2._auto_tpi_manager.heating_cycles_count = 0
    entity2._auto_tpi_manager.learning_quality = "insufficient_data"
    entity2._auto_tpi_manager.time_constant = 0.0
    entity2._auto_tpi_manager.confidence = 0.0
    
    # Simulate async_added_to_hass
    await entity2.async_added_to_hass()
    
    # Verify params UPDATED
    assert entity2._tpi_coef_int == 0.5
    assert entity2._tpi_coef_ext == 0.02
