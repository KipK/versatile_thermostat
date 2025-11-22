""" Test the TPI integration in BaseThermostat """
import logging
import pytest
from unittest.mock import MagicMock, patch
from homeassistant.core import HomeAssistant

from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_HEAT, VThermHvacMode_OFF
from custom_components.versatile_thermostat.base_thermostat import BaseThermostat
from custom_components.versatile_thermostat.prop_algorithm import PROPORTIONAL_FUNCTION_TPI
from .commons import *  # pylint: disable=wildcard-import, unused-wildcard-import

_LOGGER = logging.getLogger(__name__)

@pytest.mark.parametrize("expected_lingering_tasks", [True])
@pytest.mark.parametrize("expected_lingering_timers", [True])
async def test_tpi_integration_timer_start_stop(
    hass: HomeAssistant, skip_hass_states_is_state: None
):
    """Test that TPI timer starts and stops correctly with mode changes"""

    entry = MockConfigEntry(
        domain=DOMAIN,
        title="TheOverSwitchMockName",
        unique_id="uniqueId",
        data={
            CONF_NAME: "TheOverSwitchMockName",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 5,
            CONF_TEMP_MIN: 15,
            CONF_TEMP_MAX: 30,
            CONF_HEATER: "switch.mock_switch",
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_TPI,
            CONF_TPI_COEF_INT: 0.3,
            CONF_TPI_COEF_EXT: 0.01,
        },
    )

    # 1. Create Thermostat
    entity: BaseThermostat = await create_thermostat(
        hass, entry, "climate.theoverswitchmockname"
    )
    assert entity
    
    # 2. Mock AutoTpiManager
    # We need an AsyncMock for async methods
    entity._auto_tpi_manager = MagicMock()
    entity._auto_tpi_manager.learning_active = True # Simulate active learning
    # Mock async methods to return a coroutine
    async def async_mock(*args, **kwargs):
        pass
    entity._auto_tpi_manager.on_thermostat_mode_changed = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.on_cycle_elapsed = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.update = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.calculate = MagicMock(side_effect=async_mock)
    
    # 3. Switch to HEAT -> Should start timer
    with patch("custom_components.versatile_thermostat.base_thermostat.async_call_later") as mock_call_later:
        _LOGGER.info("Setting HVAC Mode to HEAT")
        await entity.async_set_hvac_mode(VThermHvacMode_HEAT)
        
        # Verify timer started
        mock_call_later.assert_called()
        args, _ = mock_call_later.call_args
        assert args[1] == entity._cycle_min * 60
        assert args[2] == entity._on_tpi_cycle_elapsed
        
        # Verify AutoTpiManager notified
        entity._auto_tpi_manager.on_thermostat_mode_changed.assert_called_with("HEAT", entity.target_temperature)

    # 4. Switch to OFF -> Should stop timer
    # Mock the timer cancel function
    cancel_mock = MagicMock()
    entity._cycle_timer = cancel_mock
    
    with patch("custom_components.versatile_thermostat.base_thermostat.async_call_later") as mock_call_later:
        _LOGGER.info("Setting HVAC Mode to OFF")
        await entity.async_set_hvac_mode(VThermHvacMode_OFF)
        
        # Verify timer cancelled
        cancel_mock.assert_called_once()
        assert entity._cycle_timer is None
        
        # Verify AutoTpiManager notified
        entity._auto_tpi_manager.on_thermostat_mode_changed.assert_called_with("off", entity.target_temperature)
        
        # Verify timer NOT restarted
        mock_call_later.assert_not_called()

@pytest.mark.parametrize("expected_lingering_tasks", [True])
@pytest.mark.parametrize("expected_lingering_timers", [True])
async def test_tpi_integration_cycle_elapsed(
    hass: HomeAssistant, skip_hass_states_is_state: None
):
    """Test that TPI cycle elapsed restarts timer if in HEAT"""
    
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="TheOverSwitchMockName2",
        unique_id="uniqueId2",
        data={
            CONF_NAME: "TheOverSwitchMockName2",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            CONF_CYCLE_MIN: 5,
            CONF_HEATER: "switch.mock_switch",
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_TPI,
            CONF_TPI_COEF_INT: 0.3,
            CONF_TPI_COEF_EXT: 0.01,
        },
    )

    entity: BaseThermostat = await create_thermostat(
        hass, entry, "climate.theoverswitchmockname2"
    )
    
    entity._auto_tpi_manager = MagicMock()
    entity._auto_tpi_manager.learning_active = True
    # Mock async methods to return a coroutine
    async def async_mock(*args, **kwargs):
        pass
    entity._auto_tpi_manager.on_thermostat_mode_changed = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.on_cycle_elapsed = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.update = MagicMock(side_effect=async_mock)
    entity._auto_tpi_manager.calculate = MagicMock(side_effect=async_mock)
    
    # Set HEAT mode
    entity._state_manager.current_state.set_hvac_mode(VThermHvacMode_HEAT)
    
    # Mock restart
    with patch("custom_components.versatile_thermostat.base_thermostat.async_call_later") as mock_call_later:
        # Simulate timer elapsed
        await entity._on_tpi_cycle_elapsed(None)
        
        # Verify AutoTpiManager notified of cycle end
        entity._auto_tpi_manager.on_cycle_elapsed.assert_called()
        
        # Verify timer restarted
        mock_call_later.assert_called()