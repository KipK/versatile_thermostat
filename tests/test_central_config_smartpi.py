""" Test the central_configuration for SmartPI """
from unittest.mock import patch
from homeassistant.core import HomeAssistant
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.versatile_thermostat.thermostat_switch import (
    ThermostatOverSwitch,
)
from custom_components.versatile_thermostat.vtherm_api import VersatileThermostatAPI
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.const import CONFIG_VERSION, CONFIG_MINOR_VERSION

from .commons import *
from .const import *

import pytest

@pytest.mark.parametrize("expected_lingering_timers", [True])
async def test_smartpi_over_switch_with_central_config(
    hass: HomeAssistant, skip_hass_states_is_state, init_central_power_manager
):
    """Tests that a VTherm with central_configuration for SmartPI inherits parameters correctly."""

    # 1. Create Central Config
    # Note: We manually insert CONF_CYCLE_MIN here as if it was saved by the config flow
    # (since we updated schema, it is allowed)
    central_config_entry = MockConfigEntry(
        domain=DOMAIN,
        title="TheCentralConfigMockName",
        unique_id="centralConfigUniqueId",
        version=CONFIG_VERSION,
        minor_version=CONFIG_MINOR_VERSION,
        data={
            CONF_NAME: CENTRAL_CONFIG_NAME,
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_CENTRAL_CONFIG,
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_central_ext_temp_sensor",
            CONF_TEMP_MIN: 15,
            CONF_TEMP_MAX: 30,
            CONF_STEP_TEMPERATURE: 0.1,
            # Cycle Min in Central Config
            CONF_CYCLE_MIN: 10,
            # TPI params (not used if SmartPI selected but needed for schema?)
            # SmartPI params in Central Config
            CONF_SMART_PI_DEADBAND: 0.2,
            CONF_SMART_PI_AGGRESSIVENESS: 1.5,
            CONF_SMART_PI_USE_SETPOINT_FILTER: False,
        },
    )
    # We need to register the central config
    central_config_entry.add_to_hass(hass)

    # Force update to ensure keys persist through any migration/validation side effects
    entries = hass.config_entries.async_entries(DOMAIN)
    central = entries[0]
    new_data = central.data.copy()
    new_data[CONF_CYCLE_MIN] = 10
    new_data[CONF_SMART_PI_DEADBAND] = 0.2
    new_data[CONF_SMART_PI_AGGRESSIVENESS] = 1.5
    new_data[CONF_SMART_PI_USE_SETPOINT_FILTER] = False
    hass.config_entries.async_update_entry(central, data=new_data)

    # DEBUG: verify API finds it
    api = VersatileThermostatAPI.get_vtherm_api(hass)
    assert api is not None
    # Force finding
    central = api.find_central_configuration()
    assert central is not None, "API did not find central config"

    # 2. Create specific Thermostat using Central Config
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="TheOverSwitchMockName",
        unique_id="uniqueId",
        data={
            CONF_NAME: "TheOverSwitchMockName",
            CONF_THERMOSTAT_TYPE: CONF_THERMOSTAT_SWITCH,
            CONF_TEMP_SENSOR: "sensor.mock_temp_sensor",
            CONF_EXTERNAL_TEMP_SENSOR: "sensor.mock_ext_temp_sensor",
            # Start with LOCAL values that differ from Central
            CONF_CYCLE_MIN: 5,
            CONF_SMART_PI_DEADBAND: 0.05,
            CONF_SMART_PI_AGGRESSIVENESS: 1.0,
            CONF_TEMP_MIN: 8,
            CONF_TEMP_MAX: 18,
            CONF_STEP_TEMPERATURE: 0.3,  # Should be overridden by central (0.1)
            CONF_USE_WINDOW_FEATURE: False,
            CONF_USE_MOTION_FEATURE: False,
            CONF_USE_POWER_FEATURE: False,
            CONF_USE_PRESENCE_FEATURE: False,
            CONF_UNDERLYING_LIST: ["switch.mock_switch"],
            CONF_PROP_FUNCTION: PROPORTIONAL_FUNCTION_SMART_PI,
            CONF_INVERSE_SWITCH: False,
            CONF_MINIMAL_ACTIVATION_DELAY: 30,
            CONF_MINIMAL_DEACTIVATION_DELAY: 0,
            # Request Central Config usage
            CONF_USE_MAIN_CENTRAL_CONFIG: True,
            # We added this key in base_thermostat to trigger cleaning
            CONF_USE_SMART_PI_CENTRAL_CONFIG: True,
            CONF_USE_TPI_CENTRAL_CONFIG: False,
            CONF_USE_WINDOW_CENTRAL_CONFIG: False,
            CONF_USE_MOTION_CENTRAL_CONFIG: False,
            CONF_USE_POWER_CENTRAL_CONFIG: False,
            CONF_USE_PRESENCE_CENTRAL_CONFIG: False,
            CONF_USE_PRESETS_CENTRAL_CONFIG: False,
            CONF_USE_ADVANCED_CENTRAL_CONFIG: False,
        },
    )

    with patch("homeassistant.core.ServiceRegistry.async_call"):
        entity: ThermostatOverSwitch = await create_thermostat(
            hass, entry, "climate.theoverswitchmockname"
        )
        assert entity
        assert entity.name == "TheOverSwitchMockName"
        assert entity.proportional_algorithm is not None
        assert isinstance(entity.proportional_algorithm, SmartPI)

        # VERIFICATION

        # 1. Check Cycle Min (Should be 5 from Local)
        # Note: entity._cycle_min should reflect the config
        assert entity._cycle_min == 5, f"Cycle min should be 5 (Local), got {entity._cycle_min}"

        # 2. Check SmartPI Params (Should be 0.2/1.5 from Central, not 0.05/1.0 from Local)
        algo = entity.proportional_algorithm
        assert algo.deadband_c == 0.2, f"Deadband should be 0.2 (Central), got {algo.deadband_c}"
        assert algo.aggressiveness == 1.5, f"Aggressiveness should be 1.5 (Central), got {algo.aggressiveness}"
        assert algo._use_setpoint_filter is False, "Setpoint filter should be False (Central)"

        # 3. Check Main Params (overridden)
        assert entity.min_temp == 15
        assert entity.max_temp == 30
        assert entity.target_temperature_step == 0.1

    entity.remove_thermostat()
