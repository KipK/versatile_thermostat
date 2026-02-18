"""Test SmartPI Hysteresis Force Logic."""
import pytest
from datetime import datetime
from unittest.mock import MagicMock, AsyncMock, patch
from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.smartpi.const import SmartPIPhase
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT
from custom_components.versatile_thermostat.smartpi.guards import GuardAction

@pytest.mark.asyncio
async def test_smartpi_hysteresis_forces_cycle():
    """Test that SmartPIHandler forces cycle update when in Hysteresis phase.

    The handler logic at line 236 forces cycle when:
        force or (phase == HYSTERESIS and on_percent_changed)

    So on_percent must differ from the initial 0.0 to trigger force=True.
    In real usage, hysteresis toggles between 0% and 100%.
    """

    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.is_device_active = True
    thermostat.current_temperature = 19.0
    thermostat.current_outdoor_temperature = 10.0
    thermostat.target_temperature = 20.0
    thermostat.last_temperature_slope = 0.0
    thermostat.power_manager = None

    # Mock Underlying Entity
    underlying = MagicMock()
    underlying.start_cycle = AsyncMock()
    thermostat.underlyings = [underlying]

    # Create Handler
    handler = SmartPIHandler(thermostat)

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    # Simulate hysteresis toggling to 100% (changed from default 0%)
    algo.on_percent = 1.0
    algo.calculate = MagicMock()
    algo._last_calculate_time = None  # Needed by handler's _data_provider

    # Mock process_cycle behavior: it calls the data_provider we pass to it
    async def fake_process_cycle(timestamp, provider, sender, force):
        await provider()
    algo.process_cycle = AsyncMock(side_effect=fake_process_cycle)

    # CRITICAL: Set phase to HYSTERESIS
    algo.phase = SmartPIPhase.HYSTERESIS
    algo.guard_cut_active = False
    algo.guard_kick_active = False
    algo.in_near_band = False
    algo._near_band_below_deg = 0.5
    algo._near_band_above_deg = 0.5
    algo.guards = MagicMock()
    algo.guards.check_guard_cut.return_value = GuardAction.NONE
    algo.guards.check_guard_kick.return_value = GuardAction.NONE

    # helper for update_realized_power
    algo.update_realized_power = MagicMock()

    thermostat.prop_algorithm = algo

    # Call control_heating - hysteresis toggled from 0% to 100%
    await handler.control_heating(timestamp=datetime.now())

    # Verify start_cycle call arguments
    args, kwargs = underlying.start_cycle.call_args
    force_arg = kwargs.get('force') if 'force' in kwargs else args[4]

    print(f"Force argument used: {force_arg}")

    # In Hysteresis, when on_percent changes (0% -> 100%), force should be True
    assert force_arg is True, f"Expected force=True in Hysteresis phase, but got {force_arg}"
