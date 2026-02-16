"""Test SmartPI Guard Kick and Guard Cut Logic."""
import pytest
from datetime import datetime
from unittest.mock import MagicMock, AsyncMock
from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI, SmartPIPhase
from custom_components.versatile_thermostat.smartpi.guards import GuardAction

@pytest.mark.asyncio
async def test_smartpi_guard_kick_trigger():
    """Test that Guard Kick triggers correctly (Force Cycle)."""
    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.target_temperature = 20.0
    thermostat.underlyings = []
    
    # Create Handler
    handler = SmartPIHandler(thermostat)

    # Mock Algorithm and Guards
    algo = MagicMock(spec=SmartPI)
    algo.guards = MagicMock()
    
    # Case 1: Trigger Kick
    algo.guards.check_guard_kick.return_value = GuardAction.KICK_TRIGGER
    algo.guards.check_guard_cut.return_value = GuardAction.NONE
    algo.on_percent = 0.5

    thermostat.prop_algorithm = algo

    # Execute
    await handler.control_heating(timestamp=datetime.now())

    # Verify
    # Handler should call process_cycle(..., force=True)
    algo.process_cycle.assert_called_once()
    # Check 4th positional argument (force)
    args, _ = algo.process_cycle.call_args
    assert len(args) >= 4
    assert args[3] is True

@pytest.mark.asyncio
async def test_smartpi_guard_kick_reset():
    """Test that Guard Kick resets (informational)."""
    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.target_temperature = 20.0
    thermostat.underlyings = []
    thermostat._async_save = AsyncMock()

    # Create Handler
    handler = SmartPIHandler(thermostat)

    # Mock Algorithm and Guards
    algo = MagicMock(spec=SmartPI)
    algo.guards = MagicMock()
    
    # Case 2: Reset Kick
    algo.guards.check_guard_kick.return_value = GuardAction.KICK_RESET
    algo.guards.check_guard_cut.return_value = GuardAction.NONE
    algo.on_percent = 0.5

    thermostat.prop_algorithm = algo

    # Execute
    await handler.control_heating(timestamp=datetime.now())

    # Verify
    # Should NOT force cycle restart
    algo.process_cycle.assert_called_once()
    args, _ = algo.process_cycle.call_args
    # Check 4th argument if present, or default False
    if len(args) >= 4:
        assert args[3] is False

@pytest.mark.asyncio
async def test_smartpi_guard_kick_antiloop():
    """Test that Guard Kick respects anti-loop (MAINTAIN)."""
    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.target_temperature = 20.0
    thermostat.underlyings = []
    
    handler = SmartPIHandler(thermostat)

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.guards = MagicMock()
    
    # Case 3: Maintain (Anti-loop)
    algo.guards.check_guard_kick.return_value = GuardAction.KICK_MAINTAIN
    algo.guards.check_guard_cut.return_value = GuardAction.NONE
    algo.on_percent = 0.5

    thermostat.prop_algorithm = algo

    # Execute
    await handler.control_heating(timestamp=datetime.now())

    # Verify
    # Should NOT force
    algo.process_cycle.assert_called_once()
    args, _ = algo.process_cycle.call_args
    if len(args) >= 4:
        assert args[3] is False

@pytest.mark.asyncio
async def test_smartpi_guard_cut_trigger():
    """Test that Guard Cut triggers (Force Cycle with 0%)."""
    
    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.target_temperature = 20.0
    
    # Mock Underlying
    under1 = MagicMock()
    under1.is_device_active = True
    under1.turn_off_and_cancel_cycle = AsyncMock()
    under1.start_cycle = AsyncMock()
    thermostat.underlyings = [under1]
    thermostat._async_save = AsyncMock()

    # Create Handler
    handler = SmartPIHandler(thermostat)
    # We need to mock _async_save on handler or algo depending on implementation
    # In Handler: await self._async_save()
    handler._async_save = AsyncMock()

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.guards = MagicMock()
    
    # Case 4: Trigger Cut
    algo.guards.check_guard_cut.return_value = GuardAction.CUT_TRIGGER
    # calculate should be called and implementation would deal with 0%
    algo.calculate = MagicMock()
    # Set on_percent to a float to avoid TypeError in calculate_cycle_times
    algo.on_percent = 0.0

    thermostat.prop_algorithm = algo

    # Execute
    await handler.control_heating(timestamp=datetime.now())

    # Verify
    # 1. Check Guard Cut called
    algo.guards.check_guard_cut.assert_called_once()
    
    # 2. Check Calculate called
    algo.calculate.assert_called_once()

    # 3. Process cycle CALLED with force=True
    algo.process_cycle.assert_called_once()
    args, _ = algo.process_cycle.call_args
    assert len(args) >= 4
    assert args[3] is True

@pytest.mark.asyncio
async def test_smartpi_guard_cut_maintain():
    """Test that Guard Cut maintains OFF state."""
    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.target_temperature = 20.0
    thermostat.underlyings = []
    thermostat._async_save = AsyncMock()

    handler = SmartPIHandler(thermostat)
    handler._async_save = AsyncMock()

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.guards = MagicMock()
    
    # Case 6: Maintain Cut
    algo.guards.check_guard_cut.return_value = GuardAction.CUT_MAINTAIN
    algo.on_percent = 0.0 # Maintain means it was likely 0
    # calculate should be called
    algo.calculate = MagicMock()

    thermostat.prop_algorithm = algo

    # Execute
    await handler.control_heating(timestamp=datetime.now())

    # Verify
    # 1. Check Guard Cut called
    algo.guards.check_guard_cut.assert_called_once()

    # 2. Check Calculate called
    algo.calculate.assert_called_once()
    
    # 3. Process cycle CALLED (but not necessarily forced)
    algo.process_cycle.assert_called_once()
    args, _ = algo.process_cycle.call_args
    if len(args) >= 4:
        assert args[3] is False
