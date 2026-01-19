"""Test SmartPI Hysteresis Force Logic."""
import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI, SmartPIPhase
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

@pytest.mark.asyncio
async def test_smartpi_hysteresis_forces_cycle():
    """Test that SmartPIHandler forces cycle update when in Hysteresis phase."""
    
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
    # We use a real SmartPI instance or a mock, but a mock is easier to control 'phase'
    algo = MagicMock(spec=SmartPI)
    algo.on_percent = 0.0
    algo.calculate = MagicMock()
    algo._last_calculate_time = None # Needed for dt calculation check inside Handler? No, Handler uses passed timestamp usually, but let's see why it failed.
    # The AttributeError was on the algo mock object.
    
    # Mock process_cycle behavior: it calls the data_provider we pass to it
    async def fake_process_cycle(timestamp, provider, sender, force):
        await provider()
    algo.process_cycle = AsyncMock(side_effect=fake_process_cycle)
    
    # CRITICAL: Set phase to HYSTERESIS
    algo.phase = SmartPIPhase.HYSTERESIS
    
    # helper for update_realized_power
    algo.update_realized_power = MagicMock()
    
    thermostat.prop_algorithm = algo
    
    # --- TEST 1: Normal call (should ideally force, but checking current behavior) ---
    # In current implementation (buggy), force depends on 'force' arg passed to control_heating
    # User validates that we WANT it to force.
    
    # Call control_heating
    await handler.control_heating(timestamp=datetime.now())
    
    # Verify start_cycle call arguments
    # Expected: start_cycle(hvac_mode, on_time, off_time, on_percent, force=???)
    args, kwargs = underlying.start_cycle.call_args
    force_arg = kwargs.get('force') if 'force' in kwargs else args[4]
    
    # Assertion: BEFORE FIX, this might be False (if default force=False is passed)
    # The test is strictly to reproduce/verify the logic we are changing.
    # Note: 'force' is the 5th argument (index 4) in start_cycle signature:
    # async def start_cycle(self, hvac_mode, on_time_sec, off_time_sec, on_percent, force=False)
    
    print(f"Force argument used: {force_arg}")
    
    # Ideally, we want this to be True. If it's False, reproduction successful.
    # Asserting True to demonstrate failure (Red Phase)
    assert force_arg is True, f"Expected force=True in Hysteresis phase, but got {force_arg}"

from datetime import datetime
