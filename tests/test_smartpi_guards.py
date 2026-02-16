"""Test SmartPI Guard Kick and Guard Cut Logic."""
import pytest
from datetime import datetime
from unittest.mock import MagicMock, AsyncMock
from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.smartpi.const import SmartPIPhase, NEAR_BAND_HYSTERESIS_C
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

# --- GUARD KICK TESTS ---

@pytest.mark.asyncio
async def test_smartpi_guard_kick_trigger():
    """Test that Guard Kick triggers when temperature drops below near-band."""

    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.is_device_active = True
    thermostat.target_temperature = 20.0
    thermostat.last_temperature_slope = 0.0
    thermostat.power_manager = None
    thermostat.underlyings = []

    # Create Handler
    handler = SmartPIHandler(thermostat)

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_kick_active = False
    algo._guard_kick_count = 0
    # Guard Cut defaults
    algo.guard_cut_active = False
    algo._near_band_above_deg = 0.5
    
    algo.in_near_band = True
    algo._near_band_below_deg = 0.5
    algo.on_percent = 0.5  # Not maxed out
    
    # Mock calculate to not crash
    algo.calculate = MagicMock()
    algo._last_calculate_time = None

    # Mock process_cycle
    async def fake_process_cycle(timestamp, provider, sender, force):
        # We check the force argument here
        algo.last_force_arg = force
        await provider()
    algo.process_cycle = AsyncMock(side_effect=fake_process_cycle)

    thermostat.prop_algorithm = algo

    # Case 1: Trigger Kick
    # Threshold = 20.0 - 0.5 = 19.5
    # Current Temp = 19.4 (Below threshold)
    thermostat.current_temperature = 19.4
    
    await handler.control_heating(timestamp=datetime.now())

    # Verify Kick Triggered
    assert algo.guard_kick_active is True
    assert algo._guard_kick_count == 1
    assert algo.last_force_arg is True


@pytest.mark.asyncio
async def test_smartpi_guard_kick_reset():
    """Test that Guard Kick resets when temperature recovers."""

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

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_kick_active = True # Already active
    
    # Guard Cut defaults
    algo.guard_cut_active = False
    algo._near_band_above_deg = 0.5
    
    algo._guard_kick_count = 1
    algo.in_near_band = False # Likely false if we are out
    algo._near_band_below_deg = 0.5
    algo.on_percent = 0.5
    
    # Mock calculate
    algo.calculate = MagicMock()
    algo._last_calculate_time = None
    
    # Mock process_cycle
    async def fake_process_cycle(timestamp, provider, sender, force):
        algo.last_force_arg = force
        await provider()
    algo.process_cycle = AsyncMock(side_effect=fake_process_cycle)

    thermostat.prop_algorithm = algo

    # Case 2: Reset
    # Threshold = 19.5
    # Reset Threshold = 19.5 + Hysteresis (0.05) = 19.55
    # Current Temp = 19.6 (Recovered)
    thermostat.current_temperature = 19.6

    await handler.control_heating(timestamp=datetime.now())

    # Verify Reset
    assert algo.guard_kick_active is False
    # Should NOT force cycle on reset (just continues)
    assert algo.last_force_arg is False

@pytest.mark.asyncio
async def test_smartpi_guard_kick_antiloop():
    """Test that Guard Kick does not re-trigger (force) if already active."""

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

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_kick_active = True # Already active from previous kick
    
    # Guard Cut defaults
    algo.guard_cut_active = False
    algo._near_band_above_deg = 0.5
    
    algo._guard_kick_count = 1
    algo._near_band_below_deg = 0.5
    algo.on_percent = 0.5
    
    # Mock process_cycle
    async def fake_process_cycle(timestamp, provider, sender, force):
        algo.last_force_arg = force
        await provider()
    algo.process_cycle = AsyncMock(side_effect=fake_process_cycle)
    algo.calculate = MagicMock()
    algo._last_calculate_time = None

    thermostat.prop_algorithm = algo

    # Case 3: Still Low
    # Threshold = 19.5
    # Current Temp = 19.4 (Still below)
    thermostat.current_temperature = 19.4

    await handler.control_heating(timestamp=datetime.now())

    # Verify status unchanged
    assert algo.guard_kick_active is True
    assert algo._guard_kick_count == 1
    # Should NOT force cycle again (anti-loop)
    assert algo.last_force_arg is False

# --- GUARD CUT TESTS ---

@pytest.mark.asyncio
async def test_smartpi_guard_cut_trigger():
    """Test that Guard Cut triggers when temperature exceeds near-band."""

    # Mock Thermostat
    thermostat = MagicMock()
    thermostat.hass = MagicMock()
    thermostat.name = "TestThermostat"
    thermostat.cycle_min = 10
    thermostat.minimal_activation_delay = 0
    thermostat.minimal_deactivation_delay = 0
    thermostat.vtherm_hvac_mode = VThermHvacMode_HEAT
    thermostat.is_device_active = True
    thermostat.target_temperature = 20.0
    thermostat.last_temperature_slope = 0.0
    thermostat.power_manager = None
    
    # Mock Underlying Entity
    underlying = MagicMock()
    underlying.is_device_active = True
    underlying.turn_off_and_cancel_cycle = AsyncMock()
    thermostat.underlyings = [underlying]
    
    thermostat._async_save = AsyncMock()

    # Create Handler
    handler = SmartPIHandler(thermostat)
    handler._async_save = AsyncMock()

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_cut_active = False
    algo._guard_cut_count = 0
    algo.guard_kick_active = False # Default
    
    algo.in_near_band = True
    algo._near_band_above_deg = 0.5
    algo._near_band_below_deg = 0.5 # Needed for fall-through checks
    algo.on_percent = 0.5
    
    algo.calculate = MagicMock()
    algo._last_calculate_time = None

    thermostat.prop_algorithm = algo

    # Case 4: Trigger Cut
    # Threshold = 20.0 + 0.5 = 20.5
    # Current Temp = 20.6 (Above threshold)
    thermostat.current_temperature = 20.6
    
    await handler.control_heating(timestamp=datetime.now())

    # Verify Cut Triggered
    assert algo.guard_cut_active is True
    assert algo._guard_cut_count == 1
    assert algo._on_percent == 0.0
    
    # Verify actions
    underlying.turn_off_and_cancel_cycle.assert_called_once()
    assert thermostat._on_time_sec == 0
    assert thermostat._off_time_sec == 600 # 10 min * 60
    handler._async_save.assert_called()

@pytest.mark.asyncio
async def test_smartpi_guard_cut_reset():
    """Test that Guard Cut resets when temperature drops back."""

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

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_cut_active = True # Already active
    algo._guard_cut_count = 1
    algo.guard_kick_active = False
    
    algo._near_band_above_deg = 0.5
    algo._near_band_below_deg = 0.5 # Needed for fall-through
    algo.on_percent = 0.5
    
    # Mock calculate
    algo.calculate = MagicMock()
    algo._last_calculate_time = None
    
    thermostat.prop_algorithm = algo

    # Case 5: Reset
    # Threshold = 20.5
    # Reset Threshold = 20.5 - Hysteresis (0.05) = 20.45
    # Current Temp = 20.4 (Recovered)
    thermostat.current_temperature = 20.4

    await handler.control_heating(timestamp=datetime.now())

    # Verify Reset
    assert algo.guard_cut_active is False
    # Should proceed to calculate (we can check if calculate was called)
    algo.calculate.assert_called_once()

@pytest.mark.asyncio
async def test_smartpi_guard_cut_maintain():
    """Test that Guard Cut maintains OFF state if still active."""

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
    handler._async_save = AsyncMock()

    # Mock Algorithm
    algo = MagicMock(spec=SmartPI)
    algo.phase = SmartPIPhase.STABLE
    algo.guard_cut_active = True # Already active
    algo._guard_cut_count = 1
    algo.guard_kick_active = False
    
    algo._near_band_above_deg = 0.5
    algo._near_band_below_deg = 0.5 # Needed for safety
    
    algo.calculate = MagicMock()
    algo._last_calculate_time = None

    thermostat.prop_algorithm = algo

    # Case 6: Still High
    # Threshold = 20.5
    # Current Temp = 20.6 (Still above)
    thermostat.current_temperature = 20.6

    await handler.control_heating(timestamp=datetime.now())

    # Verify status unchanged
    assert algo.guard_cut_active is True
    assert algo._on_percent == 0.0
    
    # Verify still off
    assert thermostat._on_time_sec == 0
    assert thermostat._off_time_sec == 600
    handler._async_save.assert_called()
    
    # Should NOT proceed to calculate
    algo.calculate.assert_not_called()
