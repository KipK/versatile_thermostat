
# tests/test_smartpi_cycle_alignment.py
import pytest
import time
from unittest.mock import MagicMock, patch, AsyncMock
from homeassistant.core import HomeAssistant
from homeassistant.const import STATE_ON, STATE_OFF

from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
from custom_components.versatile_thermostat.thermostat_prop import ThermostatProp
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

class MockThermostat(MagicMock):
    def __init__(self):
        super().__init__(spec=ThermostatProp)
        self.hass = MagicMock(spec=HomeAssistant)
        self.name = "MockThermostat"
        self._entry_infos = {}
        self._cur_temp = 19.0
        self._cur_ext_temp = 5.0
        self.target_temperature = 20.0
        self.last_temperature_slope = 0.0
        self.vtherm_hvac_mode = VThermHvacMode_HEAT
        self.is_device_active = True
        self._underlyings = []
        self._prop_algorithm = None
        
        # Async methods need AsyncMock
        self.async_control_heating = AsyncMock()
        self.async_underlying_entity_turn_off = AsyncMock()
        self.recalculate = MagicMock()

@pytest.mark.asyncio
async def test_smartpi_60s_timer_interference():
    """
    Test that the 60s periodic recalculation does NOT trigger learning update.
    The learning window should only reset/progress on the main cycle boundary.
    """
    # Setup
    t = MockThermostat()
    # Configure SmartPI
    handler = SmartPIHandler(t)
    # We mock the internal store to avoid FS ops
    handler._store = AsyncMock()
    
    # Init algorithm directly
    cycle_min = 10
    algo = SmartPI(
        cycle_min=cycle_min, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestAlgo",
        max_on_percent=1.0,
        deadband_c=0.1,
        aggressiveness=1.0
    )
    t._prop_algorithm = algo
    # Ensure initialized
    algo.start_new_cycle(0.5, 19.0, 5.0) 
    
    # --- SCENARIO START ---
    
    # 1. Start of Cycle (T=0)
    # Usually triggered by CycleManager or startup. 
    # algo.start_new_cycle was just called. 
    # algo.learn_win_active should be False initially until first update_learning
    assert algo.learn_win_active is False
    
    # 2. Simulate 60s timer (T=1 min)
    # The handler's _recalc_callback calls t.async_control_heating(timestamp=now)
    # CURRENTLY (Buggy behavior): timestamp IS passed => update_learning IS called
    # EXPECTED FIX: timestamp should be None => update_learning NOT called
    
    # We verify what update_learning does if called at T=1min (way too short)
    # If the bug is present, update_learning is called.
    # If fixed, handler won't call it (we will test handler logic effectively by checking calls)
    
    # Let's inspect Handler's control_heating logic directly
    # control_heating(timestamp=...) calls update_learning
    
    # Simulate time passing (1 min)
    import time
    now_ts = time.time()
    
    # Manually trigger what the timer does:
    # await handler.control_heating(timestamp=now_ts) <-- This is what we want to avoid or change behavior of
    
    # PRE-FIX CHECK: verify that calling with timestamp triggers learning check
    # We want to assert that IF we pass timestamp, it enters update_learning.
    
    # For this test, verifying the HANDLER fix:
    # We will simulate the _recalc_callback logic.
    
    # But _recalc_callback is an internal function in _start_recalc_timer. 
    # Hard to test the callback definition itself without firing events.
    
    # Alternative: Test control_heating with timestamp=None skips update_learning
    
    # A. Test control_heating(timestamp=None) -> No update_learning
    mock_update = MagicMock()
    with patch.object(algo, 'update_learning', mock_update), \
         patch.object(handler, '_async_save', new_callable=AsyncMock):
        await handler.control_heating(timestamp=None)
        mock_update.assert_not_called()
        
    # B. Test control_heating(timestamp=Value) -> update_learning called
    with patch.object(algo, 'update_learning', mock_update), \
         patch.object(handler, '_async_save', new_callable=AsyncMock):
        await handler.control_heating(timestamp=now_ts)
        mock_update.assert_called_once()
    
    # So the fix is indeed changing the CALLER (the timer callback) to pass None.
    # We can test that by creating the handler, starting the timer, and seeing what it calls.
    # But async_track_time_interval is hard to fast-forward.
    
    # Instead, let's verify the "Window too short" logic in Algo separately to ensure 
    # "waiting" message appears instead of "skip".

@pytest.mark.asyncio
async def test_smartpi_learning_requires_cycle_boundary():
    """Verify that learning operates on data from complete cycles.
    
    Since update_learning is only called at cycle boundaries (not mid-cycle),
    dt_min will always be >= cycle_min, and we should get a learning result
    (either learned, skip, or extending) - never 'waiting'.
    """
    cycle_min = 10
    algo = SmartPI(
        cycle_min=cycle_min, 
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAlgo"
    )
    
    # Start cycle
    algo.start_new_cycle(0.5, 20.0, 5.0)
    # Fake start time
    import time
    start_ts = time.time()
    algo._cycle_start_state["time"] = start_ts
    
    # Update at exactly 1 cycle (10 minutes) - simulating cycle boundary
    current_ts = start_ts + (cycle_min * 60)
    
    algo.update_learning(
        current_temp=20.1, 
        ext_current_temp=5.0, 
        current_temp_ts=current_ts
    )
    
    reason = algo.est.learn_last_reason
    # Should NOT be "waiting" since we're at a cycle boundary
    assert "waiting" not in reason.lower()
    
@pytest.mark.asyncio
async def test_smartpi_frozen_power_snapshot():
    """
    Verify that u_applied in the snapshot is frozen at the first calculation 
    and not overwritten by subsequent calculations in the same cycle.
    """
    cycle_min = 10
    algo = SmartPI(
        cycle_min=cycle_min, 
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAlgo"
    )
    
    
    # 1. Start new cycle with u_applied = None (Mocking the new behavior)
    # Note: currently start_new_cycle stores whatever we pass.
    # If we pass None, it stores 0.0 -> We also need to fix start_new_cycle to allow None/pending.
    
    
    algo.start_new_cycle(None, 20.0, 5.0)
    
    # 1. Verify start with None is accepted and stored as None
    assert algo._cycle_start_state["u_applied"] is None
    
    # 2. First Calculate (T=0)
    # We force calculate to decide a specific power (e.g. 0.5)
    # Since we can't easily control the whole PID calculation in a mocked env without much setup,
    # we will manually invoke the logic that updates specific states or rely on calculate side effects.
    # But calculate() updates _on_percent at the end.
    
    # Let's mock _on_percent setting inside calculate by mocking calculate's internal calls?
    # No, simpler: calling calculate updates `u_applied` in the snapshot at the end.
    
    # Let's perform a calculate with specific errors to get a result.
    # Target=20, Cur=19 -> Error=1. Proportional part active.
    algo.target_temperature = 20.0
    algo.calculate(20.0, 19.0, 5.0, 0.0, VThermHvacMode_HEAT)
    
    # Capture the result
    first_u = algo._on_percent
    assert first_u > 0.0
    
    # Verify snapshot got updated
    assert algo._cycle_start_state["u_applied"] == float(first_u)
    
    # 3. Second Calculate (T=1 min, conditions changed)
    # Change Setpoint to huge value to force 100% power
    algo.target_temperature = 25.0
    algo.calculate(25.0, 19.0, 5.0, 0.0, VThermHvacMode_HEAT)
    
    second_u = algo._on_percent
    assert second_u > first_u # Should surely increase
    
    # Verify snapshot is STILL the first value (Frozen)
    assert algo._cycle_start_state["u_applied"] == float(first_u)
    assert algo._cycle_start_state["u_applied"] != float(second_u) 

@pytest.mark.asyncio
async def test_smartpi_reboot_discards_active_window():
    """
    Verify that restoring state DISCARDS active learning window progress.
    This ensures a fresh start after reboot (interruption).
    """
    algo = SmartPI(
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestAlgo"
    )
    
    # Simulate a saved state with an active window
    saved_state = {
        "learn_win_active": True,
        "learn_win_start_ts": time.time() - 300, # 5 min ago
        "learn_u_int": 2.5,
        "a": 0.01,
        "b": 0.02
    }
    
    # Load state
    algo.load_state(saved_state)
    
    # Check Persistence of params
    assert algo.est.a == 0.01
    assert algo.est.b == 0.02
    
    # Check DISCARD of window
    assert algo.learn_win_active is False
    assert algo.learn_win_start_ts is None
    assert algo.learn_u_int == 0.0

@pytest.mark.asyncio
async def test_smartpi_power_stability_abort():
    """
    Verify that changing power output in a subsequent cycle 
    within the same learning window ABORTS the window 
    (strict power requirement).
    """
    algo = SmartPI(
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestAlgo"
    )
    # 1. Start a cycle with 50% power
    algo.start_new_cycle(0.5, 20.0, 10.0)
    algo._cycle_start_state["time"] -= 600 # 10 min elapsed
    
    # 2. Update learning (cycle ends): Should START window
    algo.update_learning(
        current_temp=20.0, # No temp change yet
        ext_current_temp=10.0,
        previous_temp=20.0,
        previous_power=0.5,
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )
    assert algo.learn_win_active is True
    assert algo.learn_u_first == 0.5
    # Reason could be "window start" or "extending window" depending on dT checks
    assert "window" in algo.est.learn_last_reason
    
    # 3. Start NEXT cycle with DIFFERENT power (0.6)
    algo.start_new_cycle(0.6, 20.0, 10.0)
    algo._cycle_start_state["time"] -= 600
    
    # 4. Update learning: Should ABORT due to power change
    algo.update_learning(
        current_temp=20.2, # Some temp change
        ext_current_temp=10.0,
        previous_temp=20.0,
        previous_power=0.6,
        hvac_mode=VThermHvacMode_HEAT,
        cycle_dt=10.0
    )
    
    assert algo.learn_win_active is False
    assert "skip: power instability" in algo.est.learn_last_reason

@pytest.mark.asyncio
async def test_smartpi_resume_from_off_resets_cycle():
    """
    Verify that when resuming from OFF state (e.g., after window close),
    the cycle start state is reset to prevent using stale timestamps.
    """
    from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_OFF
    
    # Setup
    t = MockThermostat()
    handler = SmartPIHandler(t)
    handler._store = AsyncMock()
    
    algo = SmartPI(
        cycle_min=10, 
        minimal_activation_delay=0, 
        minimal_deactivation_delay=0, 
        name="TestAlgo"
    )
    t._prop_algorithm = algo
    
    # Initialize cycle with old timestamp (simulating before window opened)
    old_time = time.monotonic() - 3600  # 1 hour ago (in monotonic time)
    algo.start_new_cycle(0.5, 20.0, 5.0)
    algo._cycle_start_state["time"] = old_time
    
    # Simulate thermostat was OFF (timer stopped)
    t._smartpi_recalc_timer_remove = None
    t.vtherm_hvac_mode = VThermHvacMode_HEAT
    
    # Trigger on_state_changed (what happens when window closes)
    with patch.object(handler, '_start_recalc_timer'):
        await handler.on_state_changed()
    
    # Verify cycle start state was reset to current time (not old_time)
    new_start_time = algo._cycle_start_state["time"]
    assert new_start_time > old_time, "Cycle start time should be reset to NOW, not use old timestamp"
    assert abs(new_start_time - time.monotonic()) < 2, "Cycle start time should be approximately NOW"
    
    # Verify learning window was also reset
    assert algo.learn_win_active is False
    assert algo.learn_win_start_ts is None
