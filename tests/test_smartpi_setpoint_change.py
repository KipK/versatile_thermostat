
import logging
from datetime import datetime, timedelta
import pytest
from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    VThermHvacMode_HEAT,
)
from unittest.mock import MagicMock

# Set up logging to catch the "extending window" message if needed
logging.basicConfig(level=logging.DEBUG)

def test_smartpi_setpoint_change_aborts_learning():
    """
    Test that a setpoint change during a cycle causes the learning to be skipped
    for that cycle, rather than extending the window invalidly.
    """
    # 1. Initialize SmartPI
    # cycle_min = 10 minutes
    smart_pi = SmartPI(
        hass=MagicMock(),
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestTherm",
        aggressiveness=1.0,  # default
    )
    
    # 2. Start a cycle with setpoint 20.0
    # Initial state: Temp=18, Ext=5
    # We need to mock time or manipulate internal state to simulate time passing
    import time
    now_func = time.monotonic
    
    # Start cycle
    start_ts_dt = datetime.fromtimestamp(now_func() - 600) # Start in the past?
    # Actually the test relied on now - 5*60 later.
    # Let's set it to some "now" value.
    start_ts_dt = datetime.now()
    
    smart_pi._current_cycle_params = {
        "timestamp": start_ts_dt,
        "on_percent": 0.5,
        "temp_in": 18.0,
        "temp_ext": 5.0,
        "hvac_mode": VThermHvacMode_HEAT
    }
    smart_pi._cycle_start_date = start_ts_dt
    
    # smart_pi.start_new_cycle(u_applied=0.5, temp_in=18.0, temp_ext=5.0)
    # We don't have async here? This test is NOT async declared?
    # def test_smartpi_setpoint_change_aborts_learning(): -- It's synchronous!
    # But on_cycle_started is ASYNC.
    # If the test is synchronous, we cannot await.
    # We should convert the test to async.
    # I'll check if I need to update the definition too.
    # Yes, lines 13: def test_smartpi_setpoint_change_aborts_learning():
    
    # I should convert it to async.
    # But first, let's just make the replacements and wrap this test with async mark if possible?
    # Or mock on_cycle_started if it's not crucial?
    # on_cycle_started just sets self.cycle_active = True and logs.
    
    smart_pi.cycle_active = True
    smart_pi._on_percent = 0.5 # Legacy support
    smart_pi._on_time_sec = 0
    smart_pi._off_time_sec = 0
    
    # 3. Call calculate to set the setpoint
    smart_pi.calculate(
        target_temp=20.0,
        current_temp=18.0,
        ext_current_temp=5.0,
        slope=0.0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # 4. Simulate a setpoint change mid-cycle
    # We want to simulate that some time passed, e.g. 5 minutes.
    # We can adjust the start time in _cycle_start_state directly.
    # _cycle_start_state["time"] = now - 5*60
    # 4. Simulate a setpoint change mid-cycle
    # We want to simulate that some time passed, e.g. 5 minutes.
    # We can adjust the start time in _cycle_start_date directly.
    # _cycle_start_state["time"] = now - 5*60
    if smart_pi._cycle_start_date:
        smart_pi._cycle_start_date -= timedelta(minutes=5)
    
    # This call should detect the setpoint change (20.0 -> 21.0)
    smart_pi.calculate(
        target_temp=21.0,
        current_temp=18.01, # Small change
        ext_current_temp=5.0,
        slope=0.1,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # 5. Simulate the end of this cycle immediately (interruption)
    # The cycle was cut short because of the setpoint change.
    
    # 6. Verify the result
    # We need to call on_cycle_completed (Async).
    # Since the test is sync, we might need loop.run_until_complete?
    # Or mock it?
    # But verification depends on logic inside.
    # I will mark the test async in a separate replacement.
    
    pass # Replaced below logic placeholders
    
    import asyncio
    new_params = {
        "temp_in": 18.02,
        "temp_ext": 5.0,
        "timestamp": datetime.now(),
        "hvac_mode": VThermHvacMode_HEAT
    }
    # Run async logic synchronously if needed, or convert test to async.
    # I'll try to execute it if I can't await.
    loop = asyncio.new_event_loop()
    loop.run_until_complete(smart_pi.on_cycle_completed(new_params, smart_pi._current_cycle_params))
    loop.close()
    
    # 6. Verify the result
    print(f"Result reason: {smart_pi.learn_last_reason}")
    
    # We verify that with the fix, it is "skip: setpoint change"
    assert smart_pi.learn_last_reason == "skip: setpoint change"
    assert smart_pi.learn_win_active is False
