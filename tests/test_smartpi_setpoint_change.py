
import logging
from datetime import datetime, timedelta
import pytest
from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    VThermHvacMode_HEAT,
)
from .commons import force_smartpi_stable_mode
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
    force_smartpi_stable_mode(smart_pi)
    
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
    
    # Patch time for SmartPI
    from unittest.mock import patch
    
    with patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic") as mock_time:
        mock_time.return_value = 1000.0
        
        # Pre-set last calculate time so dt_min > 0 on first call
        smart_pi._last_calculate_time = 940.0 # 60 sec ago -> dt=1.0 min
        
        # 3. Call calculate to set the intial setpoint
        smart_pi.calculate(
            target_temp=20.0,
            current_temp=18.0,
            ext_current_temp=5.0,
            slope=0.0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        print(f"DEBUG: After Call 1: last_target={smart_pi._last_target_temp}, reason={smart_pi.est.learn_last_reason}, win_active={smart_pi.learn_win_active}")
        
        # Advance time by 5 mins (300s)
        mock_time.return_value = 1300.0
        
        # 4. Simulate a setpoint change mid-cycle
        # This call should detect the setpoint change (20.0 -> 21.0)
        smart_pi.calculate(
            target_temp=21.0,
            current_temp=18.01, # Small change
            ext_current_temp=5.0,
            slope=0.1,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        # Verify the result immediately
        print(f"DEBUG: After Call 2: last_target={smart_pi._last_target_temp}, reason={smart_pi.est.learn_last_reason}, win_active={smart_pi.learn_win_active}")
        
        # calculate calls update_learning, which sets the reason if setpoint changed
        print(f"Result reason: {smart_pi.est.learn_last_reason}")
        
        assert "setpoint change" in smart_pi.est.learn_last_reason
        assert smart_pi.learn_win_active is False
