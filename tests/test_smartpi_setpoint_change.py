
import logging
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
    smart_pi.start_new_cycle(u_applied=0.5, temp_in=18.0, temp_ext=5.0)
    
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
    if smart_pi._cycle_start_state:
        smart_pi._cycle_start_state["time"] -= 300.0 # 5 minutes ago
    
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
    
    smart_pi.update_learning(
        current_temp=18.02, # dT = 0.02 (small!)
        ext_current_temp=5.0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # 6. Verify the result
    print(f"Result reason: {smart_pi.learn_last_reason}")
    
    # We verify that with the fix, it is "skip: setpoint change"
    assert smart_pi.learn_last_reason == "skip: setpoint change"
    assert smart_pi.learn_win_active is False
