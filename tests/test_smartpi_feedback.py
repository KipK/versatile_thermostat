
import pytest
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI

def test_smartpi_update_realized_power():
    """Test the update_realized_power method of SmartPI."""
    # Create a barebone SmartPI instance (lots of arguments mocked/dummy)
    # We only care about update_realized_power which touches very few things
    
    # We need to mock hass
    hass = MagicMock()
    
    # Init SmartPI
    algo = SmartPI(
        hass=hass,
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAlgo",
        max_on_percent=1.0,
        deadband_c=0.0
    )
    
    # Pre-set some state
    algo._last_u_applied = 0.5
    algo.Ki = 0.01  # Ensure Ki > KI_MIN so tracking logic runs
    algo._last_i_mode = "I:RUN"
    algo._in_deadband = False
    
    # 1. Test standard update
    algo.update_realized_power(realized_percent=0.8, forced_by_timing=False, dt_min=5.0)
    
    assert algo._last_u_applied == 0.8
    assert algo._last_forced_by_timing is False
    assert algo.u_prev == 0.8
    
    # 2. Test forced by timing
    algo.update_realized_power(realized_percent=0.0, forced_by_timing=True, dt_min=5.0)
    
    assert algo._last_u_applied == 0.0
    assert algo._last_forced_by_timing is True
    # When forced by timing, tracking error should be skipped (du=0.0)
    assert algo._last_aw_du == 0.0
    assert algo.u_prev == 0.0
    
    # 3. Test clamping tracking
    # Setup conditions where tracking would happen
    algo._last_u_cmd = 0.9
    algo._max_on_percent = 0.8
    algo._last_u_limited = 0.8 # Clamped
    
    # Simulating a case where output was clamped to 0.8
    # realized_percent = 0.8
    algo.update_realized_power(realized_percent=0.8, forced_by_timing=False, dt_min=1.0)
    
    # du calculation in code:
    # u_aw_ref = u_limited (0.8)
    # if u_cmd (0.9) > max_on (0.8): u_aw_ref = u_cmd (0.9)
    # du = u_applied (0.8) - u_aw_ref (0.9) = -0.1
    
    assert abs(algo._last_aw_du - (-0.1)) < 1e-9

