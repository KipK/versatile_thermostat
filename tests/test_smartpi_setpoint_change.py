
import logging
from datetime import datetime, timedelta
import pytest
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT
from .commons import force_smartpi_stable_mode
from unittest.mock import MagicMock

# Set up logging to catch the "extending window" message if needed
logging.basicConfig(level=logging.DEBUG)

def test_smartpi_setpoint_change_continues_learning():
    """
    Test that a setpoint change during an active learning window does NOT abort it.
    The window continues; power-transition detection (u_active ≠ u_first) is the
    real guard and will close the window if heating power actually changes.
    """
    smart_pi = SmartPI(
        hass=MagicMock(),
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestTherm",
    )
    force_smartpi_stable_mode(smart_pi)

    # Open a window via update_learning directly (t_heat_episode_start=None → deadtime bypassed)
    smart_pi.update_learning(1.0, 18.0, 5.0, 1.0)  # ON phase, 1 min
    assert smart_pi.learn_win_active

    # Setpoint change mid-window: window should continue
    smart_pi.update_learning(1.0, 18.0, 5.0, 1.0, setpoint_changed=True)
    assert smart_pi.learn_win_active
    assert "setpoint change" not in smart_pi.est.learn_last_reason

    # Power transition (heater off) is what actually closes the window
    smart_pi.update_learning(1.0, 18.0, 5.0, 0.0)  # OFF phase → u_active ≠ u_first
    assert not smart_pi.learn_win_active
