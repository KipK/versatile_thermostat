"""Test SmartPI Heartbeat Learning Mechanism."""
import pytest
from unittest.mock import MagicMock, patch
import time
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.smartpi.learning import ABEstimator
from custom_components.versatile_thermostat.smartpi.const import (
    EPISODE_MIN_DURATION_ON_S,
    EPISODE_MIN_DURATION_OFF_S,
)
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_heartbeat_accumulation():
    """Verify that learning window accumulates over multiple calculate calls."""
    smartpi = SmartPI(hass=MagicMock(), cycle_min=10, minimal_activation_delay=0, minimal_deactivation_delay=0, name="TestHB")
    # Simulate deadtimes already learned so the bootstrap gate does not block A/B collection
    smartpi.dt_est.deadtime_heat_reliable = True
    smartpi.dt_est.deadtime_heat_s = 30.0
    smartpi.dt_est.deadtime_cool_reliable = True
    smartpi.dt_est.deadtime_cool_s = 30.0

    # Initial state
    # Pre-warmup to stabilize power output (avoid power instability reset)
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    # We advanced time? No, it used current time.
    # Now reset last_calculate_time for the test sequence
    smartpi._last_calculate_time = time.monotonic() - 60.0
    # Clear the reboot learning freeze: this test doesn't concern reboot behavior
    smartpi.learn_win.set_learning_resume_ts(None)
    # Clear episode start so the deadtime gating does not block the learning window
    smartpi._t_heat_episode_start = None
    smartpi._t_cool_episode_start = None
    
    # 1. Start accumulation
    # Call calculate: should start window (using the power from warmup)
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert smartpi.learn_win_active
    assert smartpi.learn_t_int_s > 0
    first_duration = smartpi.learn_t_int_s
    
    # 2. Add more time (simulation)
    # Patch time to advance 60s
    with patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic") as mock_time:
        mock_time.return_value = smartpi._last_calculate_time + 60.0
        
        smartpi.calculate(
            target_temp=20.0,
            current_temp=19.0, # Keep temp constant to ensure power constant
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
    assert smartpi.learn_t_int_s > first_duration
    assert smartpi.learn_win_active

def test_heartbeat_learning_trigger_on_duration():
    """Verify that learning triggers when duration threshold is met."""
    smartpi = SmartPI(hass=MagicMock(), cycle_min=10, minimal_activation_delay=0, minimal_deactivation_delay=0, name="TestHB_Trigger")
    # Simulate deadtimes already learned so the bootstrap gate does not block A/B collection
    smartpi.dt_est.deadtime_heat_reliable = True
    smartpi.dt_est.deadtime_heat_s = 30.0
    smartpi.dt_est.deadtime_cool_reliable = True
    smartpi.dt_est.deadtime_cool_s = 30.0

    # Cheat: set history manually so robustness checks pass
    smartpi.dt_est._tin_history = [(time.monotonic() - i*60, 20.0 - i*0.01) for i in range(20)]
    
    # Force minimal duration to something small for testing if possible?
    # No, constants are imported. We simulate time passing.
    
    target_dur = EPISODE_MIN_DURATION_ON_S + 60 # 11 mins
    
    # Start window
    smartpi.update_learning(1.0, 19.0, 10.0, 1.0) # 1 min, ON
    assert smartpi.learn_win_active
    
    # Advance time to near completion
    smartpi.update_learning(target_dur / 60.0, 20.0, 10.0, 1.0) # Add remaining time
    
    # Should have triggered learning and reset
    assert not smartpi.learn_win_active
    # Check if learn was called (learn_ok_count increments if successful)
    # We might fail on robustness if data is dummy, but learn_last_reason should indicate attempt
    assert "learned" in smartpi.est.learn_last_reason or "skip" in smartpi.est.learn_last_reason
    
    # If using dummy data, likely "learned a (Median)" or similar if slope consistent
    # Or "skip" if robustness fails.
    # But main point is window reset -> attempt made.

def test_learning_continues_on_setpoint_change():
    """Verify window is NOT reset on setpoint change — it continues.

    Power-transition detection (u_active ≠ u_first) is the real guard;
    a blind reset on setpoint change would discard valid in-flight data.
    """
    smartpi = SmartPI(hass=MagicMock(), cycle_min=10, minimal_activation_delay=0, minimal_deactivation_delay=0, name="TestHB_Reset")
    # Simulate deadtimes already learned so the bootstrap gate does not block A/B collection
    smartpi.dt_est.deadtime_heat_reliable = True
    smartpi.dt_est.deadtime_heat_s = 30.0
    smartpi.dt_est.deadtime_cool_reliable = True
    smartpi.dt_est.deadtime_cool_s = 30.0

    # Start window
    smartpi.update_learning(1.0, 19.0, 10.0, 1.0)
    assert smartpi.learn_win_active

    # Change setpoint: window should continue
    smartpi.update_learning(1.0, 19.0, 10.0, 1.0, setpoint_changed=True)

    assert smartpi.learn_win_active
    assert "setpoint change" not in smartpi.est.learn_last_reason

def test_heartbeat_integration_in_calculate():
    """Verify calculate() calls update_learning()."""
    smartpi = SmartPI(hass=MagicMock(), cycle_min=10, minimal_activation_delay=0, minimal_deactivation_delay=0, name="TestHB_Integration")
    
    # Mock update_learning to verify call
    smartpi.update_learning = MagicMock()
    
    # First run (dt=0) -> No call
    smartpi.calculate(20, 19, 10, 0, VThermHvacMode_HEAT)
    smartpi.update_learning.assert_not_called()
    
    # Second run (dt > 0)
    with patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic") as mock_time:
        mock_time.return_value = smartpi._last_calculate_time + 60.0
        smartpi.calculate(20, 19, 10, 0, VThermHvacMode_HEAT)
        
    smartpi.update_learning.assert_called_once()
