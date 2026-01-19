
import pytest
from unittest.mock import MagicMock
import time
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI, VThermHvacMode_HEAT, VThermHvacMode_OFF, LEARNING_PAUSE_RESUME_MIN
from .commons import force_smartpi_stable_mode

class MockVTherm:
    def __init__(self):
        self.name = "test_vtherm"

def test_start_pause_logic():
    """Test that learning is paused after startup/resume."""
    vtherm = MockVTherm()
    pi = SmartPI(vtherm, 1, 1, 1, 1) # dummy inputs
    force_smartpi_stable_mode(pi)
    
    # 1. Initial State (Startup)
    assert pi._last_calculate_time is None
    assert pi._learning_resume_ts is None
    assert pi._startup_grace_period is True
    
    # 2. First calculation (Startup / Reboot) -> SHOULD NOT PAUSE
    now = time.monotonic()
    pi.calculate(
        target_temp=20.0,
        current_temp=19.0, 
        ext_current_temp=10.0,
        slope=0.0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Startup grace period consumed
    assert pi._startup_grace_period is False
    # No pause set
    assert pi._learning_resume_ts is None
    
    # 3. Simulate OFF/Resume cycle
    # Force OFF state reset
    pi.calculate(
         target_temp=20.0,
        current_temp=19.0, 
        ext_current_temp=10.0,
        slope=0.0,
        hvac_mode=VThermHvacMode_OFF # OFF
    )
    assert pi._last_calculate_time is None
    
    # Resume (Second "first run") -> SHOULD PAUSE
    pi.calculate(
        target_temp=20.0,
        current_temp=19.0, 
        ext_current_temp=10.0,
        slope=0.0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Check pause timestamp set
    assert pi._learning_resume_ts is not None
    # Allowed delta for processing time
    expected_resume = now + LEARNING_PAUSE_RESUME_MIN * 60.0
    # Note: 'now' is slightly old but < 1s difference in test execution
    assert abs(pi._learning_resume_ts - expected_resume) < 5.0
    
    # 3. Simulate cycle ongoing (or heartbeat) IMMEDIATELY (should be skipped)
    
    # We need valid timestamps for dt calculation inside update_learning?
    # update_learning takes dt_min.
    
    # Reset counts
    pi.est.learn_skip_count = 0
    
    # Call update_learning
    # Since we are < 20 mins from start (now is close to resume start), it should skip
    pi.update_learning(
        dt_min=10.0,
        current_temp=19.5,
        ext_temp=10.0,
        u_active=0.5,
        setpoint_changed=False
    )
    
    assert pi.est.learn_skip_count == 1
    assert "resume cool-down" in pi.est.learn_last_reason
    
    # 4. Simulate cycle completion AFTER 20 mins
    # We cheat by rewinding _learning_resume_ts instead of sleeping 20 mins
    # Or advancing time.monotonic(). 
    # Let's advance time.monotonic() to be cleaner if we mock it?
    # But calculate() used real time.monotonic().
    # Changing _learning_resume_ts is easier.
    pi._learning_resume_ts = time.monotonic() - 1.0 # Resume time passed
    
    pi.update_learning(
        dt_min=10.0,
        current_temp=19.5,
        ext_temp=10.0,
        u_active=0.5,
        setpoint_changed=False
    )
    
    # Should NOT skip due to resume (might skip due to other reasons like window start)
    # But definitely NOT resume cool-down
    assert "resume cool-down" not in pi.est.learn_last_reason
