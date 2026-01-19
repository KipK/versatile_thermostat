"""Tests for Smart-PI Forced Calibration."""
from datetime import datetime
import pytest
from unittest.mock import MagicMock, patch
import time

from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    SmartPIPhase,
    SmartPICalibrationPhase,
    FORCE_CALIBRATION_INTERVAL_HOURS,
    HYST_LOWER_C,
    HYST_LOWER_C,
    HYST_UPPER_C,
)
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_HEAT

class MockHass:
    def __init__(self):
        self.config = MagicMock()
        self.bus = MagicMock()

def create_smartpi():
    hass = MockHass()
    # Basic init params
    algo = SmartPI(
        hass=hass,
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="test_vtherm",
        max_on_percent=1.0,
        deadband_c=0.1,
        aggressiveness=1.0,
        use_setpoint_filter=True,
    )
    # Bypass initial learning by filling history
    # AB_HISTORY_SIZE is 31 in prop_algo_smartpi
    algo.est.a_meas_hist = [1.0] * 35
    algo.est.b_meas_hist = [1.0] * 35
    return algo

def test_manual_trigger():
    algo = create_smartpi()
    assert algo.calibration_state == SmartPICalibrationPhase.IDLE
    
    # Trigger 
    algo.force_calibration()
    assert algo._force_calibration_requested is True
    
    # Run calculate to start state machine
    algo.calculate(
        target_temp=20.0,
        current_temp=20.0, # At setpoint
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.phase == SmartPIPhase.CALIBRATION
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo.on_percent == 0.0
    assert algo._force_calibration_requested is False

def test_auto_trigger_48h():
    algo = create_smartpi()
    # Set last calibration to > 48h ago
    now = time.time()
    algo._last_calibration_time = now - (FORCE_CALIBRATION_INTERVAL_HOURS + 1) * 3600
    
    # Make sure deadtime is reliable (otherwise it triggers due to missing deadtime)
    algo.dt_est.deadtime_heat_reliable = True
    
    # Run calculate
    algo.calculate(
        target_temp=20.0,
        current_temp=20.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo._calibration_retry_count == 1

def test_auto_trigger_missing_deadtime():
    algo = create_smartpi()
    algo._last_calibration_time = time.time() # Recent
    
    # Force deadtimes unreliable
    algo.dt_est.deadtime_heat_reliable = False
    algo.dt_est.deadtime_cool_reliable = True # Only one is missing
    
    algo.calculate(
        target_temp=20.0,
        current_temp=20.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo._calibration_retry_count == 1
    
    # Verify it also triggers if ONLY cool is missing
    algo._calibration_state = SmartPICalibrationPhase.IDLE
    algo._calibration_retry_count = 0
    algo.dt_est.deadtime_heat_reliable = True
    algo.dt_est.deadtime_cool_reliable = False
    
    algo.calculate(
        target_temp=20.0,
        current_temp=20.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN

def test_calibration_cycle_flow():
    algo = create_smartpi()
    algo.force_calibration()
    
    target = 20.0
    
    # 1. Start -> COOL_DOWN
    algo.calculate(target, current_temp=20.0, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo.on_percent == 0.0
    
    # 2. Reach Low Threshold -> HEAT_UP
    # Low is target - 0.3 = 19.7
    low_thresh = target - HYST_LOWER_C - 0.1
    high_thresh = target + HYST_UPPER_C + 0.1
    
    # Transition cycle (COOL_DOWN logic runs, detects threshold, switches state)
    algo.calculate(target, current_temp=low_thresh, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.HEAT_UP
    assert algo.on_percent == 0.0 # Remains 0.0 for this cycle as logic fell through
    
    # Next cycle (HEAT_UP logic runs)
    algo.calculate(target, current_temp=low_thresh, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.on_percent == 1.0
    
    # 3. Reach High Threshold -> COOL_DOWN_FINAL
    # Transition cycle
    algo.calculate(target, current_temp=high_thresh, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN_FINAL
    assert algo.on_percent == 1.0 # Remains 1.0 (from HEAT_UP logic? No, HEAT_UP logic set 1.0 then switched state)
    
    # Next cycle
    algo.calculate(target, current_temp=high_thresh, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.on_percent == 0.0
    
    # 4. Reach Low Threshold again -> IDLE
    # Transition cycle
    algo.calculate(target, current_temp=low_thresh, ext_current_temp=10.0, slope=0, hvac_mode=VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.IDLE
    assert algo._last_calibration_time is not None

def test_attribute_exposure():
    """Verify that attributes are correctly exposed in the handler."""
    algo = create_smartpi()
    now_ts = time.time()
    algo._last_calibration_time = now_ts
    algo._calibration_state = SmartPICalibrationPhase.IDLE
    algo._calibration_retry_count = 2

    # Mock ThermostatProp and Handler interactions
    # We can't easily mock the full integration test here without more setup,
    # but we can verify the properties on the algo instance which the handler reads.
    
    # Actually, let's verify the logic we just added to the handler by instantiating it with a mock thermostat.
    from custom_components.versatile_thermostat.prop_handler_smartpi import SmartPIHandler
    
    mock_thermostat = MagicMock()
    mock_thermostat.prop_algorithm = algo
    mock_thermostat._attr_extra_state_attributes = {
        "specific_states": {},
        "configuration": {}
    }
    mock_thermostat.minimal_activation_delay = 0
    mock_thermostat.minimal_deactivation_delay = 0
    
    handler = SmartPIHandler(mock_thermostat)
    # inject store to avoid errors if needed, though update_attributes doesn't use it
    
    # We need to mock datetime in prop_handler_smartpi to avoid timezone issues or just check logic.
    # The handler imports datetime inside the method? No, at top.
    
    handler.update_attributes()
    
    attrs = mock_thermostat._attr_extra_state_attributes["specific_states"]["smart_pi"]
    assert attrs["calibration_state"] == SmartPICalibrationPhase.IDLE
    assert attrs["calibration_retry_count"] == 2
    # Verify ISO format conversion
    assert attrs["last_calibration_time"] == datetime.fromtimestamp(now_ts).isoformat()

