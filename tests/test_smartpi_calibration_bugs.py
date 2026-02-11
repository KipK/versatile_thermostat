"""Verification tests for Smart-PI Calibration Fixes."""
import pytest
import time
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    SmartPIPhase,
    SmartPICalibrationPhase,
    HYST_LOWER_C,
    CALIBRATION_TIMEOUT_MIN,
)
from custom_components.versatile_thermostat.vtherm_hvac_mode import (
    VThermHvacMode_HEAT,
    VThermHvacMode_COOL,
)

class MockHass:
    def __init__(self):
        self.config = MagicMock()
        self.bus = MagicMock()

def create_smartpi():
    hass = MockHass()
    algo = SmartPI(
        hass=hass,
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="test_vtherm",
        max_on_percent=1.0,
        deadband_c=0.1,
        use_setpoint_filter=False,
    )
    # Ensure auto-calibration doesn't trigger immediately
    algo.dt_est.deadtime_heat_reliable = True
    algo.dt_est.deadtime_cool_reliable = True
    algo.dt_est.deadtime_heat_s = 60.0
    algo.dt_est.deadtime_cool_s = 60.0
    algo._last_calibration_time = time.time()
    return algo

def test_diagnostics_updated_during_calibration():
    """Verify that diagnostics (u_cmd, u_pi) are correctly updated during calibration."""
    algo = create_smartpi()
    # 1. Force STABLE phase
    algo.est.a_meas_hist = [1.0] * 40
    algo.est.b_meas_hist = [1.0] * 40
    algo.est.a_hist = [1.0] * 40
    algo.est.b_hist = [1.0] * 40
    
    # 2. Run PI loop to get non-zero values
    algo.calculate(25.0, 20.0, 10.0, 0, VThermHvacMode_HEAT)
    time.sleep(0.01)
    algo.calculate(25.0, 20.0, 10.0, 0, VThermHvacMode_HEAT)
    
    assert algo.u_pi > 0
    assert algo.u_cmd > 0
    
    # 3. Trigger calibration
    algo.force_calibration()
    
    # 4. Run calculation (stays in COOL_DOWN if temp is high)
    algo.calculate(25.0, 25.0, 10.0, 0, VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo.on_percent == 0.0
    
    # VERIFY: Diagnostics are now updated to 0.0 instead of being stale
    assert algo.u_pi == 0.0
    assert algo.u_cmd == 0.0
    assert algo.u_limited == 0.0
    assert algo.u_applied == 0.0

def test_calibration_trigger_allowed_in_hysteresis():
    """Verify that calibration triggering now works in HYSTERESIS phase."""
    algo = create_smartpi()
    # Ensure it stays in HYSTERESIS
    algo.est.a_meas_hist = []
    algo.est.b_meas_hist = []
    assert algo.phase == SmartPIPhase.HYSTERESIS
    
    # Trigger calibration
    algo.force_calibration()
    
    # Run calculation
    algo.calculate(25.0, 25.0, 10.0, 0, VThermHvacMode_HEAT)
    
    # VERIFY: It successfully entered CALIBRATION phase even from HYSTERESIS
    assert algo.phase == SmartPIPhase.CALIBRATION
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN

def test_calibration_cool_mode():
    """Verify forced calibration behavior in COOL mode (inverted logic)."""
    algo = create_smartpi()
    algo.force_calibration()
    
    # 1. COOL_DOWN in COOL mode -> Effort should be 1.0 (cooling)
    algo.calculate(20.0, 22.0, 30.0, 0, VThermHvacMode_COOL)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    assert algo.on_percent == 1.0 # Cooling effort
    
    # 2. Reach Low threshold -> HEAT_UP
    # target=20.0. Low threshold = target - 0.3 = 19.7
    algo.calculate(20.0, 19.0, 30.0, 0, VThermHvacMode_COOL)
    assert algo.calibration_state == SmartPICalibrationPhase.HEAT_UP
    assert algo.on_percent == 0.0 # Stop cooling to let it heat up (FIXED: immediate update)
    
    # 3. Reach High threshold -> COOL_DOWN_FINAL
    # High threshold = target + 0.5 = 20.5
    algo.calculate(20.0, 21.0, 30.0, 0, VThermHvacMode_COOL)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN_FINAL
    assert algo.on_percent == 1.0 # Cool down again (FIXED: immediate update)

def test_calibration_timeout():
    """Verify that calibration times out after 4 hours."""
    algo = create_smartpi()
    algo.force_calibration()
    
    # Start calibration
    algo.calculate(25.0, 25.0, 10.0, 0, VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.COOL_DOWN
    
    # Manually backdate the start time by 4h + 1min (set on both algo and manager)
    backdated_time = time.monotonic() - (CALIBRATION_TIMEOUT_MIN * 60 + 60)
    algo._calibration_start_time = backdated_time
    algo.calibration_mgr._calibration_start_time = backdated_time
    
    # Next calculate should abort
    algo.calculate(25.0, 25.0, 10.0, 0, VThermHvacMode_HEAT)
    assert algo.calibration_state == SmartPICalibrationPhase.IDLE
    assert algo._calibration_start_time is None
