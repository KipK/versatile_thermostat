
""" Test the AutoPI algorithm """

import logging
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.autopi_algorithm import AutoPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_autopi_instantiation():
    """Test instantiation of AutoPI"""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=30,
        minimal_deactivation_delay=10,
        name="TestAutoPI"
    )
    assert autopi
    assert autopi.Kp == 0.8  # Default
    assert autopi.Ki == 0.05 # Default

def test_autopi_calculation():
    """Test basic calculation"""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Initial state
    autopi.calculate(target_temp=20, current_temp=19, ext_current_temp=10, slope=0, hvac_mode=VThermHvacMode_HEAT)
    
    # Check if calculation produces a value
    assert autopi.on_percent >= 0
    assert autopi.on_percent <= 1
    

    # Verify diagnostics
    diag = autopi.get_diagnostics()
    assert "Kp" in diag
    assert "Ki" in diag


def test_integrator_hold_only_on_heating_start():
    """Test that integrator_hold only triggers when heating starts.
    
    This uses hvac_action to track heating state transitions.
    """
    import time
    from unittest.mock import patch
    
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Simulate enough time passing that integrator_hold should be false
    # since _last_heating_start_ts is 0 (boot time in past)
    with patch('time.time', return_value=time.time() + 300):
        autopi.calculate(
            target_temp=20, 
            current_temp=19, 
            ext_current_temp=10, 
            slope=0, 
            hvac_mode=VThermHvacMode_HEAT,
            hvac_action="idle"
        )
    
    diag = autopi.get_diagnostics()
    # After 300s (> deadtime_s default of 180s), integrator should NOT be held
    assert diag["integrator_hold"] == False, "Integrator should not be held after 300s from heating start"
    
    # Now simulate transition to heating (hvac_action goes from idle to heating)
    current_time = time.time()
    with patch('time.time', return_value=current_time):
        autopi.calculate(
            target_temp=20, 
            current_temp=18,
            ext_current_temp=10, 
            slope=0, 
            hvac_mode=VThermHvacMode_HEAT,
            hvac_action="heating"  # Transition to heating
        )
    
    # Next calculation should have integrator_hold = true (within deadtime)
    with patch('time.time', return_value=current_time + 60):  # 60s later
        autopi.calculate(
            target_temp=20, 
            current_temp=18, 
            ext_current_temp=10, 
            slope=0, 
            hvac_mode=VThermHvacMode_HEAT,
            hvac_action="heating"
        )
    
    diag = autopi.get_diagnostics()
    assert diag["integrator_hold"] == True, "Integrator should be held 60s after heating started"


def test_feedforward_scaling_during_overshoot():
    """Test that feedforward is scaled down during overshoot (temp > setpoint)."""
    autopi = AutoPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestAutoPI"
    )
    
    # Set up a known model state for predictable behavior
    autopi.est.a = 0.02  # Reasonable heating efficiency
    autopi.est.b = 0.001  # Reasonable losses
    
    # Calculate with normal heating (temp below setpoint)
    autopi.calculate(
        target_temp=20, 
        current_temp=19,  # 1°C below setpoint (positive error)
        ext_current_temp=5, 
        slope=0, 
        hvac_mode=VThermHvacMode_HEAT
    )
    
    diag_normal = autopi.get_diagnostics()
    u_ff_normal = diag_normal.get("u_ff", 0)
    
    # Now calculate with overshoot (temp above setpoint)
    autopi.calculate(
        target_temp=20, 
        current_temp=20.5,  # 0.5°C above setpoint (negative error)
        ext_current_temp=5, 
        slope=0, 
        hvac_mode=VThermHvacMode_HEAT
    )
    
    diag_overshoot = autopi.get_diagnostics()
    u_ff_overshoot = diag_overshoot.get("u_ff", 0)
    
    # Feedforward should be reduced during overshoot
    assert u_ff_overshoot < u_ff_normal, f"u_ff during overshoot ({u_ff_overshoot}) should be less than normal ({u_ff_normal})"
    # At -0.5°C error, u_ff should be 0
    assert u_ff_overshoot == 0, f"u_ff should be 0 at -0.5°C overshoot, got {u_ff_overshoot}"
