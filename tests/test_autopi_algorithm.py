
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

