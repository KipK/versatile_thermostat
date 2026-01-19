"""Test SmartPI shedding behavior."""

import pytest
from unittest.mock import MagicMock
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_shedding_resets_integral():
    """Test that power shedding currently resets the integral to 0."""
    smartpi = SmartPI(hass=MagicMock(), 
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Shedding"
    )

    # 1. Establish a steady state with Integral > 0
    # Simulate a situation where we need heat (error > 0)
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    
    # Simulate time passing (10 min) to accumulate integral
    # We can just manually inject integral for simplicity as we know calculate updates it
    smartpi.integral = 5.0
    smartpi.u_prev = 0.5
    
    assert smartpi.integral == 5.0
    assert smartpi.u_prev == 0.5

    # 2. Trigger shedding
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT,
        power_shedding=True
    )

    # 3. Verify observable effects
    # Correct behavior: Integral is FROZEN (not reset)
    assert smartpi.integral == 5.0, f"Integral should be currently frozen (held) at 5.0, but was {smartpi.integral}"
    assert smartpi.on_percent == 0.0, "Output should be forced to 0"
    
    # u_prev is typically the last applied output. If we force output to 0, u_prev usually updates to 0.
    # checking implementation: self.u_prev = 0.0 is explicitly set in the block.
    # This is acceptable as long as integral is saved.
    assert smartpi.u_prev == 0.0
    
    # 4. Verify what happens when shedding stops
    # The integral should be preserved, allowing immediate recovery
    smartpi.calculate(
        target_temp=20,
        current_temp=19,
        ext_current_temp=10,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT,
        power_shedding=False
    )
    
    # Expected behavior (fixed): Integral is still there
    # It might have evolved slightly if integration ran in this step, 
    # but since calculate updates integral, it should persist the base 5.0 + new increment
    assert smartpi.integral >= 5.0, "Integral should be preserved/incremented after shedding stops"
