"""Test SmartPI specific behaviors: first start and monotonic time."""

import pytest
import time
from unittest.mock import patch, MagicMock
from custom_components.versatile_thermostat.prop_algo_smartpi import SmartPI
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT

def test_smartpi_first_start_no_integral_jump():
    """Test that the first calculation does not cause a massive integral jump.

    If dt_min defaults to cycle_min (e.g. 10) on first start,
    integral += error * 10.
    With the fix (dt_min=0), integral += error * 0 -> 0.
    """
    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_FirstStart"
    )

    # Ensure fresh start
    assert smartpi._last_calculate_time is None
    assert smartpi.integral == 0.0

    # First call with Error = 1.0 (Target 20, Current 19)
    # This should trigger the "first run" logic
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # 1. Verify integral did NOT jump
    # If bug exists: integral = 1.0 * 10 = 10.0
    # If fix works: integral = 1.0 * 0 = 0.0
    assert smartpi.integral == 0.0, f"Integral should be 0 on first start, got {smartpi.integral}"

    # 2. Verify output is produced (P-only)
    # Kp ~ 0.55 (safe). u_pi = 0.55 * 1.0 + Ki * 0 = 0.55.
    assert smartpi.on_percent > 0.0, "Should produce output on first start"
    assert smartpi.on_percent < 1.0, "Should not saturate immediately (unless gain is huge)"

    # 3. Verify timestamp was updated
    assert smartpi._last_calculate_time is not None
    last_time = smartpi._last_calculate_time

    # 4. Second call after 1 minute
    with patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic") as mock_mono:
        # We need to mock monotonic because the algo now uses it.
        # But wait, the previous call used the REAL time.monotonic().
        # So we can't easily mock the 'next' call relative to the 'real' previous call
        # unless we know what the real one was.
        # Better: Mock monotonic for BOTH calls.
        pass

@patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic")
def test_smartpi_monotonic_time_robustness(mock_mono):
    """Test that algo is robust to system clock changes (simulated via monotonic consistency).

    Actually, we want to verify that it uses monotonic, so if we change time.time,
    it shouldn't care. But time.time is not used for dt anymore.
    So we verify that advancing monotonic time correctly advances integration.
    """
    mock_mono.return_value = 1000.0

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_Monotonic"
    )

    # First call (init)
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )
    assert smartpi._last_calculate_time == 1000.0

    # Advance time by 60 seconds
    mock_mono.return_value = 1060.0

    # Second call
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0, # Constant error
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # dt should be 1.0 minute
    # Integral increase = error * dt = 1.0 * 1.0 = 1.0
    assert smartpi.integral == 1.0
    assert smartpi._last_calculate_time == 1060.0

@patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic")
def test_smartpi_first_start_with_mock(mock_mono):
    """Re-test first start with full mocking to be sure."""
    mock_mono.return_value = 5000.0

    smartpi = SmartPI(
        cycle_min=10,
        minimal_activation_delay=0,
        minimal_deactivation_delay=0,
        name="TestSmartPI_FirstStartMock"
    )

    # 1. First Run
    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # Integral should be 0 (dt forced to 0)
    assert smartpi.integral == 0.0
    # Timestamp updated
    assert smartpi._last_calculate_time == 5000.0

    # 2. Second Run (30 sec later)
    # Should trigger the "too short" protection (min 3 sec)
    # But wait, 30 sec > 3 sec.
    mock_mono.return_value = 5030.0

    smartpi.calculate(
        target_temp=20.0,
        current_temp=19.0,
        ext_current_temp=10.0,
        slope=0,
        hvac_mode=VThermHvacMode_HEAT
    )

    # dt = 0.5 min
    # Integral += 1.0 * 0.5 = 0.5
    assert smartpi.integral == 0.5
