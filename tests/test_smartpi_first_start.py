"""Test SmartPI specific behaviors: first start, hysteresis mode, and monotonic time."""

import pytest
import time
from unittest.mock import patch, MagicMock
from custom_components.versatile_thermostat.prop_algo_smartpi import (
    SmartPI,
    SmartPIPhase,
    HYST_UPPER_C,
    HYST_LOWER_C,
    AB_HISTORY_SIZE,
)
from custom_components.versatile_thermostat.vtherm_hvac_mode import VThermHvacMode_HEAT


class TestSmartPIHysteresisPhase:
    """Tests for hysteresis control during learning phase."""

    def test_starts_in_hysteresis_phase(self):
        """Test that SmartPI starts in HYSTERESIS phase with empty measurement history."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_Phase"
        )
        
        assert smartpi.phase == SmartPIPhase.HYSTERESIS
        assert len(smartpi.est.a_meas_hist) == 0
        assert len(smartpi.est.b_meas_hist) == 0

    def test_hysteresis_turns_on_when_cold(self):
        """Test that hysteresis turns ON when temp is below setpoint - HYST_LOWER_C."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_HystOn"
        )
        
        # Temperature is 0.5°C below setpoint (below threshold of -0.3°C)
        smartpi.calculate(
            target_temp=20.0,
            current_temp=19.5,  # 20 - 0.5 < 20 - 0.3 -> should turn ON
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        assert smartpi.on_percent == 1.0, "Should be fully ON when cold"
        assert smartpi._hysteresis_state == "on"

    def test_hysteresis_turns_off_when_hot(self):
        """Test that hysteresis turns OFF when temp is above setpoint + HYST_UPPER_C."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_HystOff"
        )
        
        # First turn ON
        smartpi._on_percent = 1.0
        
        # Temperature is 0.6°C above setpoint (above threshold of +0.5°C)
        smartpi.calculate(
            target_temp=20.0,
            current_temp=20.6,  # 20.6 > 20 + 0.5 -> should turn OFF
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        assert smartpi.on_percent == 0.0, "Should be OFF when hot"
        assert smartpi._hysteresis_state == "off"

    def test_hysteresis_maintains_state_in_band(self):
        """Test that hysteresis maintains previous state when in the band."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_HystBand"
        )
        
        # Start with ON state
        smartpi._on_percent = 1.0
        
        # Temperature is at setpoint (within band: -0.3 < 0 < +0.5)
        smartpi.calculate(
            target_temp=20.0,
            current_temp=20.0,  # Exactly at setpoint -> in band
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        assert smartpi.on_percent == 1.0, "Should maintain ON state in band"
        assert smartpi._hysteresis_state == "band"
        
        # Now test maintaining OFF state
        smartpi._on_percent = 0.0
        smartpi.calculate(
            target_temp=20.0,
            current_temp=20.2,  # Still in band
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        assert smartpi.on_percent == 0.0, "Should maintain OFF state in band"

    def test_no_integral_accumulation_in_hysteresis(self):
        """Test that integral does not accumulate during hysteresis phase."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_NoIntegral"
        )
        
        assert smartpi.integral == 0.0
        
        # Multiple calls in hysteresis mode
        for _ in range(5):
            smartpi.calculate(
                target_temp=20.0,
                current_temp=19.0,
                ext_current_temp=10.0,
                slope=0,
                hvac_mode=VThermHvacMode_HEAT
            )
        
        # Integral should remain 0 (no PI calculation in hysteresis)
        assert smartpi.integral == 0.0, "Integral should not accumulate in hysteresis mode"

    def test_diagnostics_show_regulation_mode(self):
        """Test that diagnostics correctly report regulation_mode."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_Diag"
        )
        
        diag = smartpi.get_diagnostics()
        
        assert diag["phase"] == SmartPIPhase.HYSTERESIS
        assert diag["regulation_mode"] == "hysteresis"
        assert "hysteresis_state" in diag


class TestSmartPIPhaseTransition:
    """Tests for transition from HYSTERESIS to STABLE phase."""

    def test_transition_requires_31_measurements(self):
        """Test that phase transitions to STABLE only after 31 A and B measurements."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_Transition"
        )
        
        # Initially in HYSTERESIS
        assert smartpi.phase == SmartPIPhase.HYSTERESIS
        
        # Add 30 measurements (not enough)
        for i in range(30):
            smartpi.est.a_meas_hist.append(0.01 + i * 0.001)
            smartpi.est.b_meas_hist.append(0.001 + i * 0.0001)
        
        assert smartpi.phase == SmartPIPhase.HYSTERESIS, "Should still be HYSTERESIS with 30 measurements"
        
        # Add the 31st measurement
        smartpi.est.a_meas_hist.append(0.05)
        smartpi.est.b_meas_hist.append(0.005)
        
        assert len(smartpi.est.a_meas_hist) == 31
        assert len(smartpi.est.b_meas_hist) == 31
        assert smartpi.phase == SmartPIPhase.STABLE, "Should transition to STABLE with 31 measurements"

    def test_diagnostics_change_after_transition(self):
        """Test that diagnostics show 'smartpi' mode after transition."""
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_DiagTransition"
        )
        
        # Fill measurements
        for i in range(AB_HISTORY_SIZE):
            smartpi.est.a_meas_hist.append(0.01)
            smartpi.est.b_meas_hist.append(0.001)
        
        diag = smartpi.get_diagnostics()
        
        assert diag["phase"] == SmartPIPhase.STABLE
        assert diag["regulation_mode"] == "smartpi"


@patch("custom_components.versatile_thermostat.prop_algo_smartpi.time.monotonic")
class TestSmartPIMonotonicTime:
    """Tests for monotonic time handling."""

    def test_timestamp_updated_in_hysteresis(self, mock_mono):
        """Test that timestamp is updated even in hysteresis mode."""
        mock_mono.return_value = 1000.0
        
        smartpi = SmartPI(
            hass=MagicMock(),
            cycle_min=10,
            minimal_activation_delay=0,
            minimal_deactivation_delay=0,
            name="TestSmartPI_TimestampHyst"
        )
        
        smartpi.calculate(
            target_temp=20.0,
            current_temp=19.0,
            ext_current_temp=10.0,
            slope=0,
            hvac_mode=VThermHvacMode_HEAT
        )
        
        assert smartpi._last_calculate_time == 1000.0
