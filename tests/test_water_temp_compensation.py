# pylint: disable=line-too-long, protected-access
"""Tests for water temperature compensation utility."""

import pytest
from unittest.mock import MagicMock

from custom_components.versatile_thermostat.water_temp_compensation import (
    WaterTempCompensation,
    DEFAULT_REFERENCE_TEMP,
    MIN_FACTOR,
    MAX_FACTOR,
    EMA_ALPHA,
)


@pytest.fixture
def mock_hass():
    """Create a mock Home Assistant instance."""
    return MagicMock()


@pytest.fixture
def mock_sensor_state():
    """Create a mock sensor state."""
    state = MagicMock()
    state.state = "50.0"
    return state


class TestWaterTempCompensationDisabled:
    """Tests for when compensation is disabled (no sensor)."""

    def test_disabled_without_sensor(self, mock_hass):
        """Compensation should be disabled when no sensor is configured."""
        comp = WaterTempCompensation(mock_hass, None)
        assert comp.is_enabled is False
        assert comp.sensor_id is None

    def test_factor_is_one_when_disabled(self, mock_hass):
        """Factor should be 1.0 when compensation is disabled."""
        comp = WaterTempCompensation(mock_hass, None)
        assert comp.get_compensation_factor() == 1.0

    def test_compensate_passthrough_when_disabled(self, mock_hass):
        """compensate_power should pass through value when disabled."""
        comp = WaterTempCompensation(mock_hass, None)
        assert comp.compensate_power(50.0) == 50.0
        assert comp.compensate_power(100.0) == 100.0


class TestWaterTempCompensationSensorState:
    """Tests for sensor state handling."""

    def test_sensor_unavailable(self, mock_hass):
        """Should handle unavailable sensor gracefully."""
        mock_hass.states.get.return_value = None
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.is_enabled is False
        assert comp.current_water_temp is None

    def test_sensor_unknown_state(self, mock_hass):
        """Should handle unknown state gracefully."""
        state = MagicMock()
        state.state = "unknown"
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.is_enabled is False

    def test_sensor_unavailable_state(self, mock_hass):
        """Should handle unavailable state gracefully."""
        state = MagicMock()
        state.state = "unavailable"
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.is_enabled is False

    def test_sensor_valid_state(self, mock_hass, mock_sensor_state):
        """Should read valid sensor state."""
        mock_hass.states.get.return_value = mock_sensor_state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.is_enabled is True
        assert comp.current_water_temp == 50.0

    def test_sensor_out_of_range_low(self, mock_hass):
        """Should reject water temp below valid range."""
        state = MagicMock()
        state.state = "10.0"  # Below MIN_WATER_TEMP (20)
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.current_water_temp is None

    def test_sensor_out_of_range_high(self, mock_hass):
        """Should reject water temp above valid range."""
        state = MagicMock()
        state.state = "100.0"  # Above MAX_WATER_TEMP (90)
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        assert comp.current_water_temp is None


class TestReferenceTemperature:
    """Tests for reference temperature calculation."""

    def test_initial_reference_is_default(self, mock_hass):
        """Reference should start at default value."""
        comp = WaterTempCompensation(mock_hass, None)
        assert comp.reference_water_temp == DEFAULT_REFERENCE_TEMP

    def test_first_update_becomes_reference(self, mock_hass, mock_sensor_state):
        """First measurement should become the reference."""
        mock_hass.states.get.return_value = mock_sensor_state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        comp.update_reference(45.0)

        assert comp.reference_water_temp == 45.0
        assert comp.sample_count == 1

    def test_ema_update(self, mock_hass, mock_sensor_state):
        """Subsequent updates should use EMA."""
        mock_hass.states.get.return_value = mock_sensor_state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # First update: becomes reference
        comp.update_reference(50.0)
        assert comp.reference_water_temp == 50.0

        # Second update: EMA
        comp.update_reference(60.0)
        expected = EMA_ALPHA * 60.0 + (1 - EMA_ALPHA) * 50.0
        assert comp.reference_water_temp == pytest.approx(expected, abs=0.01)

    def test_update_from_sensor(self, mock_hass, mock_sensor_state):
        """Update should read from sensor when no value provided."""
        mock_sensor_state.state = "55.0"
        mock_hass.states.get.return_value = mock_sensor_state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        comp.update_reference()  # No value, reads sensor

        assert comp.reference_water_temp == 55.0


class TestCompensationFactor:
    """Tests for compensation factor calculation."""

    def test_factor_equals_one_at_reference(self, mock_hass):
        """Factor should be 1.0 when current equals reference."""
        state = MagicMock()
        state.state = "50.0"
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # Set reference to 50
        comp.update_reference(50.0)

        factor = comp.get_compensation_factor()
        assert factor == pytest.approx(1.0, abs=0.01)

    def test_factor_higher_when_water_colder(self, mock_hass):
        """Factor should be > 1 when water is colder than reference."""
        state = MagicMock()
        state.state = "40.0"  # Colder than reference
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # Set reference to 50
        comp.update_reference(50.0)

        factor = comp.get_compensation_factor()
        # 50/40 = 1.25
        assert factor == pytest.approx(1.25, abs=0.01)

    def test_factor_lower_when_water_hotter(self, mock_hass):
        """Factor should be < 1 when water is hotter than reference."""
        state = MagicMock()
        state.state = "60.0"  # Hotter than reference
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # Set reference to 50
        comp.update_reference(50.0)

        factor = comp.get_compensation_factor()
        # 50/60 = 0.833
        assert factor == pytest.approx(0.833, abs=0.01)

    def test_factor_clamped_high(self, mock_hass):
        """Factor should be clamped to MAX_FACTOR."""
        state = MagicMock()
        state.state = "25.0"  # Very cold
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # Set reference to high (will cause high factor)
        comp._reference_temp = 70.0
        comp._reference_initialized = True

        factor = comp.get_compensation_factor()
        # 70/25 = 2.8 but clamped to MAX_FACTOR
        assert factor == MAX_FACTOR

    def test_factor_clamped_low(self, mock_hass):
        """Factor should be clamped to MIN_FACTOR."""
        state = MagicMock()
        state.state = "80.0"  # Very hot
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        # Set reference to low (will cause low factor)
        comp._reference_temp = 30.0
        comp._reference_initialized = True

        factor = comp.get_compensation_factor()
        # 30/80 = 0.375 but clamped to MIN_FACTOR
        assert factor == MIN_FACTOR


class TestCompensatePower:
    """Tests for power compensation."""

    def test_power_unchanged_at_reference(self, mock_hass):
        """Power should be unchanged when at reference temperature."""
        state = MagicMock()
        state.state = "50.0"
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp.update_reference(50.0)

        result = comp.compensate_power(50.0)
        assert result == pytest.approx(50.0, abs=0.1)

    def test_power_increased_when_water_cold(self, mock_hass):
        """Power should increase when water is colder than reference."""
        state = MagicMock()
        state.state = "35.0"  # Cold water
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp.update_reference(50.0)

        result = comp.compensate_power(50.0)
        # 50 * (50/35) = 71.4%
        assert result == pytest.approx(71.4, abs=0.5)

    def test_power_decreased_when_water_hot(self, mock_hass):
        """Power should decrease when water is hotter than reference."""
        state = MagicMock()
        state.state = "70.0"  # Hot water
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp.update_reference(50.0)

        result = comp.compensate_power(50.0)
        # 50 * (50/70) = 35.7%
        assert result == pytest.approx(35.7, abs=0.5)

    def test_power_clamped_to_100(self, mock_hass):
        """Compensated power should not exceed 100%."""
        state = MagicMock()
        state.state = "30.0"  # Very cold
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp.update_reference(50.0)

        result = comp.compensate_power(80.0)
        # 80 * (50/30) = 133% but clamped to 100
        assert result == 100.0

    def test_power_clamped_to_0(self, mock_hass):
        """Compensated power should not go below 0%."""
        state = MagicMock()
        state.state = "50.0"
        mock_hass.states.get.return_value = state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp.update_reference(50.0)

        result = comp.compensate_power(-10.0)  # Invalid input
        assert result == 0.0


class TestPersistence:
    """Tests for state save/load."""

    def test_save_state(self, mock_hass):
        """Should save reference temperature and metadata."""
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        comp._reference_temp = 55.0
        comp._reference_initialized = True
        comp._sample_count = 10

        state = comp.save_state()

        assert state["reference_temp"] == 55.0
        assert state["reference_initialized"] is True
        assert state["sample_count"] == 10

    def test_load_state(self, mock_hass):
        """Should restore reference temperature and metadata."""
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        state = {
            "reference_temp": 55.0,
            "reference_initialized": True,
            "sample_count": 10,
        }
        comp.load_state(state)

        assert comp.reference_water_temp == 55.0
        assert comp._reference_initialized is True
        assert comp.sample_count == 10

    def test_load_empty_state(self, mock_hass):
        """Should handle empty state gracefully."""
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        original_ref = comp.reference_water_temp

        comp.load_state({})

        assert comp.reference_water_temp == original_ref

    def test_load_none_state(self, mock_hass):
        """Should handle None state gracefully."""
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        original_ref = comp.reference_water_temp

        comp.load_state(None)

        assert comp.reference_water_temp == original_ref

    def test_load_invalid_temp_rejected(self, mock_hass):
        """Should reject invalid temperature in saved state."""
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")

        state = {"reference_temp": 150.0}  # Invalid (> MAX_WATER_TEMP)
        comp.load_state(state)

        # Should keep default, not load invalid value
        assert comp.reference_water_temp == DEFAULT_REFERENCE_TEMP


class TestRepr:
    """Tests for string representation."""

    def test_repr_without_sensor(self, mock_hass):
        """Should show disabled state in repr."""
        comp = WaterTempCompensation(mock_hass, None)
        repr_str = repr(comp)
        assert "None" in repr_str
        assert "enabled=False" in repr_str

    def test_repr_with_sensor(self, mock_hass, mock_sensor_state):
        """Should show enabled state in repr."""
        mock_hass.states.get.return_value = mock_sensor_state
        comp = WaterTempCompensation(mock_hass, "sensor.water_temp")
        repr_str = repr(comp)
        assert "sensor.water_temp" in repr_str
        assert "enabled=True" in repr_str
