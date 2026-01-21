# pylint: disable=line-too-long
"""Water Temperature Compensation Utility.

This module provides compensation for variable boiler water temperature
(heating curve / "loi d'eau") which affects the heating power delivered
by radiators with valves.

The compensation adjusts the power command sent to underlyings based on
the current water temperature relative to a reference temperature.
When water is colder, more power is commanded to achieve the same heating effect.
"""

import logging
from typing import Any

from homeassistant.core import HomeAssistant

_LOGGER = logging.getLogger(__name__)

# Constants
DEFAULT_REFERENCE_TEMP = 50.0  # Default reference water temperature in °C
EMA_ALPHA = 0.1  # Exponential moving average smoothing factor
MIN_FACTOR = 0.5  # Minimum compensation factor (clamp)
MAX_FACTOR = 2.0  # Maximum compensation factor (clamp)
MIN_WATER_TEMP = 20.0  # Minimum valid water temperature
MAX_WATER_TEMP = 90.0  # Maximum valid water temperature


class WaterTempCompensation:
    """Compensates for variable boiler water temperature.

    When a boiler uses a heating curve, the water temperature varies based on
    outdoor temperature. This class adjusts the power command to compensate:
    - Colder water → increase power command
    - Hotter water → decrease power command

    Usage:
        compensation = WaterTempCompensation(hass, "sensor.boiler_flow_temp")

        # In thermostat_prop, before sending to underlyings:
        adjusted_power = compensation.compensate_power(raw_power_percent)
    """

    def __init__(
        self,
        hass: HomeAssistant,
        water_temp_sensor_id: str | None = None,
    ) -> None:
        """Initialize water temperature compensation.

        Args:
            hass: Home Assistant instance
            water_temp_sensor_id: Entity ID of water temperature sensor (optional)
        """
        self._hass = hass
        self._sensor_id = water_temp_sensor_id
        self._reference_temp: float = DEFAULT_REFERENCE_TEMP
        self._reference_initialized: bool = False
        self._sample_count: int = 0

    @property
    def is_enabled(self) -> bool:
        """True if water temperature sensor is configured and available."""
        if not self._sensor_id:
            return False

        state = self._hass.states.get(self._sensor_id)
        if state is None or state.state in ("unknown", "unavailable"):
            return False

        return True

    @property
    def sensor_id(self) -> str | None:
        """The configured water temperature sensor entity ID."""
        return self._sensor_id

    @property
    def current_water_temp(self) -> float | None:
        """Current water temperature from sensor.

        Returns:
            Water temperature in °C, or None if unavailable.
        """
        if not self._sensor_id:
            return None

        state = self._hass.states.get(self._sensor_id)
        if state is None or state.state in ("unknown", "unavailable"):
            return None

        try:
            temp = float(state.state)
            if MIN_WATER_TEMP <= temp <= MAX_WATER_TEMP:
                return temp
            else:
                _LOGGER.warning(
                    "Water temperature %.1f°C is outside valid range [%.0f, %.0f]",
                    temp,
                    MIN_WATER_TEMP,
                    MAX_WATER_TEMP,
                )
                return None
        except (ValueError, TypeError):
            return None

    @property
    def reference_water_temp(self) -> float:
        """Reference water temperature (running average).

        This is the baseline temperature used for compensation.
        """
        return self._reference_temp

    @property
    def sample_count(self) -> int:
        """Number of samples used to compute reference temperature."""
        return self._sample_count

    def update_reference(self, water_temp: float | None = None) -> None:
        """Update reference temperature with new measurement.

        Uses exponential moving average (EMA) for smooth updates.
        If no temperature is provided, reads from sensor.

        Args:
            water_temp: Water temperature in °C (optional, reads sensor if None)
        """
        if water_temp is None:
            water_temp = self.current_water_temp

        if water_temp is None:
            return

        if not MIN_WATER_TEMP <= water_temp <= MAX_WATER_TEMP:
            _LOGGER.debug(
                "Ignoring water temp %.1f°C outside valid range", water_temp
            )
            return

        if not self._reference_initialized:
            # First measurement becomes the reference
            self._reference_temp = water_temp
            self._reference_initialized = True
            self._sample_count = 1
            _LOGGER.info(
                "Water temp compensation initialized with reference %.1f°C",
                self._reference_temp,
            )
        else:
            # EMA update
            self._reference_temp = (
                EMA_ALPHA * water_temp + (1 - EMA_ALPHA) * self._reference_temp
            )
            self._sample_count += 1

        _LOGGER.debug(
            "Water temp reference updated to %.1f°C (current=%.1f°C, samples=%d)",
            self._reference_temp,
            water_temp,
            self._sample_count,
        )

    def get_compensation_factor(self) -> float:
        """Get the compensation factor for current water temperature.

        Returns:
            Factor to multiply power by: T_reference / T_current.
            - Factor > 1 when water is colder → increase power
            - Factor < 1 when water is hotter → decrease power
            - Returns 1.0 if compensation is disabled or unavailable.
        """
        if not self.is_enabled:
            return 1.0

        current = self.current_water_temp
        if current is None or not self._reference_initialized:
            return 1.0

        # Avoid division by zero
        if current < 1.0:
            return 1.0

        # T_reference / T_current
        # Cold water (current < ref) → factor > 1 → more power
        # Hot water (current > ref) → factor < 1 → less power
        factor = self._reference_temp / current

        # Clamp to reasonable range
        factor = max(MIN_FACTOR, min(factor, MAX_FACTOR))

        _LOGGER.debug(
            "Water temp compensation factor: %.3f (ref=%.1f°C / current=%.1f°C)",
            factor,
            self._reference_temp,
            current,
        )

        return factor

    def compensate_power(self, raw_power_percent: float) -> float:
        """Compensate power output for current water temperature.

        Call this in thermostat_prop before sending power to underlyings.

        Args:
            raw_power_percent: Raw power percentage from algorithm (0-100)

        Returns:
            Compensated power percentage, clamped to 0-100.
        """
        factor = self.get_compensation_factor()
        compensated = raw_power_percent * factor

        # Clamp to valid range
        compensated = max(0.0, min(100.0, compensated))

        _LOGGER.debug(
            "Power compensated: %.1f%% → %.1f%% (factor=%.3f)",
            raw_power_percent,
            compensated,
            factor,
        )

        return compensated

    def save_state(self) -> dict[str, Any]:
        """Get state dictionary for persistence.

        Returns:
            Dictionary with reference temperature and metadata.
        """
        return {
            "reference_temp": self._reference_temp,
            "reference_initialized": self._reference_initialized,
            "sample_count": self._sample_count,
        }

    def load_state(self, state: dict[str, Any]) -> None:
        """Restore state from persistence.

        Args:
            state: Dictionary from save_state()
        """
        if not state:
            return

        if "reference_temp" in state:
            temp = state["reference_temp"]
            if isinstance(temp, (int, float)) and MIN_WATER_TEMP <= temp <= MAX_WATER_TEMP:
                self._reference_temp = float(temp)
                _LOGGER.debug("Restored water temp reference: %.1f°C", self._reference_temp)

        if "reference_initialized" in state:
            self._reference_initialized = bool(state["reference_initialized"])

        if "sample_count" in state:
            self._sample_count = int(state.get("sample_count", 0))

    def __repr__(self) -> str:
        """String representation for debugging."""
        return (
            f"WaterTempCompensation(sensor={self._sensor_id}, "
            f"enabled={self.is_enabled}, ref={self._reference_temp:.1f}°C)"
        )
