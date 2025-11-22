"""The Auto TPI Manager module."""

import logging
import json
import os
import numpy as np
from datetime import datetime

from homeassistant.core import HomeAssistant

from .const import (
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
    CONF_CYCLE_MIN,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 1
STORAGE_KEY = "versatile_thermostat.auto_tpi"

MIN_DATA_POINTS = 50  # ~4-5 hours @ 5min cycle
MAX_DATA_POINTS = 2000 # ~7 days @ 5min cycle

class AutoTpiManager:
    def __init__(self, hass: HomeAssistant, unique_id: str, cycle_min: int):
        self._hass = hass
        self._unique_id = unique_id
        self._cycle_min = cycle_min
        self._data = []
        self._learning_active = False
        self._last_update = None
        self._storage_path = hass.config.path(f".storage/versatile_thermostat_{unique_id}_auto_tpi.json")
        self._calculated_params = {}

    @property
    def learning_active(self):
        """Is learning active."""
        return self._learning_active

    @property
    def data_points(self):
        """Count of data points."""
        return len(self._data)

    @property
    def min_data_points(self):
        """Minimum data points required."""
        return MIN_DATA_POINTS

    @property
    def progression(self):
        """Learning progression %."""
        return min(100, round((len(self._data) / MIN_DATA_POINTS) * 100))

    def get_calculated_params(self):
        """Last calculated parameters."""
        return self._calculated_params

    async def start_learning(self):
        """Start learning."""
        _LOGGER.info("%s - Starting Auto TPI learning", self._unique_id)
        self._learning_active = True
        await self.async_save_data()

    async def stop_learning(self):
        """Stop learning."""
        _LOGGER.info("%s - Stopping Auto TPI learning", self._unique_id)
        self._learning_active = False
        await self.async_save_data()

    async def update(self, room_temp: float, ext_temp: float, power_percent: float, target_temp: float):
        """Add new data point."""
        _LOGGER.info("%s - Auto TPI: update called. Learning active: %s", self._unique_id, self._learning_active)
        if not self._learning_active:
            return

        now = datetime.now()
        
        # Throttle: 80% of cycle_min
        if self._last_update and (now - self._last_update).total_seconds() < (self._cycle_min * 60 * 0.8):
            _LOGGER.info("%s - Auto TPI: update skipped due to throttling", self._unique_id)
            return

        if room_temp is None or ext_temp is None or power_percent is None:
            return

        self._data.append({
            "timestamp": now.isoformat(),
            "room_temp": room_temp,
            "ext_temp": ext_temp,
            "power_percent": power_percent,
            "target_temp": target_temp
        })

        if len(self._data) > MAX_DATA_POINTS:
            self._data.pop(0)

        self._last_update = now
        await self.async_save_data()
        
        _LOGGER.debug("%s - Auto TPI: Added data point. Total: %d", self._unique_id, len(self._data))

    async def calculate(self) -> dict | None:
        """Calculate TPI parameters."""
        if len(self._data) < MIN_DATA_POINTS:
            _LOGGER.debug("%s - Auto TPI: Not enough data to calculate (%d/%d)", self._unique_id, len(self._data), MIN_DATA_POINTS)
            return None

        # Model: dT/dt = -alpha * (T_room - T_ext) + beta * Power
        timestamps = [datetime.fromisoformat(d["timestamp"]).timestamp() for d in self._data]
        room_temps = [d["room_temp"] for d in self._data]
        ext_temps = [d["ext_temp"] for d in self._data]
        powers = [d["power_percent"] / 100.0 for d in self._data]

        dt_list = []
        dT_list = []
        T_diff_list = []
        Power_list = []

        for i in range(1, len(self._data)):
            dt = timestamps[i] - timestamps[i-1]
            if dt <= 0 or dt > 3600:
                continue
            
            dT = room_temps[i] - room_temps[i-1]
            dT_dt = dT / dt
            
            avg_room = (room_temps[i] + room_temps[i-1]) / 2
            avg_ext = (ext_temps[i] + ext_temps[i-1]) / 2
            T_diff = avg_room - avg_ext
            
            Power = powers[i-1]

            dt_list.append(dt)
            dT_list.append(dT_dt)
            T_diff_list.append(T_diff)
            Power_list.append(Power)

        if len(dT_list) < MIN_DATA_POINTS:
             return None

        # Regression: dT/dt = alpha * (-dT_diff) + beta * Power
        Y = np.array(dT_list)
        X = np.column_stack((-np.array(T_diff_list), np.array(Power_list)))

        try:
            coeffs, residuals, rank, s = np.linalg.lstsq(X, Y, rcond=None)
            alpha, beta = coeffs
            
            _LOGGER.info("%s - Auto TPI: Calculated alpha=%.6f, beta=%.6f", self._unique_id, alpha, beta)

            if beta <= 0 or alpha <= 0:
                _LOGGER.warning("%s - Auto TPI: Invalid coefficients (alpha or beta <= 0). Calculation failed.", self._unique_id)
                return None

            k_ext = alpha / beta
            
            # Target 30min correction
            target_correction_time = 1800.0
            k_int = 1.0 / (beta * target_correction_time)
            
            k_ext = max(0.0, min(1.0, k_ext))
            k_int = max(0.0, min(1.0, k_int))
            
            self._calculated_params = {
                CONF_TPI_COEF_INT: round(k_int, 3),
                CONF_TPI_COEF_EXT: round(k_ext, 3),
            }
            await self.async_save_data()
            return self._calculated_params

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Error during calculation: %s", self._unique_id, e)
            return None

    async def async_save_data(self):
        """Save data."""
        await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Save data (sync)."""
        try:
            data = {
                "version": STORAGE_VERSION,
                "learning_active": self._learning_active,
                "data": self._data,
                "calculated_params": self._calculated_params
            }
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            with open(self._storage_path, 'w') as f:
                json.dump(data, f)
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Failed to save data: %s", self._unique_id, e)

    async def async_load_data(self):
        """Load data."""
        await self._hass.async_add_executor_job(self._load_data_sync)

    def _load_data_sync(self):
        """Load data (sync)."""
        if not os.path.exists(self._storage_path):
            return

        try:
            with open(self._storage_path, 'r') as f:
                data = json.load(f)
                if data.get("version") == STORAGE_VERSION:
                    self._learning_active = data.get("learning_active", False)
                    self._data = data.get("data", [])
                    self._calculated_params = data.get("calculated_params", {})
                    _LOGGER.info("%s - Auto TPI: Data loaded. Learning active: %s, Data points: %d", self._unique_id, self._learning_active, len(self._data))
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Failed to load data: %s", self._unique_id, e)
