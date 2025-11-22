"""Enhanced Auto TPI Manager with robust learning algorithm."""

import logging
import json
import os
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass, asdict
from enum import Enum

from homeassistant.core import HomeAssistant

from .const import (
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 2
MIN_DATA_POINTS = 100  # More data for more robustness
MAX_DATA_POINTS = 5000
MIN_HEATING_CYCLES = 10  # Complete heating cycles needed


class LearningQuality(Enum):
    """Quality of learning."""
    INSUFFICIENT = "insufficient"
    POOR = "poor"
    FAIR = "fair"
    GOOD = "good"
    EXCELLENT = "excellent"


@dataclass
class DataPoint:
    """Structured data point."""
    timestamp: float
    room_temp: float
    ext_temp: float
    power_percent: float
    target_temp: float
    is_heating: bool
    temp_derivative: Optional[float] = None
    
    def to_dict(self) -> dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, d: dict):
        return cls(**d)


@dataclass
class ThermalModel:
    """Building thermal model."""
    # Thermal loss coefficient (W/K) normalized
    heat_loss_coef: float  # alpha
    # Heating efficiency (K/s for 100% power)
    heating_efficiency: float  # beta
    # Thermal time constant (seconds)
    time_constant: float  # tau = 1/alpha
    # Coefficient of determination
    r_squared: float
    # Root mean squared error (K/h)
    rmse: float
    # Number of points used
    n_points: int
    
    def to_dict(self) -> dict:
        return asdict(self)


class AutoTpiManager:
    """Auto TPI Manager with robust learning algorithm."""
    
    def __init__(self, hass: HomeAssistant, unique_id: str, cycle_min: int):
        self._hass = hass
        self._unique_id = unique_id
        self._cycle_min = cycle_min
        self._data: List[DataPoint] = []
        self._learning_active = False
        self._last_update = None
        self._storage_path = hass.config.path(
            f".storage/versatile_thermostat_{unique_id}_auto_tpi_v2.json"
        )
        self._thermal_model: Optional[ThermalModel] = None
        self._calculated_params = {}
        self._learning_quality = LearningQuality.INSUFFICIENT
        
        # Cycle detection
        self._heating_cycles = []
        self._current_cycle_start = None
        self._last_power_state = 0.0

    @property
    def learning_active(self) -> bool:
        return self._learning_active

    @property
    def data_points(self) -> int:
        return len(self._data)

    @property
    def heating_cycles_count(self) -> int:
        """Count of heating cycles."""
        return len(self._heating_cycles)

    @property
    def min_data_points(self) -> int:
        """Minimum data points required."""
        return MIN_DATA_POINTS

    @property
    def progression(self) -> int:
        """Progression based on heating cycles."""
        cycle_progress = min(100, (len(self._heating_cycles) / MIN_HEATING_CYCLES) * 100)
        data_progress = min(100, (len(self._data) / MIN_DATA_POINTS) * 100)
        return int(min(cycle_progress, data_progress))

    @property
    def learning_quality(self) -> str:
        return self._learning_quality.value
        
    @property
    def time_constant(self) -> Optional[float]:
        """Return the thermal time constant in hours if available."""
        if self._thermal_model:
            return round(self._thermal_model.time_constant / 3600, 2)
        return None
        
    @property
    def confidence(self) -> Optional[float]:
        """Return the confidence (R²) in percent if available."""
        if self._thermal_model:
            return round(self._thermal_model.r_squared * 100, 1)
        return None

    def get_calculated_params(self) -> dict:
        return self._calculated_params

    def get_thermal_model(self) -> Optional[dict]:
        """Return the thermal model."""
        return self._thermal_model.to_dict() if self._thermal_model else None

    async def start_learning(self):
        """Start learning."""
        _LOGGER.info("%s - Starting Enhanced Auto TPI learning", self._unique_id)
        self._learning_active = True
        self._data.clear()
        self._heating_cycles.clear()
        await self.async_save_data()

    async def stop_learning(self):
        """Stop learning."""
        _LOGGER.info("%s - Stopping Auto TPI learning", self._unique_id)
        self._learning_active = False
        await self.async_save_data()

    def _is_outlier(self, point: DataPoint, window: List[DataPoint]) -> bool:
        """Detect outliers with IQR method."""
        if len(window) < 10:
            return False
        
        temps = [p.room_temp for p in window]
        q1, q3 = np.percentile(temps, [25, 75])
        iqr = q3 - q1
        lower = q1 - 3 * iqr
        upper = q3 + 3 * iqr
        
        return point.room_temp < lower or point.room_temp > upper

    def _detect_heating_cycle(self, power_percent: float):
        """Detect complete heating cycles."""
        is_heating = power_percent > 10.0
        
        # Cycle start
        if is_heating and self._last_power_state < 10.0:
            self._current_cycle_start = datetime.now()
        
        # Cycle end
        elif not is_heating and self._last_power_state > 10.0:
            if self._current_cycle_start:
                cycle_duration = (datetime.now() - self._current_cycle_start).total_seconds()
                if 60 < cycle_duration < 3600:  # Between 1min and 1h
                    self._heating_cycles.append({
                        'start': self._current_cycle_start.isoformat(),
                        'duration': cycle_duration
                    })
                    _LOGGER.debug("%s - Auto TPI: Heating cycle detected: %.1fs", 
                                self._unique_id, cycle_duration)
            self._current_cycle_start = None
        
        self._last_power_state = power_percent

    async def update(self, room_temp: float, ext_temp: float, 
                    power_percent: float, target_temp: float):
        """Add a data point with validation."""
        if not self._learning_active:
            return

        now = datetime.now()
        
        # Adaptive throttling
        min_interval = self._cycle_min * 60 * 0.5  # 50% of cycle
        if self._last_update and (now - self._last_update).total_seconds() < min_interval:
            return

        # Data validation
        if room_temp is None or ext_temp is None or power_percent is None or target_temp is None:
            return
        
        if not (-20 <= ext_temp <= 40 and 5 <= room_temp <= 35):
            _LOGGER.warning("%s - Auto TPI: Temperature out of reasonable range", self._unique_id)
            return

        # Cycle detection
        self._detect_heating_cycle(power_percent)

        # Point creation
        point = DataPoint(
            timestamp=now.timestamp(),
            room_temp=room_temp,
            ext_temp=ext_temp,
            power_percent=power_percent,
            target_temp=target_temp,
            is_heating=power_percent > 10.0
        )

        # Outlier filtering (on sliding window)
        recent_window = self._data[-50:] if len(self._data) >= 50 else self._data
        if self._is_outlier(point, recent_window):
            _LOGGER.warning("%s - Auto TPI: Outlier detected, skipping data point", self._unique_id)
            return

        self._data.append(point)

        # Memory limitation
        if len(self._data) > MAX_DATA_POINTS:
            self._data = self._data[-MAX_DATA_POINTS:]

        self._last_update = now
        
        # Derivative calculation
        self._compute_derivatives()
        
        await self.async_save_data()
        
        _LOGGER.debug("%s - Data point added. Total: %d, Cycles: %d", 
                     self._unique_id, len(self._data), len(self._heating_cycles))

    def _compute_derivatives(self):
        """Calculate smoothed temperature derivatives."""
        if len(self._data) < 3:
            return
        
        # Use a sliding window for smoothing
        window = 5
        for i in range(window, len(self._data)):
            # If already calculated, skip
            if self._data[i].temp_derivative is not None:
                continue

            points = self._data[i-window:i+1]
            times = [p.timestamp for p in points]
            temps = [p.room_temp for p in points]
            
            # Local linear regression for derivative
            if len(times) > 1:
                dt = times[-1] - times[0]
                if dt > 0:
                    dT = temps[-1] - temps[0]
                    self._data[i].temp_derivative = (dT / dt) * 3600  # K/h

    async def calculate(self) -> Optional[dict]:
        """Calculate TPI parameters with robust validation."""
        
        # Pre-checks
        if len(self._data) < MIN_DATA_POINTS:
            _LOGGER.debug("%s - Insufficient data (%d/%d)", 
                         self._unique_id, len(self._data), MIN_DATA_POINTS)
            return None
        
        if len(self._heating_cycles) < MIN_HEATING_CYCLES:
            _LOGGER.debug("%s - Auto TPI: Insufficient heating cycles (%d/%d)", 
                         self._unique_id, len(self._heating_cycles), MIN_HEATING_CYCLES)
            return None

        try:
            # Filtering: only points with valid derivative
            valid_points = [p for p in self._data if p.temp_derivative is not None]
            
            if len(valid_points) < MIN_DATA_POINTS:
                return None

            # Model: dT/dt = -alpha*(T_room - T_ext) + beta*Power - gamma*(T_room - T_target)
            # Simplified: dT/dt = -k_loss*dT_ext + k_heat*Power
            
            Y = np.array([p.temp_derivative for p in valid_points])
            
            # Explanatory variables
            temp_diff = np.array([p.room_temp - p.ext_temp for p in valid_points])
            power = np.array([p.power_percent / 100.0 for p in valid_points])
            target_diff = np.array([p.room_temp - p.target_temp for p in valid_points])
            
            # Design matrix with inertia term
            # X columns: -(Tr - Text), Power, -(Tr - Ttarget)
            
            X = np.column_stack([
                -temp_diff,  # Thermal losses
                power,        # Heat input
                -target_diff  # Correction towards target (inertia/control loop effect)
            ])
            
            # Weighted regression (recent data more important)
            timestamps = np.array([p.timestamp for p in valid_points])
            age = timestamps[-1] - timestamps
            weights = np.exp(-age / (7 * 86400))  # Decay over 7 days
            weights = weights / weights.sum()
            
            # Weighted least squares
            W = np.diag(weights)
            X_weighted = np.sqrt(W) @ X
            Y_weighted = np.sqrt(W) @ Y
            
            coeffs, residuals, rank, s = np.linalg.lstsq(X_weighted, Y_weighted, rcond=None)
            alpha, beta, gamma = coeffs
            
            _LOGGER.info("%s - Auto TPI: Model coefficients: alpha=%.6f, beta=%.6f, gamma=%.6f", 
                        self._unique_id, alpha, beta, gamma)

            # Physical validation
            if beta <= 0 or alpha <= 0:
                _LOGGER.warning("%s - Auto TPI: Invalid coefficients (non-positive)", self._unique_id)
                return None

            # Quality metrics calculation
            Y_pred = X @ coeffs
            ss_res = np.sum((Y - Y_pred) ** 2)
            ss_tot = np.sum((Y - np.mean(Y)) ** 2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
            rmse = np.sqrt(np.mean((Y - Y_pred) ** 2))
            
            # Quality assessment
            if r_squared < 0.3:
                self._learning_quality = LearningQuality.POOR
            elif r_squared < 0.5:
                self._learning_quality = LearningQuality.FAIR
            elif r_squared < 0.7:
                self._learning_quality = LearningQuality.GOOD
            else:
                self._learning_quality = LearningQuality.EXCELLENT
            
            _LOGGER.info("%s - Auto TPI: Model quality: R²=%.3f, RMSE=%.3f K/h, Quality=%s",
                        self._unique_id, r_squared, rmse, self._learning_quality.value)

            # Rejection if quality too low
            if r_squared < 0.3:
                _LOGGER.warning("%s - Auto TPI: Model quality too low (R²=%.3f)", 
                              self._unique_id, r_squared)
                return None

            # Thermal model calculation
            time_constant = 1.0 / alpha if alpha > 0 else 3600
            
            self._thermal_model = ThermalModel(
                heat_loss_coef=alpha,
                heating_efficiency=beta,
                time_constant=time_constant,
                r_squared=r_squared,
                rmse=rmse,
                n_points=len(valid_points)
            )

            # Optimized TPI coefficients calculation
            # k_ext: thermal loss compensation
            # The higher alpha is, the faster the losses are
            k_ext = np.clip(alpha / (alpha + beta), 0.01, 0.90)
            
            # k_int: reactivity to internal deviations
            # Based on characteristic time and efficiency
            desired_response_time = 1800  # 30 min target
            k_int = np.clip(1.0 / (beta * desired_response_time), 0.01, 0.40)
            
            self._calculated_params = {
                CONF_TPI_COEF_INT: round(k_int, 3),
                CONF_TPI_COEF_EXT: round(k_ext, 3),
                "confidence": round(r_squared, 3),
                "quality": self._learning_quality.value,
                "time_constant_hours": round(time_constant / 3600, 2)
            }
            
            await self.async_save_data()
            
            _LOGGER.info("%s - Auto TPI: TPI params calculated: k_int=%.3f, k_ext=%.3f (confidence=%.1f%%)",
                        self._unique_id, k_int, k_ext, r_squared * 100)
            
            return self._calculated_params

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Calculation error: %s", self._unique_id, e, exc_info=True)
            return None

    async def async_save_data(self):
        """Save data."""
        await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Sync save."""
        try:
            data = {
                "version": STORAGE_VERSION,
                "learning_active": self._learning_active,
                "data": [p.to_dict() for p in self._data[-1000:]],  # Keep last 1000
                "heating_cycles": self._heating_cycles[-50:],
                "thermal_model": self._thermal_model.to_dict() if self._thermal_model else None,
                "calculated_params": self._calculated_params,
                "learning_quality": self._learning_quality.value
            }
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            with open(self._storage_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Save error: %s", self._unique_id, e)

    async def async_load_data(self):
        """Load data."""
        await self._hass.async_add_executor_job(self._load_data_sync)

    def _load_data_sync(self):
        """Sync load."""
        if not os.path.exists(self._storage_path):
            return

        try:
            with open(self._storage_path, 'r') as f:
                data = json.load(f)
                
                # Check storage version or migration
                if data.get("version") == STORAGE_VERSION:
                    self._learning_active = data.get("learning_active", False)
                    self._data = [DataPoint.from_dict(d) for d in data.get("data", [])]
                    self._heating_cycles = data.get("heating_cycles", [])
                    
                    if model_data := data.get("thermal_model"):
                        self._thermal_model = ThermalModel(**model_data)
                    
                    self._calculated_params = data.get("calculated_params", {})
                    
                    quality_str = data.get("learning_quality", "insufficient")
                    self._learning_quality = LearningQuality(quality_str)
                    
                    _LOGGER.info("%s - Auto TPI: Data loaded: %d points, %d cycles, quality=%s",
                               self._unique_id, len(self._data), 
                               len(self._heating_cycles), self._learning_quality.value)
                else:
                    _LOGGER.info("%s - Auto TPI: Old storage version, starting fresh", self._unique_id)
                    # If version mismatch, we just start fresh or could implement migration
                    # For now, safe default is starting fresh to avoid data corruption
                    
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Load error %s", self._unique_id, e)
