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

STORAGE_VERSION = 3
MIN_DATA_POINTS = 100  # More data for more robustness
MAX_DATA_POINTS = 5000
MIN_TPI_CYCLES = 5  # Complete TPI cycles needed


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
class TpiCycle:
    """Represents a complete TPI cycle."""
    start_time: float
    duration_target: float  # cycle_min in seconds
    data_points: List[DataPoint]  # Points collected during the cycle
    end_time: Optional[float] = None
    
    @property
    def average_power(self) -> float:
        """Average power over the cycle."""
        if not self.data_points:
            return 0.0
        return sum(p.power_percent for p in self.data_points) / len(self.data_points)
        
    @property
    def temp_evolution(self) -> float:
        """ΔT between start and end of cycle."""
        if not self.data_points:
            return 0.0
        return self.data_points[-1].room_temp - self.data_points[0].room_temp
        
    @property
    def is_complete(self) -> bool:
        """Is the cycle finished (duration >= target)."""
        if self.end_time is not None:
            return True
        if not self.data_points:
            return False
        return (self.data_points[-1].timestamp - self.start_time) >= self.duration_target

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict):
        data = d.copy()
        if "data_points" in data:
            data["data_points"] = [DataPoint.from_dict(dp) for dp in data["data_points"]]
        return cls(**data)


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
        
        # New TPI Cycle tracking
        self._current_tpi_cycle: Optional[TpiCycle] = None
        self._completed_tpi_cycles: List[TpiCycle] = []
        self._thermostat_mode: str = "unknown"

    @property
    def learning_active(self) -> bool:
        return self._learning_active

    @property
    def data_points(self) -> int:
        return len(self._data)

    @property
    def heating_cycles_count(self) -> int:
        """Count of heating cycles."""
        return len(self._completed_tpi_cycles)

    @property
    def min_data_points(self) -> int:
        """Minimum data points required."""
        return MIN_DATA_POINTS

    @property
    def progression(self) -> int:
        """Progression based on heating cycles."""
        cycle_progress = min(100, (len(self._completed_tpi_cycles) / MIN_TPI_CYCLES) * 100)
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
        _LOGGER.info("%s - Auto TPI: Starting Enhanced Auto TPI learning", self._unique_id)
        self._learning_active = True
        self._data.clear()
        self._completed_tpi_cycles.clear()
        self._current_tpi_cycle = None
        await self.async_save_data()

    async def stop_learning(self):
        """Stop learning."""
        _LOGGER.info("%s - Auto TPI: Stopping Auto TPI learning", self._unique_id)
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

    async def update(self, room_temp: float, ext_temp: float,
                    power_percent: float, target_temp: float, hvac_action: str):
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
        
        # Add to current cycle if active
        if self._current_tpi_cycle:
            self._current_tpi_cycle.data_points.append(point)

        # Memory limitation
        if len(self._data) > MAX_DATA_POINTS:
            self._data = self._data[-MAX_DATA_POINTS:]

        self._last_update = now
        
        # Derivative calculation
        self._compute_derivatives()
        
        await self.async_save_data()
        
        _LOGGER.info("%s - Auto TPI: Data point added. Total: %d, Cycles: %d",
                     self._unique_id, len(self._data), len(self._completed_tpi_cycles))

    async def on_thermostat_mode_changed(self, new_mode: str, target_temp: float):
        """Handle thermostat mode changes."""
        _LOGGER.info("%s - Auto TPI: Mode changed: %s -> %s", self._unique_id, self._thermostat_mode, new_mode)
        self._thermostat_mode = new_mode

        # If switching to HEAT, start a new cycle
        if new_mode == "HEAT":
            if self._current_tpi_cycle:
                self._terminate_current_cycle()
            self._start_new_cycle()
        
        # If switching OFF (or anything else) from HEAT, terminate current cycle
        elif self._current_tpi_cycle:
             self._terminate_current_cycle()

    async def on_cycle_elapsed(self):
        """Handle end of TPI cycle (timer based)."""
        _LOGGER.info("%s - Auto TPI: Cycle elapsed", self._unique_id)
        
        if self._current_tpi_cycle:
             self._terminate_current_cycle()
             
        # If still in HEAT mode, start a new cycle immediately
        if self._thermostat_mode == "HEAT":
            self._start_new_cycle()

    def _start_new_cycle(self):
        """Start a new TPI cycle."""
        self._current_tpi_cycle = TpiCycle(
            start_time=datetime.now().timestamp(),
            duration_target=self._cycle_min * 60,
            data_points=[]
        )
        _LOGGER.info("%s - Auto TPI: New cycle started", self._unique_id)

    def _terminate_current_cycle(self):
        """Terminate the current TPI cycle."""
        if not self._current_tpi_cycle:
            return

        self._current_tpi_cycle.end_time = datetime.now().timestamp()
        
        # Validate cycle (e.g. check duration)
        duration = self._current_tpi_cycle.end_time - self._current_tpi_cycle.start_time
        if duration > 10:  # Minimum 10 seconds to be considered valid
             self._completed_tpi_cycles.append(self._current_tpi_cycle)
             _LOGGER.info("%s - Auto TPI: Cycle completed. Duration: %.1fs, Points: %d",
                          self._unique_id, duration, len(self._current_tpi_cycle.data_points))
        else:
             _LOGGER.info("%s - Auto TPI: Cycle discarded (too short). Duration: %.1fs",
                          self._unique_id, duration)
        
        self._current_tpi_cycle = None

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
        if len(self._completed_tpi_cycles) < MIN_TPI_CYCLES:
            _LOGGER.info("%s - Auto TPI: Insufficient TPI cycles (%d/%d)",
                         self._unique_id, len(self._completed_tpi_cycles), MIN_TPI_CYCLES)
            return None

        try:
            # Gather data from cycles
            y_list = []
            x_loss_list = []
            x_power_list = []
            
            # We want to fit: DeltaTemp/dt = -alpha * (Room - Ext) + beta * Power
            # So Y = DeltaTemp/dt
            # X1 = -(Room - Ext)
            # X2 = Power (0-1)

            for cycle in self._completed_tpi_cycles:
                if not cycle.data_points:
                    continue
                
                # Duration in hours
                duration_h = (cycle.end_time - cycle.start_time) / 3600.0
                if duration_h <= 0:
                    continue
                
                # Y: Temperature evolution rate (K/h)
                d_temp_dt = cycle.temp_evolution / duration_h
                
                # X variables (averages over the cycle)
                avg_room_temp = np.mean([p.room_temp for p in cycle.data_points])
                avg_ext_temp = np.mean([p.ext_temp for p in cycle.data_points])
                avg_power = cycle.average_power / 100.0  # Normalize to 0-1
                
                y_list.append(d_temp_dt)
                x_loss_list.append(-(avg_room_temp - avg_ext_temp))
                x_power_list.append(avg_power)
                
            if len(y_list) < MIN_TPI_CYCLES:
                return None
            
            Y = np.array(y_list)
            X = np.column_stack([x_loss_list, x_power_list])
            
            # Weighted regression (recent cycles more important)
            # Use end_time of cycles for weighting
            timestamps = np.array([c.end_time for c in self._completed_tpi_cycles if c.data_points and (c.end_time - c.start_time) > 0])
            if len(timestamps) != len(Y):
                 # Should match if filtered consistently.
                 # Re-filter to be safe or assuming consistent order
                 timestamps = np.array([c.end_time for c in self._completed_tpi_cycles if c.data_points and (c.end_time - c.start_time) > 0])
            
            age = timestamps.max() - timestamps
            weights = np.exp(-age / (14 * 86400))  # Decay over 14 days
            weights = weights / weights.sum()
            
            # Weighted least squares
            W = np.diag(weights)
            X_weighted = np.sqrt(W) @ X
            Y_weighted = np.sqrt(W) @ Y
            
            coeffs, residuals, rank, s = np.linalg.lstsq(X_weighted, Y_weighted, rcond=None)
            alpha, beta = coeffs
            
            _LOGGER.info("%s - Auto TPI: Model coefficients: alpha=%.6f, beta=%.6f",
                        self._unique_id, alpha, beta)

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
                n_points=len(self._completed_tpi_cycles)
            )

            # Optimized TPI coefficients calculation
            # k_ext: thermal loss compensation
            # The higher alpha is, the faster the losses are
            # Improved formula: alpha / beta represents the power % needed to compensate 1°C diff
            k_ext = np.clip(alpha / beta, 0.01, 0.20)
            
            # k_int: reactivity to internal deviations
            # Based on characteristic time and efficiency
            desired_response_time = 1800 / 3600  # 30 min target in hours
            k_int = np.clip(1.0 / (beta * desired_response_time), 0.01, 1.0)
            
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
                "completed_tpi_cycles": [c.to_dict() for c in self._completed_tpi_cycles[-50:]],
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
                version = data.get("version", 0)
                if version >= 3:
                    self._learning_active = data.get("learning_active", False)
                    self._data = [DataPoint.from_dict(d) for d in data.get("data", [])]
                    if cycles_data := data.get("completed_tpi_cycles"):
                        self._completed_tpi_cycles = [TpiCycle.from_dict(c) for c in cycles_data]
                    
                    if model_data := data.get("thermal_model"):
                        self._thermal_model = ThermalModel(**model_data)
                    
                    self._calculated_params = data.get("calculated_params", {})
                    
                    quality_str = data.get("learning_quality", "insufficient")
                    self._learning_quality = LearningQuality(quality_str)
                    
                    _LOGGER.info("%s - Auto TPI: Data loaded: %d points, %d tpi_cycles, quality=%s",
                               self._unique_id, len(self._data),
                               len(self._completed_tpi_cycles),
                               self._learning_quality.value)
                else:
                    _LOGGER.info("%s - Auto TPI: Storage version %d < 3. Discarding old data.",
                                 self._unique_id, version)
                    # Force clean start for version < 3
                    self._data = []
                    self._completed_tpi_cycles = []
                    self._current_tpi_cycle = None
                    self._calculated_params = {}
                    self._learning_quality = LearningQuality.INSUFFICIENT
                    # We do NOT return here, we just initialized empty state
                    
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Load error %s", self._unique_id, e)
