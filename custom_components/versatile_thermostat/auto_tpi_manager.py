"""Auto TPI Manager with robust learning algorithm."""

import logging
import json
import os
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, Optional, List, Tuple
from dataclasses import dataclass, asdict, fields
from enum import Enum

try:
    from scipy.optimize import minimize
except ImportError:
    minimize = None

try:
    from scipy.signal import savgol_filter
except ImportError:
    savgol_filter = None

from homeassistant.core import HomeAssistant

from .const import (
    CONF_TPI_COEF_INT,
    CONF_TPI_COEF_EXT,
)

_LOGGER = logging.getLogger(__name__)

STORAGE_VERSION = 3
MIN_DATA_POINTS = 100  # More data for more robustness
MAX_DATA_POINTS = 10000
MIN_TPI_CYCLES = 20  # Complete TPI cycles needed (16 train + 4 val with 80/20 split)
MAX_STORED_CYCLES = 2500  # Approx 30 days history


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
    humidity: Optional[float] = None
    
    def to_dict(self) -> dict:
        return asdict(self)
    
    @classmethod
    def from_dict(cls, d: dict):
        # Backward compatibility: ignore unknown fields (like is_gated_off)
        valid_fields = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in valid_fields})


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
    humidity_coef: Optional[float] = None
    
    def to_dict(self) -> dict:
        return asdict(self)


class NumpyEncoder(json.JSONEncoder):
    """Custom encoder for numpy data types."""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.bool_):
            return bool(obj)
        return super().default(obj)


class AutoTpiManager:
    """Auto TPI Manager with robust learning algorithm."""
    
    def __init__(self, hass: HomeAssistant, unique_id: str, name: str, cycle_min: int,
                 tpi_threshold_low: float = 0.0, tpi_threshold_high: float = 0.0):
        self._hass = hass
        self._unique_id = unique_id
        self._name = name
        self._cycle_min = cycle_min
        self._tpi_threshold_low = tpi_threshold_low
        self._tpi_threshold_high = tpi_threshold_high
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
        """Start learning (preserves existing data)."""
        _LOGGER.info("%s - Auto TPI: Starting Auto TPI learning (data preserved)", self._name)
        self._learning_active = True
        await self.async_save_data()

    async def stop_learning(self):
        """Stop learning without clearing data."""
        _LOGGER.info("%s - Auto TPI: Stopping Auto TPI learning (data preserved)", self._name)
        self._learning_active = False
        await self.async_save_data()

    async def reset_learning_data(self):
        """Reset all learning data."""
        _LOGGER.info("%s - Auto TPI: Resetting all learning data", self._name)
        self._data.clear()
        self._completed_tpi_cycles.clear()
        self._current_tpi_cycle = None
        self._calculated_params = {}
        self._thermal_model = None
        self._learning_quality = LearningQuality.INSUFFICIENT
        await self.async_save_data()

    def _detect_concept_drift(self) -> bool:
        """Detect concept drift by comparing recent vs older cycles."""
        cycles = self._completed_tpi_cycles
        if len(cycles) < 40:
            return False

        recent = cycles[-20:]
        older = cycles[-40:-20]

        # Extract metric: temp_evolution
        recent_vals = [c.temp_evolution for c in recent]
        older_vals = [c.temp_evolution for c in older]

        mu_recent = np.mean(recent_vals)
        sigma_recent = np.std(recent_vals)
        
        mu_older = np.mean(older_vals)
        sigma_older = np.std(older_vals)

        # Avoid division by zero if both std devs are 0
        if sigma_recent == 0 and sigma_older == 0:
            return False
            
        sigma_pooled = np.sqrt((sigma_recent**2 + sigma_older**2) / 2)
        
        if sigma_pooled == 0:
            return False

        delta_norm = abs(mu_recent - mu_older) / sigma_pooled

        if delta_norm > 2.0:
            _LOGGER.warning("%s - Auto TPI: Concept drift detected (delta_norm=%.2f). Old data may be less relevant.",
                            self._name, delta_norm)
            # We do NOT stop learning, we rely on the sliding window (MAX_STORED_CYCLES) to adapt.
            return False

        return False

    def _compress_historical_data(self) -> List[DataPoint]:
        """Compress historical data by downsampling older points."""
        if len(self._data) <= 500:
            return self._data

        # Split data
        recent_data = self._data[-500:]
        older_data = self._data[:-500]

        # Downsample older data (1 out of 5)
        compressed_older = older_data[::5]

        # Combine
        compressed_data = compressed_older + recent_data
        
        reduction = len(self._data) - len(compressed_data)
        if reduction > 0:
             _LOGGER.info("%s - Auto TPI: Data compressed. Removed %d points.", self._name, reduction)

        return compressed_data

    def _is_outlier(self, point: DataPoint, window: List[DataPoint]) -> bool:
        """Detect outliers with IQR method."""
        if len(window) < 10:
            return False
        
        temps = [p.room_temp for p in window]
        q1, q3 = np.percentile(temps, [25, 75])
        iqr = q3 - q1
        
        # Enforce a minimum IQR to avoid rejecting data when temperature is stable
        # 0.5°C allows for normal sensor noise/fluctuation without triggering outlier detection
        if iqr < 0.2:
            iqr = 0.2
            
        lower = q1 - 3 * iqr
        upper = q3 + 3 * iqr
        
        return point.room_temp < lower or point.room_temp > upper

    async def update(self, room_temp: float, ext_temp: float,
                    power_percent: float, target_temp: float, hvac_action: str,
                    humidity: Optional[float] = None):
        """Add a data point with validation."""
        if not self._learning_active:
            return

        now = datetime.now()
        
        # Adaptive throttling
        min_interval = self._cycle_min * 60 * 0.1  # 10% of cycle
        if self._last_update and (now - self._last_update).total_seconds() < min_interval:
            return

        # Data validation
        if room_temp is None or ext_temp is None or power_percent is None or target_temp is None:
            return
        
        if not (-20 <= ext_temp <= 40 and 5 <= room_temp <= 35):
            _LOGGER.warning("%s - Auto TPI: Temperature out of reasonable range", self._name)
            return

        # Point creation
        point = DataPoint(
            timestamp=now.timestamp(),
            room_temp=room_temp,
            ext_temp=ext_temp,
            power_percent=power_percent,
            target_temp=target_temp,
            is_heating=power_percent > 10.0,
            humidity=humidity
        )

        # Outlier filtering (on sliding window)
        recent_window = self._data[-50:] if len(self._data) >= 50 else self._data
        if self._is_outlier(point, recent_window):
            _LOGGER.warning("%s - Auto TPI: Outlier detected, skipping data point", self._name)
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
                     self._name, len(self._data), len(self._completed_tpi_cycles))

    async def on_thermostat_mode_changed(self, new_mode: str, target_temp: float):
        """Handle thermostat mode changes."""
        _LOGGER.info("%s - Auto TPI: Mode changed: %s -> %s", self._name, self._thermostat_mode, new_mode)
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
        _LOGGER.info("%s - Auto TPI: Cycle elapsed", self._name)
        
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
        _LOGGER.info("%s - Auto TPI: New cycle started", self._name)

    def _terminate_current_cycle(self):
        """Terminate the current TPI cycle."""
        if not self._current_tpi_cycle:
            return

        self._current_tpi_cycle.end_time = datetime.now().timestamp()
        
        # Validate cycle (e.g. check duration)
        duration = self._current_tpi_cycle.end_time - self._current_tpi_cycle.start_time
        if duration > 10:  # Minimum 10 seconds to be considered valid
             self._completed_tpi_cycles.append(self._current_tpi_cycle)
             
             # Prune old cycles to keep within MAX_STORED_CYCLES
             if len(self._completed_tpi_cycles) > MAX_STORED_CYCLES:
                 self._completed_tpi_cycles = self._completed_tpi_cycles[-MAX_STORED_CYCLES:]
                 
             _LOGGER.info("%s - Auto TPI: Cycle completed. Duration: %.1fs, Points: %d. Total cycles: %d",
                          self._name, duration, len(self._current_tpi_cycle.data_points), len(self._completed_tpi_cycles))
        else:
             _LOGGER.info("%s - Auto TPI: Cycle discarded (too short). Duration: %.1fs",
                          self._name, duration)
        
        self._current_tpi_cycle = None

    def _compute_derivatives(self):
        """Calculate temperature derivatives dispatching to best available method."""
        if savgol_filter is not None and len(self._data) >= 11:
            self._compute_smooth_derivatives()
        else:
            self._compute_derivatives_simple()

    def _compute_smooth_derivatives(self):
        """Calculate derivatives using Savitzky-Golay filter."""
        window_length = 11
        polyorder = 2
        
        if len(self._data) < window_length:
             return

        # Extract temperature and time
        temps = np.array([p.room_temp for p in self._data])
        times = np.array([p.timestamp for p in self._data])
        
        # Calculate derivation step (average dt)
        dt_avg = np.mean(np.diff(times))
        if dt_avg <= 0:
            return

        # Savitzky-Golay derivative (deriv=1)
        # delta=dt_avg scales the derivative to unit/second
        try:
            derivs = savgol_filter(temps, window_length=window_length, polyorder=polyorder, deriv=1, delta=dt_avg)
            
            # Convert to K/h
            derivs_h = derivs * 3600.0
            
            # Update data points
            for i in range(len(self._data)):
                self._data[i].temp_derivative = derivs_h[i]
                
            _LOGGER.debug("%s - Auto TPI: Derivatives updated using Savitzky-Golay", self._name)
        except Exception as e:
            _LOGGER.warning("%s - Auto TPI: Savitzky-Golay failed: %s. Fallback to simple.", self._name, e)
            self._compute_derivatives_simple()

    def _compute_derivatives_simple(self):
        """Calculate derivatives using simple local regression."""
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
        
        _LOGGER.debug("%s - Auto TPI: Derivatives updated using simple method", self._name)

    def _apply_gating_filter(self, points: List[DataPoint]) -> List[DataPoint]:
        """
        Apply gating filter dynamically at calculation time.
        This allows for flexible threshold adjustment without re-acquisition.
        """
        # If thresholds are 0.0, filtering is disabled
        if self._tpi_threshold_low == 0.0 or self._tpi_threshold_high == 0.0:
            return points
        
        filtered = []
        gated_off = False
        
        for point in points:
            overshoot = point.room_temp - point.target_temp
            
            # Hysteresis logic
            if overshoot > self._tpi_threshold_high:
                gated_off = True
            elif overshoot < self._tpi_threshold_low:
                gated_off = False
            
            # Keep only non-gated points
            if not gated_off:
                filtered.append(point)
        
        return filtered

    async def calculate(self) -> Optional[dict]:
        """Calculate TPI parameters with robust validation using Constrained Optimization."""
        
        # Check for concept drift (now only warns)
        self._detect_concept_drift()

        # Pre-checks
        if len(self._completed_tpi_cycles) < MIN_TPI_CYCLES:
            _LOGGER.info("%s - Auto TPI: Insufficient TPI cycles (%d/%d)",
                         self._name, len(self._completed_tpi_cycles), MIN_TPI_CYCLES)
            return None
        
        if minimize is None:
            _LOGGER.warning("%s - Auto TPI: Scipy minimize not available. Cannot perform calculation.", self._name)
            return None

        try:
            # Gather data from cycles
            y_list = []
            x_loss_list = []
            x_power_list = []
            x_humidity_list = []

            # Statistics for logging
            total_points = 0
            filtered_points = 0
            
            for cycle in self._completed_tpi_cycles:
                if not cycle.data_points:
                    continue

                total_points += len(cycle.data_points)

                # Dynamic filtering
                valid_points = self._apply_gating_filter(cycle.data_points)
                
                if len(valid_points) < 2:
                    filtered_points += len(cycle.data_points)
                    continue
                
                filtered_points += len(cycle.data_points) - len(valid_points)
                
                # Re-calculate cycle metrics based on valid points
                start_time = valid_points[0].timestamp
                end_time = valid_points[-1].timestamp
                
                # Duration in hours
                duration_h = (end_time - start_time) / 3600.0
                if duration_h <= 0.01: # Avoid division by zero
                    continue
                
                # Y: Temperature evolution rate (K/h)
                temp_evolution = valid_points[-1].room_temp - valid_points[0].room_temp
                d_temp_dt = temp_evolution / duration_h
                
                # X variables (averages over the valid points)
                avg_room_temp = np.mean([p.room_temp for p in valid_points])
                avg_ext_temp = np.mean([p.ext_temp for p in valid_points])
                avg_power = sum(p.power_percent for p in valid_points) / len(valid_points) / 100.0  # Normalize to 0-1
                
                # Humidity handling
                humidities = [p.humidity for p in valid_points if p.humidity is not None]
                if humidities:
                    avg_humidity = np.mean(humidities) - 50.0  # Centered
                else:
                    avg_humidity = None

                y_list.append(d_temp_dt)
                x_loss_list.append(-(avg_room_temp - avg_ext_temp))
                x_power_list.append(avg_power)
                x_humidity_list.append(avg_humidity)

            _LOGGER.info(
                "%s - Auto TPI: Filtering stats: %d/%d points filtered (%.1f%%), %d samples from %d cycles",
                self._name, filtered_points, total_points,
                100 * filtered_points / total_points if total_points > 0 else 0,
                len(y_list), len(self._completed_tpi_cycles)
            )
                
            if len(y_list) < MIN_TPI_CYCLES:
                _LOGGER.warning(
                    "%s - Auto TPI: Only %d valid samples after filtering (need %d). "
                    "Consider adjusting thresholds (low=%.2f, high=%.2f) or disabling gating.",
                    self._name, len(y_list), MIN_TPI_CYCLES,
                    self._tpi_threshold_low, self._tpi_threshold_high
                )
                return None
            
            # Determine if we use humidity
            # We check the percentage of cycles that had valid humidity data (avg_humidity is not None)
            valid_humidity_cycles = sum(1 for h in x_humidity_list if h is not None)
            use_humidity = (valid_humidity_cycles / len(y_list)) >= 0.70
            
            Y = np.array(y_list)
            
            if use_humidity:
                # Replace None with 0 (mean of centered data if data is missing)
                x_humidity_filled = [h if h is not None else 0.0 for h in x_humidity_list]
                X = np.column_stack([x_loss_list, x_power_list, x_humidity_filled])
            else:
                X = np.column_stack([x_loss_list, x_power_list])

            # Split data: 80% train, 20% val
            n = len(Y)
            split_idx = int(0.8 * n)

            # Ensure we have enough data for validation
            if n - split_idx < 1:
                _LOGGER.warning("%s - Auto TPI: Not enough data for validation (n=%d)", self._name, n)
                return None

            X_train = X[:split_idx]
            Y_train = Y[:split_idx]
            X_val = X[split_idx:]
            Y_val = Y[split_idx:]

            # --- Constrained Optimization Strategy ---
            
            # Helper: Train model with bounds
            # bounds: list of (min, max) tuples
            def train_constrained(X_in, Y_in, initial_guess, bounds):
                def objective(params):
                    return np.sum((Y_in - (X_in @ params))**2)
                
                try:
                    res = minimize(
                        objective,
                        x0=initial_guess,
                        method='L-BFGS-B',
                        bounds=bounds
                    )
                    return res.x, res.fun, res.success
                except Exception as ex:
                    _LOGGER.warning("Auto TPI: Optimization exception: %s", ex)
                    return initial_guess, float('inf'), False

            # Initial guesses (physically reasonable values)
            # alpha ~ 0.5 (2 hours time constant), beta ~ 10 (10K/h max heating), gamma ~ 0
            
            final_coeffs = None
            gamma = None

            if use_humidity:
                # 3 variables: alpha, beta, gamma
                # Bounds:
                # Alpha: 0.001 to 10 (Time constant: 1000h down to 6 min)
                # Beta:  0.001 to 50 (Efficiency: minimal to super powerful)
                # Gamma: -0.5 to 0.5 (Humidity effect: limited range)
                bounds_3var = [(0.001, 10.0), (0.001, 50.0), (-0.5, 0.5)]
                guess_3var = [0.5, 10.0, 0.0]
                
                coeffs, _, success = train_constrained(X_train, Y_train, guess_3var, bounds_3var)
                
                if success:
                    alpha, beta, gamma = coeffs
                    _LOGGER.info("%s - Auto TPI: 3-var optimization success: a=%.4f, b=%.4f, g=%.4f",
                                 self._name, alpha, beta, gamma)
                    final_coeffs = coeffs
                else:
                    _LOGGER.info("%s - Auto TPI: 3-var optimization failed. Retrying with 2 vars.", self._name)
                    use_humidity = False

            if not use_humidity:
                # 2 variables: alpha, beta
                bounds_2var = [(0.001, 10.0), (0.001, 50.0)]
                guess_2var = [0.5, 10.0]
                
                coeffs, _, success = train_constrained(X_train, Y_train, guess_2var, bounds_2var)
                
                alpha, beta = coeffs
                gamma = None
                
                if success:
                    _LOGGER.info("%s - Auto TPI: 2-var optimization success: a=%.4f, b=%.4f",
                                 self._name, alpha, beta)
                else:
                     _LOGGER.warning("%s - Auto TPI: 2-var optimization failed to converge. Using best effort.", self._name)

                final_coeffs = coeffs

            # Re-verify we have valid coeffs (just in case)
            if final_coeffs is None:
                 return None

            alpha = final_coeffs[0]
            beta = final_coeffs[1]
            if alpha <= 0 or beta <= 0:
                 _LOGGER.warning("%s - Auto TPI: Constrained optimization returned non-positive values?! a=%.4f, b=%.4f", self._name, alpha, beta)
                 return None

            # Evaluation on VAL
            Y_pred_val = X_val @ final_coeffs
            ss_res_val = np.sum((Y_val - Y_pred_val)**2)
            ss_tot_val = np.sum((Y_val - np.mean(Y_val))**2)

            if ss_tot_val == 0:
                _LOGGER.warning("%s - Auto TPI: Validation set has no variance (n_val=%d)", self._name, len(Y_val))
                return None

            r_squared_val = 1 - (ss_res_val / ss_tot_val)
            rmse_val = np.sqrt(np.mean((Y_val - Y_pred_val) ** 2))

            # Quality assessment based on Validation R²
            quality = LearningQuality.INSUFFICIENT
            if r_squared_val < 0.3:
                quality = LearningQuality.POOR
            elif r_squared_val < 0.5:
                quality = LearningQuality.FAIR
            elif r_squared_val < 0.7:
                quality = LearningQuality.GOOD
            else:
                quality = LearningQuality.EXCELLENT

            _LOGGER.info("%s - Auto TPI: Model quality on validation: R²=%.3f, RMSE=%.3f K/h, Quality=%s",
                        self._name, r_squared_val, rmse_val, quality.value)

            # REJET si R²_val < 0.1 (relaxed from 0.4 to allow continuous learning with noisy data)
            if r_squared_val < 0.1:
                _LOGGER.warning("%s - Auto TPI: Model signal too weak on validation set (R²_val=%.3f)",
                                self._name, r_squared_val)
                self._learning_quality = LearningQuality.INSUFFICIENT
                return None
            
            self._learning_quality = quality

            # Thermal model calculation
            # alpha is in 1/h (since Y is K/h), so 1/alpha is hours.
            # We convert to seconds for the ThermalModel storage and consistency
            time_constant = (1.0 / alpha) * 3600 if alpha > 0 else 3600
            
            self._thermal_model = ThermalModel(
                heat_loss_coef=alpha,
                heating_efficiency=beta,
                time_constant=time_constant,
                r_squared=r_squared_val,
                rmse=rmse_val,
                n_points=len(self._completed_tpi_cycles),
                humidity_coef=gamma
            )

            # Optimized TPI coefficients calculation
            # k_ext: thermal loss compensation
            # The higher alpha is, the faster the losses are
            # Improved formula: alpha / beta represents the power % needed to compensate 1°C diff
            k_ext = np.clip(alpha / beta, 0.01, 0.20)
            
            # Adaptive TPI Coefficients Logic
            tau_hours = time_constant / 3600.0
            
            # Desired response time (adaptive)
            # Rule: max(0.5, tau * 0.1)
            desired_response_hours = max(0.5, tau_hours * 0.1)
            
            # k_int calculation
            # k_int = 1.0 / (beta * desired_response_hours)
            val_k_int = 1.0 / (beta * desired_response_hours)
            
            # Fast building boost (tau < 2h)
            if tau_hours < 2.0:
                val_k_int *= 1.2
                
            k_int = np.clip(val_k_int, 0.01, 1.0)
            
            self._calculated_params = {
                CONF_TPI_COEF_INT: round(k_int, 3),
                CONF_TPI_COEF_EXT: round(k_ext, 3),
                "confidence": round(r_squared_val, 3),
                "quality": self._learning_quality.value,
                "time_constant_hours": round(tau_hours, 2),
                "desired_response_hours": round(desired_response_hours, 2)
            }
            
            await self.async_save_data()
            
            _LOGGER.info("%s - Auto TPI: TPI params calculated: k_int=%.3f, k_ext=%.3f (tau=%.1fh, resp=%.1fh, conf=%.1f%%)",
                        self._name, k_int, k_ext, tau_hours, desired_response_hours, r_squared_val * 100)
            
            return self._calculated_params

        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Calculation error: %s", self._name, e, exc_info=True)
            return None

    async def async_save_data(self):
        """Save data."""
        await self._hass.async_add_executor_job(self._save_data_sync)

    def _save_data_sync(self):
        """Sync save."""
        try:
            # Compress data before saving
            points_to_save = self._compress_historical_data()
            
            data = {
                "version": STORAGE_VERSION,
                "learning_active": self._learning_active,
                "data": [p.to_dict() for p in points_to_save],
                "completed_tpi_cycles": [c.to_dict() for c in self._completed_tpi_cycles[-MAX_STORED_CYCLES:]],
                "thermal_model": self._thermal_model.to_dict() if self._thermal_model else None,
                "calculated_params": self._calculated_params,
                "learning_quality": self._learning_quality.value,
            }
            os.makedirs(os.path.dirname(self._storage_path), exist_ok=True)
            with open(self._storage_path, 'w') as f:
                json.dump(data, f, indent=2, cls=NumpyEncoder)
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Save error: %s", self._name, e)

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
                    
        except json.JSONDecodeError as e:
            _LOGGER.error("%s - Auto TPI: Corrupted storage file detected at line %d, column %d. " 
                         "Deleting and starting fresh.", self._unique_id, e.lineno, e.colno)
            try:
                os.remove(self._storage_path)
                _LOGGER.info("%s - Auto TPI: Corrupted storage file deleted successfully.", self._unique_id)
            except Exception as delete_error:
                _LOGGER.error("%s - Auto TPI: Failed to delete corrupted file: %s", self._unique_id, delete_error)
            
            # Initialize clean state
            self._data = []
            self._completed_tpi_cycles = []
            self._current_tpi_cycle = None
            self._calculated_params = {}
            self._learning_quality = LearningQuality.INSUFFICIENT
        except Exception as e:
            _LOGGER.error("%s - Auto TPI: Unexpected load error: %s", self._unique_id, e, exc_info=True)
            # Initialize clean state on any other error
            self._data = []
            self._completed_tpi_cycles = []
            self._current_tpi_cycle = None
            self._calculated_params = {}
            self._learning_quality = LearningQuality.INSUFFICIENT

