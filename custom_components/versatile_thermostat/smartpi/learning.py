"""
Smart-PI Learning Modules
Contains DeadTimeEstimator and ABEstimator.
"""
from __future__ import annotations

import logging
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

from .timestamp_utils import convert_monotonic_to_wall_ts, convert_wall_to_monotonic_ts

from .const import (
    AB_HISTORY_SIZE,
    AB_MAD_K,
    AB_MAD_SIGMA_MULT,
    AB_MIN_SAMPLES,
    AB_VAL_TOLERANCE,
    B_STABILITY_MAD_RATIO_MAX,
    DELTA_MIN_OFF,
    DELTA_MIN_ON,
    DT_DERIVATIVE_MIN_ABS,
    U_OFF_MAX,
    U_ON_MIN,
    clamp,
)

_LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class TauReliability:
    """Result of tau (time constant) reliability check."""
    reliable: bool
    tau_min: float  # minutes (min of candidates used)


class DeadTimeEstimator:
    """
    Simplified Dead Time Estimator for Smart-PI.
    Based on Finite State Machine detecting sharp power transitions.
    """

    def __init__(self):
        self.deadtime_heat_s: float | None = None
        self.deadtime_cool_s: float | None = None
        self.deadtime_heat_reliable: bool = False
        self.deadtime_cool_reliable: bool = False
        
        # Configuration
        self.min_off_time_seconds = 600.0
        self.min_power_heat_threshold = 0.80
        self.min_power_cool_threshold = 0.80
        self.detection_threshold = 0.05
        self.timeout_seconds = 14400.0  # 4 hours timeout for slow systems with inertia
        
        # State
        self.state = "OFF"  # OFF, HEATING, COOLING, WAITING_HEAT_RESPONSE, WAITING_COOL_RESPONSE
        self.last_power = 0.0
        self.last_stop_time: float | None = None
        
        # Detection ephemeral data
        self.heat_start_time: float | None = None
        self.heat_start_temp: float | None = None
        
        self.cool_start_time: float | None = None
        self.cool_peak_temp: float | None = None
        
        # History for averaging
        self._history_heat = deque(maxlen=6)
        self._history_cool = deque(maxlen=6)
        
        # History for external access (SmartPI learning)
        self._tin_history: Deque[Tuple[float, float]] = deque(maxlen=300)

    @property
    def tin_history(self) -> Deque[Tuple[float, float]]:
        """Temperature history for learning window slope calculation."""
        return self._tin_history

    def reset(self):
        """Reset estimator state."""
        self.deadtime_heat_s = None
        self.deadtime_cool_s = None
        self.deadtime_heat_reliable = False
        self.deadtime_cool_reliable = False
        self.state = "OFF"
        self.last_power = 0.0
        self.last_stop_time = None
        self._history_heat.clear()
        self._history_cool.clear()
        self._tin_history.clear()

    def update(  # pylint: disable=unused-argument
        self, now: float, tin: float, sp: float, u_applied: float, max_on_percent: float = 1.0, is_hysteresis: bool = False
    ) -> None:
        """
        Update state machine with new measures.
        """
        self._tin_history.append((now, tin))
        
        # --- Power Transition Detection ---
        
        # 0 -> >0 (Heat Start)
        if self.last_power <= 0.01 and u_applied > 0.01:
            allow_start = True
            
            # 1. Check Power Level
            if u_applied < self.min_power_heat_threshold:
                allow_start = False
                _LOGGER.debug("DeadTime: Heat Start ignored (Power %.2f < %s)", u_applied, self.min_power_heat_threshold)
            
            # 2. Check Min OFF Time
            if allow_start and self.last_stop_time is not None:
                off_duration = now - self.last_stop_time
                if off_duration < self.min_off_time_seconds:
                    allow_start = False
                    _LOGGER.debug("DeadTime: Heat Start ignored (OFF duration %.0fs < %s)", off_duration, self.min_off_time_seconds)
            
            if allow_start:
                self.heat_start_time = now
                self.heat_start_temp = tin
                self.state = "WAITING_HEAT_RESPONSE"
                _LOGGER.debug("DeadTime: State -> WAITING_HEAT_RESPONSE (u=%.2f, temp=%.3f)", u_applied, tin)
            else:
                self.state = "HEATING"  # Active but not detecting

        # >0 -> 0 (Cool Start)
        elif self.last_power > 0.01 and u_applied <= 0.01:
            self.last_stop_time = now
            
            if self.last_power < self.min_power_cool_threshold:
                self.state = "COOLING" # Ignore
                _LOGGER.debug("DeadTime: Cool Start ignored (Prev Power %.2f < %s)", self.last_power, self.min_power_cool_threshold)
            else:
                self.cool_start_time = now
                self.cool_peak_temp = tin
                self.state = "WAITING_COOL_RESPONSE"
                _LOGGER.debug("DeadTime: State -> WAITING_COOL_RESPONSE (temp=%.3f)", tin)

        # --- State Logic ---
        
        # Abort condition (3.B): if power state reverses while waiting
        # This means the setpoint changed and we shouldn't wait for a response anymore
        if self.state == "WAITING_HEAT_RESPONSE" and u_applied <= 0.01:
            _LOGGER.debug("DeadTime: Aborting %s because power dropped to %.2f", self.state, u_applied)
            self.state = "OFF"
            self.heat_start_time = None
        elif self.state == "WAITING_COOL_RESPONSE" and u_applied > 0.01:
            _LOGGER.debug("DeadTime: Aborting %s because power rose to %.2f", self.state, u_applied)
            self.state = "HEATING"
            self.cool_start_time = None
        
        if self.state == "WAITING_HEAT_RESPONSE":
            if self.heat_start_time is not None:
                elapsed = now - self.heat_start_time
                
                # Check Timeout
                if elapsed > self.timeout_seconds:
                    self.state = "HEATING"
                    _LOGGER.debug("DeadTime: Heat Timeout (%.0fs)", elapsed)
                else:
                    delta = tin - self.heat_start_temp
                    if delta >= self.detection_threshold:
                        # 3.A Look back for inflection point
                        inflection_time = now
                        for t_hist, v_hist in reversed(self._tin_history):
                            if t_hist < self.heat_start_time:
                                break
                            # The temperature started rising here
                            if v_hist <= self.heat_start_temp + 0.01:
                                inflection_time = t_hist
                                break
                            
                        # True deadtime is from heat_start_time to inflection_time
                        dt = max(0.0, inflection_time - self.heat_start_time)
                        
                        self._add_sample_heat(dt)
                        self.state = "HEATING"
                        _LOGGER.info("SmartPI: Heat Deadtime detected = %.1fs (ascension delayed by %.1fs)", dt, now - inflection_time)
        
        elif self.state == "WAITING_COOL_RESPONSE":
            if self.cool_start_time is not None:
                elapsed = now - self.cool_start_time
                
                # Check Timeout
                if elapsed > self.timeout_seconds:
                    self.state = "COOLING"
                    _LOGGER.debug("DeadTime: Cool Timeout (%.0fs)", elapsed)
                else:
                    # Peak update
                    if tin > self.cool_peak_temp:
                        self.cool_peak_temp = tin
                    
                    # Drop detection
                    delta = self.cool_peak_temp - tin
                    if delta >= self.detection_threshold:
                        # 3.A Look back for inflection point
                        inflection_time = now
                        for t_hist, v_hist in reversed(self._tin_history):
                            if t_hist < self.cool_start_time:
                                break
                            # The temperature started dropping here
                            if v_hist >= self.cool_peak_temp - 0.01:
                                inflection_time = t_hist
                                break
                                
                        dt = max(0.0, inflection_time - self.cool_start_time)

                        self._add_sample_cool(dt)
                        self.state = "COOLING"
                        _LOGGER.info("SmartPI: Cool Deadtime detected = %.1fs (drop delayed by %.1fs)", dt, now - inflection_time)
                        
        # Default states if running without detection
        elif u_applied > 0.01 and self.state == "OFF":
            self.state = "HEATING"
        elif u_applied <= 0.01:
            if self.state != "OFF" and self.state != "WAITING_COOL_RESPONSE" and self.state != "COOLING":
                self.state = "OFF"

        self.last_power = u_applied

    def _add_sample_heat(self, dt: float):
        self._history_heat.append(dt)
        self.deadtime_heat_s = statistics.mean(self._history_heat)
        self.deadtime_heat_reliable = len(self._history_heat) >= 1

    def _add_sample_cool(self, dt: float):
        self._history_cool.append(dt)
        self.deadtime_cool_s = statistics.mean(self._history_cool)
        self.deadtime_cool_reliable = len(self._history_cool) >= 1

    def save_state(self) -> dict:
        """Save state for persistence."""
        return {
            "deadtime_heat_s": self.deadtime_heat_s,
            "deadtime_cool_s": self.deadtime_cool_s,
            "deadtime_heat_reliable": self.deadtime_heat_reliable,
            "deadtime_cool_reliable": self.deadtime_cool_reliable,
            "history_heat": list(self._history_heat),
            "history_cool": list(self._history_cool),
            "state": self.state,
            "last_stop_time": convert_monotonic_to_wall_ts(self.last_stop_time),
            "heat_start_time": convert_monotonic_to_wall_ts(self.heat_start_time),
            "heat_start_temp": self.heat_start_temp,
            "cool_start_time": convert_monotonic_to_wall_ts(self.cool_start_time),
            "cool_peak_temp": self.cool_peak_temp,
            "tin_history": [(convert_monotonic_to_wall_ts(t), v) for t, v in self._tin_history],
        }

    def load_state(self, state: dict) -> None:
        """Restore state."""
        if not state:
            return
        self.deadtime_heat_s = state.get("deadtime_heat_s")
        self.deadtime_cool_s = state.get("deadtime_cool_s")
        self.deadtime_heat_reliable = bool(state.get("deadtime_heat_reliable", False))
        self.deadtime_cool_reliable = bool(state.get("deadtime_cool_reliable", False))
        
        hh = state.get("history_heat", [])
        self._history_heat = deque(hh, maxlen=6)
        hc = state.get("history_cool", [])
        self._history_cool = deque(hc, maxlen=6)

        # Restore detection state
        self.state = state.get("state", "OFF")
        self.last_stop_time = convert_wall_to_monotonic_ts(state.get("last_stop_time"))
        self.heat_start_time = convert_wall_to_monotonic_ts(state.get("heat_start_time"))
        self.heat_start_temp = state.get("heat_start_temp")
        self.cool_start_time = convert_wall_to_monotonic_ts(state.get("cool_start_time"))
        self.cool_peak_temp = state.get("cool_peak_temp")

        # Restore tin_history
        th = state.get("tin_history", [])
        self._tin_history.clear()
        for t_wall, v in th:
            t_mono = convert_wall_to_monotonic_ts(t_wall)
            if t_mono is not None:
                self._tin_history.append((t_mono, v))


class ABEstimator:
    """
    Robust Online Estimator for a and b using Continuous approach:
    
    Model: dT/dt = a*u - b*(T_int - T_ext)
    
    1. Theil-Sen is used for robust dT/dt calculation over a sliding window.
    2. Median + MAD is used for robust a and b parameter estimation from history.
    """

    def __init__(self, a_init: float = 0.0005, b_init: float = 0.0010):
        self.A_INIT = a_init
        self.B_INIT = b_init
        
        self.a = a_init
        self.b = b_init

        # Robust bounds
        self.A_MIN: float = 1e-5
        self.A_MAX: float = 0.15
        self.B_MIN: float = 1e-5
        self.B_MAX: float = 0.05

        # --- Strategy: Median + MAD (Robust) ---
        # Raw measurement history
        self.a_meas_hist: Deque[float] = deque(maxlen=AB_HISTORY_SIZE) # Keep last 31
        self.b_meas_hist: Deque[float] = deque(maxlen=AB_HISTORY_SIZE)

        # Stability tracking for a and b (tau) - used for reliability check
        self._b_hat_hist: Deque[float] = deque(maxlen=20)
        self._a_hat_hist: Deque[float] = deque(maxlen=20)
        
        # Counters
        self.learn_ok_count = 0  # Total successful updates
        self.learn_ok_count_a = 0
        self.learn_ok_count_b = 0
        self.learn_skip_count = 0
        self.learn_last_reason: Optional[str] = "init"

        # Diagnostics
        self.diag_dTdt_method: str = "init"

        self.diag_b_mad_over_med: Optional[float] = None
        self.diag_a_mad_over_med: Optional[float] = None

    def reset(self) -> None:
        """Reset learned parameters and history to initial values."""
        self.a = self.A_INIT
        self.b = self.B_INIT
        self.learn_ok_count = 0
        self.learn_ok_count_a = 0
        self.learn_ok_count_b = 0
        self.learn_skip_count = 0
        self.learn_last_reason = "reset"
        self._b_hat_hist.clear()
        self._a_hat_hist.clear()
        self.a_meas_hist.clear()
        self.b_meas_hist.clear()

        self.diag_b_mad_over_med = None
        self.diag_a_mad_over_med = None

    # ---------- Robust helpers (Static) ----------

    @staticmethod
    def _mad(values):
        if len(values) < 2:
            return None
        med = statistics.median(values)
        try:
            return statistics.median(abs(v - med) for v in values)
        except statistics.StatisticsError:
            return None

    def _get_window(self, history: Deque[float]):
        """
        Get the learning window subset according to step logic.
        - The window size grows progressively up to the max size.
        """
        return list(history)

    @staticmethod
    def _theil_sen_slope(x: list[float], y: list[float]) -> float | None:
        """Robust slope estimation using Theil-Sen estimator (Median of slopes)."""
        n = len(x)
        if n < 2:
            return None
        
        slopes = []
        # O(N^2) but N is small (ticket 6-30 points)
        for i in range(n):
            for j in range(i + 1, n):
                dx = x[j] - x[i]
                if dx != 0:
                    slope = (y[j] - y[i]) / dx
                    slopes.append(slope)
        
        if not slopes:
            return None
            
        return statistics.median(slopes)

    @staticmethod
    def robust_dTdt_per_min(
        samples: list[Tuple[float, float]],
        _window_min: float = 8.0,
        *,
        trim_start_frac: float = 0.0,
        trim_end_frac: float = 0.0,
    ) -> Tuple[float | None, str, int]:
        """
        Calculate robust dT/dt (°C/min) given a list of (t_sec, T_int).
        
        Args:
            samples: list of (timestamp, value)
            window_min: desired window size in minutes (check consistence)
            trim_start_frac: fraction of time window to ignore at start (0.0-0.5)
            trim_end_frac: fraction of time window to ignore at end (0.0-0.5)

        Returns:
            (slope_per_min, method_used, n_points)
            slope_per_min is None if calculation impossible
        """
        if not samples or len(samples) < 6:
            return None, "insufficient_samples", len(samples)
            
        # Unzip
        # Sort by time just in case
        samples_sorted = sorted(samples, key=lambda p: p[0])
        
        # Optional trimming by time span (safety-clamped)
        if trim_start_frac > 0.0 or trim_end_frac > 0.0:
            t_start = samples_sorted[0][0]
            t_end = samples_sorted[-1][0]
            span = t_end - t_start
            
            # Clamp fractions
            tf_start = clamp(trim_start_frac, 0.0, 0.45)
            tf_end = clamp(trim_end_frac, 0.0, 0.45)
            
            t_valid_start = t_start + span * tf_start
            t_valid_end = t_end - span * tf_end
            
            # Filter samples
            samples_trimmed = [p for p in samples_sorted if t_valid_start <= p[0] <= t_valid_end]
            
            # Check if we still have enough points
            if len(samples_trimmed) < 4:
                # Trimming left too few points -> abort robust calc
                # (Caller might fallback, or getting None is the intended "skip")
                return None, "insufficient_samples_trimmed", len(samples_trimmed)
                
            samples_sorted = samples_trimmed

        x = [p[0] for p in samples_sorted]
        y_raw = [p[1] for p in samples_sorted]
        
        # 0. Light EMA Smoothing to counter sensor quantization (staircase effect)
        # alpha=0.3 provides a gentle smoothing without adding too much phase lag
        alpha = 0.3
        y = []
        current_ema = y_raw[0]
        for val in y_raw:
            current_ema = alpha * val + (1 - alpha) * current_ema
            y.append(current_ema)
        
        # 1. Amplitude check
        amp = max(y) - min(y)
        if amp < DT_DERIVATIVE_MIN_ABS:
            return None, "low_amplitude", len(samples_sorted)

        # 2. Minimum slope magnitude check (prevent noise/flat learning)
        # We need a rough estimate of the window duration
        dt_min_window = (x[-1] - x[0]) / 60.0

        # 3. Theil-Sen
        slope_sec = ABEstimator._theil_sen_slope(x, y)
        if slope_sec is None:
            return None, "theil_sen_fail", len(samples_sorted)
             
        slope_min = slope_sec * 60.0
        
        # Clamp result to physically reasonable values for HVAC (-0.35 to +0.35 C/min)
        # This prevents wild values from exploding the estimator
        slope_min = clamp(slope_min, -0.35, 0.35)
        
        return slope_min, "theil_sen", len(samples_sorted)

    # ---------- Main learning ----------

    def learn(
        self,
        dT_int_per_min: float,
        u: float,
        t_int: float,
        t_ext: float,
        *,
        max_abs_dT_per_min: float = 0.35,
    ) -> None:
        """
        Update (a,b) using Median + MAD approach.

        Model: dT/dt = a*u - b*(T_int - T_ext)
        """
        dTdt = float(dT_int_per_min)
        delta = float(t_int - t_ext)

        # 1. Reject gross physics outliers
        if abs(dTdt) > max_abs_dT_per_min:
            self.learn_skip_count += 1
            self.learn_last_reason = "skip: slope outlier"
            return

        # ---------- OFF phase: learn b ----------
        # dT/dt = -b * delta  =>  b = -dT/dt / delta
        if u < U_OFF_MAX and abs(delta) >= DELTA_MIN_OFF:

            b_meas = -dTdt / delta
            if b_meas <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: b_meas <= 0"
                return

            # Simulate adding to see the window
            temp_history = list(self.b_meas_hist)
            temp_history.append(b_meas)

            if len(temp_history) < AB_MIN_SAMPLES:
                self.b_meas_hist.append(b_meas)
                self.learn_skip_count += 1
                self.learn_last_reason = f"skip: collecting b meas ({len(self.b_meas_hist)}/{AB_MIN_SAMPLES})"
                return

            # Step logic: Select temporary window
            b_window = temp_history[-AB_HISTORY_SIZE:] if len(temp_history) > AB_HISTORY_SIZE else temp_history

            # Median + MAD outlier rejection
            med_b = statistics.median(b_window)
            mad_b = self._mad(b_window)

            if mad_b is not None and mad_b > AB_VAL_TOLERANCE:
                self.diag_b_mad_over_med = mad_b / (abs(med_b) + 1e-9)
                sigma_b = AB_MAD_K * mad_b
                if abs(b_meas - med_b) > AB_MAD_SIGMA_MULT * sigma_b:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: b_meas outlier"
                    return
            else:
                self.diag_b_mad_over_med = 0.0

            # It's an acceptable value, add it permanently
            self.b_meas_hist.append(b_meas)

            new_b = med_b  # Use median directly
            new_b = clamp(new_b, self.B_MIN, self.B_MAX)

            self.b = new_b
            self._b_hat_hist.append(new_b)
            self.learn_ok_count += 1
            self.learn_ok_count_b += 1
            self.learn_last_reason = "learned b (Median)"
            return

        # ---------- ON phase: learn a ----------
        # dT/dt = a*u - b*delta  =>  a = (dT/dt + b*delta) / u
        if u > U_ON_MIN and abs(delta) >= DELTA_MIN_ON:
            a_meas = (dTdt + self.b * delta) / u
            if a_meas <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: a_meas <= 0"
                return

            # Simulate adding to see the window
            temp_history = list(self.a_meas_hist)
            temp_history.append(a_meas)

            if len(temp_history) < AB_MIN_SAMPLES:
                self.a_meas_hist.append(a_meas)
                self.learn_skip_count += 1
                self.learn_last_reason = f"skip: collecting a meas ({len(self.a_meas_hist)}/{AB_MIN_SAMPLES})"
                return

            # Step logic: Select temporary window
            a_window = temp_history[-AB_HISTORY_SIZE:] if len(temp_history) > AB_HISTORY_SIZE else temp_history

            # Median + MAD outlier rejection
            med_a = statistics.median(a_window)
            mad_a = self._mad(a_window)

            if mad_a is not None and mad_a > AB_VAL_TOLERANCE:
                self.diag_a_mad_over_med = mad_a / (abs(med_a) + 1e-9)
                sigma_a = AB_MAD_K * mad_a
                if abs(a_meas - med_a) > AB_MAD_SIGMA_MULT * sigma_a:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: a_meas outlier"
                    return
            else:
                self.diag_a_mad_over_med = 0.0

            # It's an acceptable value, add it permanently
            self.a_meas_hist.append(a_meas)

            new_a = med_a  # Use median directly
            new_a = clamp(new_a, self.A_MIN, self.A_MAX)

            self.a = new_a
            self._a_hat_hist.append(new_a)
            self.learn_ok_count += 1
            self.learn_ok_count_a += 1
            self.learn_last_reason = "learned a (Median)"
            return

        self.learn_skip_count += 1
        self.learn_last_reason = "skip: low excitation"

    def tau_reliability(self) -> TauReliability:
        """
        Check if tau (1/b) is statistically stable and within bounds.
        """
        # Enough updates?
        if self.learn_ok_count_b < 5:
            return TauReliability(reliable=False, tau_min=9999.0)

        if len(self._b_hat_hist) < 5:
            return TauReliability(reliable=False, tau_min=9999.0)

        med_b = statistics.median(self._b_hat_hist)
        mad_b = self._mad(self._b_hat_hist)

        if mad_b is None or med_b <= 0:
            return TauReliability(reliable=False, tau_min=9999.0)

        # Stability check: Relative dispersion of b estimates (0.60 threshold)
        if (mad_b / med_b) > B_STABILITY_MAD_RATIO_MAX:
            return TauReliability(reliable=False, tau_min=9999.0)

        # Value bounds check
        if med_b < self.B_MIN or med_b > self.B_MAX:
            return TauReliability(reliable=False, tau_min=9999.0)

        tau = 1.0 / med_b
        # tau bounds are implicitly enforced by B_MIN/B_MAX
        return TauReliability(reliable=True, tau_min=tau)

    def save_state(self) -> dict:
        """Save state for persistence."""
        return {
            "a": self.a,
            "b": self.b,
            "learn_ok_count": self.learn_ok_count,
            "learn_ok_count_a": self.learn_ok_count_a,
            "learn_ok_count_b": self.learn_ok_count_b,
            "learn_skip_count": self.learn_skip_count,
            "a_meas_hist": list(self.a_meas_hist),
            "b_meas_hist": list(self.b_meas_hist),
            "a_hat_hist": list(self._a_hat_hist),
            "b_hat_hist": list(self._b_hat_hist),
            "diag_a_mad_over_med": self.diag_a_mad_over_med,
            "diag_b_mad_over_med": self.diag_b_mad_over_med,
            "learn_last_reason": self.learn_last_reason,
            "diag_dTdt_method": self.diag_dTdt_method,
        }

    def load_state(self, state: dict) -> None:
        """Restore state."""
        if not state:
            return
        self.a = float(state.get("a", self.A_INIT))
        self.b = float(state.get("b", self.B_INIT))
        self.learn_ok_count = int(state.get("learn_ok_count", 0))
        self.learn_ok_count_a = int(state.get("learn_ok_count_a", 0))
        self.learn_ok_count_b = int(state.get("learn_ok_count_b", 0))
        self.learn_skip_count = int(state.get("learn_skip_count", 0))
        
        amh = state.get("a_meas_hist", [])
        self.a_meas_hist = deque(amh, maxlen=AB_HISTORY_SIZE)
        bmh = state.get("b_meas_hist", [])
        self.b_meas_hist = deque(bmh, maxlen=AB_HISTORY_SIZE)
        
        # Restore filtered histories for tau reliability
        a_hat = state.get("a_hat_hist", [])
        self._a_hat_hist = deque(a_hat, maxlen=20)
        b_hat = state.get("b_hat_hist", [])
        self._b_hat_hist = deque(b_hat, maxlen=20)

        # Restore diagnostics
        self.diag_a_mad_over_med = state.get("diag_a_mad_over_med")
        self.diag_b_mad_over_med = state.get("diag_b_mad_over_med")
        self.learn_last_reason = state.get("learn_last_reason", "init")
        self.diag_dTdt_method = state.get("diag_dTdt_method", "init")
