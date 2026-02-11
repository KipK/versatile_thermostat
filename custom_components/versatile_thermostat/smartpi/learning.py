"""
Smart-PI Learning Modules
Contains DeadTimeEstimator and ABEstimator.
"""
from __future__ import annotations

import logging
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

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

    def update(self, now: float, tin: float, sp: float, u_applied: float, max_on_percent: float = 1.0, is_hysteresis: bool = False) -> None:
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
                _LOGGER.debug(f"DeadTime: Heat Start ignored (Power {u_applied:.2f} < {self.min_power_heat_threshold})")
            
            # 2. Check Min OFF Time
            if allow_start and self.last_stop_time is not None:
                off_duration = now - self.last_stop_time
                if off_duration < self.min_off_time_seconds:
                    allow_start = False
                    _LOGGER.debug(f"DeadTime: Heat Start ignored (OFF duration {off_duration:.0f}s < {self.min_off_time_seconds})")
            
            if allow_start:
                self.heat_start_time = now
                self.heat_start_temp = tin
                self.state = "WAITING_HEAT_RESPONSE"
                _LOGGER.debug(f"DeadTime: State -> WAITING_HEAT_RESPONSE (u={u_applied:.2f}, temp={tin:.3f})")
            else:
                 self.state = "HEATING" # Active but not detecting

        # >0 -> 0 (Cool Start)
        elif self.last_power > 0.01 and u_applied <= 0.01:
            self.last_stop_time = now
            
            if self.last_power < self.min_power_cool_threshold:
                self.state = "COOLING" # Ignore
                _LOGGER.debug(f"DeadTime: Cool Start ignored (Prev Power {self.last_power:.2f} < {self.min_power_cool_threshold})")
            else:
                self.cool_start_time = now
                self.cool_peak_temp = tin
                self.state = "WAITING_COOL_RESPONSE"
                _LOGGER.debug(f"DeadTime: State -> WAITING_COOL_RESPONSE (temp={tin:.3f})")

        # --- State Logic ---
        
        if self.state == "WAITING_HEAT_RESPONSE":
            if self.heat_start_time is not None:
                elapsed = now - self.heat_start_time
                
                # Check Timeout
                if elapsed > self.timeout_seconds:
                    self.state = "HEATING"
                    _LOGGER.debug(f"DeadTime: Heat Timeout ({elapsed:.0f}s)")
                else:
                    delta = tin - self.heat_start_temp
                    if delta >= self.detection_threshold:
                        dt = elapsed
                        self._add_sample_heat(dt)
                        self.state = "HEATING"
                        _LOGGER.info(f"SmartPI: Heat Deadtime detected = {dt:.1f}s")
        
        elif self.state == "WAITING_COOL_RESPONSE":
            if self.cool_start_time is not None:
                elapsed = now - self.cool_start_time
                
                # Check Timeout
                if elapsed > self.timeout_seconds:
                    self.state = "COOLING"
                    _LOGGER.debug(f"DeadTime: Cool Timeout ({elapsed:.0f}s)")
                else:
                    # Peak update
                    if tin > self.cool_peak_temp:
                         self.cool_peak_temp = tin
                    
                    # Drop detection
                    delta = self.cool_peak_temp - tin
                    if delta >= self.detection_threshold:
                        dt = elapsed
                        self._add_sample_cool(dt)
                        self.state = "COOLING"
                        _LOGGER.info(f"SmartPI: Cool Deadtime detected = {dt:.1f}s")
                        
        # Default states if running without detection
        elif u_applied > 0.01 and self.state == "OFF":
             self.state = "HEATING"
        elif u_applied <= 0.01 and self.state != "OFF" and self.state != "WAITING_COOL_RESPONSE":
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
        - If accumulating (11 <= len < 31): Use last 11 samples.
        - If full (len == 31): Use all 31 samples.
        """
        if len(history) < AB_HISTORY_SIZE:
            # Logic: From 11 to 30, we stay in "mode 11" (rolling 11)
            return list(history)[-AB_MIN_SAMPLES:]
        # Mode 31
        return history

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
        window_min: float = 8.0,
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
        y = [p[1] for p in samples_sorted]
        
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
        
        # Check against minimum physical slope if we are in a learning context (implied by amplitude check)
        # If we passed amplitude check but slope is very close to 0, it means it oscillates?
        # A simple check:
        if dt_min_window > 0.5:
            min_abs_slope = DT_DERIVATIVE_MIN_ABS / dt_min_window
            if abs(slope_min) < min_abs_slope:
                return None, "low_slope", len(samples_sorted)
        
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

            # Add to history
            self.b_meas_hist.append(b_meas)

            if len(self.b_meas_hist) < AB_MIN_SAMPLES:
                self.learn_skip_count += 1
                self.learn_last_reason = f"skip: collecting b meas ({len(self.b_meas_hist)}/{AB_MIN_SAMPLES})"
                return

            # Step logic: Select window
            b_window = self._get_window(self.b_meas_hist)

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

            # Add to history
            self.a_meas_hist.append(a_meas)

            if len(self.a_meas_hist) < AB_MIN_SAMPLES:
                self.learn_skip_count += 1
                self.learn_last_reason = f"skip: collecting a meas ({len(self.a_meas_hist)}/{AB_MIN_SAMPLES})"
                return

            # Step logic: Select window
            a_window = self._get_window(self.a_meas_hist)

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
