"""
This module implements the AutoPI algorithm (Robuste Estimator + PI Controller)
for Versatile Thermostat.

Based on regul6.py:
- Model learned online (Robust Estimator): dT_int (°C/min) ≈ a * u - b * (T_int - T_ext)
- Control: u = clamp(u_ff + u_PI, 0..1)
  - u_ff compensates for thermal losses (feed-forward based on T_ext)
  - u_PI corrects error (T_set - T_int) with PI + safeguards (Anti-Windup, Gain Scheduling)
"""
import logging
import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Tuple, Deque, List

from .vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL, VThermHvacMode_OFF, VThermHvacMode_HEAT


_LOGGER = logging.getLogger(__name__)

# Constant from regul6.py
A_INIT = 0.00050
B_INIT = 0.00100
A_MIN, A_MAX = 1e-5, 0.05
B_MIN, B_MAX = 1e-6, 0.02
ALPHA_A = 0.12
ALPHA_B = 0.10
AB_MEAS_WINDOW_A = 24
AB_MEAS_WINDOW_B = 30
AB_OUTLIER_K = 4.0
AB_HUBER_C = 1.5
AB_MIN_SAMPLES = 6
B_STABILITY_WINDOW = 20
B_CV_MAX = 0.35
LEARN_OK_MIN = 6

KP_MIN = 0.2
KP_MAX = 1.5
KI_MIN = 0.0005
KI_MAX = 0.25

TAU_MIN_PLAUSIBLE_MIN = 30
TAU_MAX_PLAUSIBLE_MIN = 3000

# Tuning constants
LAMBDA_FACTOR = 2.0
LAMBDA_MIN_MIN = 5.0
GAINS_SMOOTH_ALPHA = 0.25
GAIN_SCHED_ENABLE = True
GAIN_SCHED_E1_C = 0.10
GAIN_SCHED_E2_C = 0.40
GAIN_SCHED_KP_MIN_FACTOR = 0.45
GAIN_SCHED_KI_EXP = 2.0
LAMBDA_SCHED_ENABLE = True
LAMBDA_SCHED_NEAR_FACTOR = 2.2
AW_BACKCALC_ENABLE = True
AW_BACKCALC_GAIN = 0.40
INTEGRAL_LEAK = 0.985
SETPOINT_BUMPLESS_MAX_DU = 0.12

# Dead time learning constants
DEADTIME_INIT_S = 180
DEADTIME_MIN_S = 60
DEADTIME_MAX_S = 1800  # 30 minutes max
DEADTIME_SAMPLES_MAX = 40
DEAD_DT_MIN_C = 0.02  # Minimum temperature change for deadtime detection
DEAD_DTPM_TH = 0.003  # Slope threshold (°C/min) to consider heating response started
DEADTIME_LEARN_E_MIN_C = 0.20  # Minimum |filtered error| to allow learning
DEADTIME_LEARN_U_MIN = 0.25  # Minimum u_cycle for significant heating


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value to the interval [lo, hi]."""
    return max(lo, min(hi, x))


def median_and_mad(values: List[float]) -> Tuple[float, float]:
    """Return (median, MAD). MAD = median(|x - median(x)|)."""
    if not values:
        raise ValueError("values empty")
    m = statistics.median(values)
    abs_dev = [abs(x - m) for x in values]
    mad = statistics.median(abs_dev) if abs_dev else 0.0
    return m, mad


def robust_huber_location(
    values: Deque[float],
    outlier_k: float = AB_OUTLIER_K,
    huber_c: float = AB_HUBER_C,
) -> Tuple[float, dict]:
    """Estimate a robust location (center)."""
    vals = list(values)
    if len(vals) == 0:
        return 0.0, {"scale": 0.0}

    m, mad = median_and_mad(vals)
    s = 1.4826 * mad
    if s <= 1e-12:
        return m, {"scale": s}

    thr = outlier_k * s
    kept = [x for x in vals if abs(x - m) <= thr]
    if not kept:
        return m, {"scale": s}

    mu = statistics.mean(kept)
    c = max(1e-9, huber_c * s)
    for _ in range(3):
        num = 0.0
        den = 0.0
        for x in kept:
            r = x - mu
            ar = abs(r)
            w = 1.0 if ar <= c else (c / ar)
            num += w * x
            den += w
        if den <= 1e-12:
            break
        mu_new = num / den
        if abs(mu_new - mu) < 1e-9:
            mu = mu_new
            break
        mu = mu_new

    return mu, {"scale": s, "median": m}


@dataclass
class TauReliability:
    reliable: bool
    reason: str
    tau_min: float


@dataclass
class UHistory:
    """Buffer (timestamp, u_applied) to retrieve delayed u."""
    buf: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=2000))

    def push(self, ts: float, u_applied: float) -> None:
        self.buf.append((ts, float(u_applied)))

    def get_delayed(self, ts: float, delay_s: int) -> float:
        if not self.buf:
            return 0.0
        target = ts - delay_s
        # If buffer is empty or not enough history, return latest or 0
        if not self.buf:
             return 0.0
        
        # Simple search
        best_u = self.buf[0][1]
        best_dt = abs(self.buf[0][0] - target)
        for t, u in reversed(self.buf):
            dt = abs(t - target)
            if dt < best_dt:
                best_dt = dt
                best_u = u
            if t < target and best_dt < 1.0:
                 break
        return float(best_u)


@dataclass
class DeadtimeEstimator:
    """Estimates thermal dead time using robust median of samples."""
    samples: Deque[int] = field(default_factory=lambda: deque(maxlen=DEADTIME_SAMPLES_MAX))
    current_estimate: int = DEADTIME_INIT_S
    
    def add_sample(self, delay_s: int) -> None:
        """Add a new dead time sample."""
        if DEADTIME_MIN_S <= delay_s <= DEADTIME_MAX_S:
            self.samples.append(delay_s)
            self._update_estimate()
            
    def _update_estimate(self) -> None:
        """Update the estimate using median of samples."""
        if len(self.samples) >= 1:
            self.current_estimate = int(statistics.median(self.samples))
            self.current_estimate = clamp(self.current_estimate, DEADTIME_MIN_S, DEADTIME_MAX_S)
            
    def get_estimate(self) -> int:
        """Return current dead time estimate in seconds."""
        return self.current_estimate


@dataclass
class ABEstimator:
    """Robust estimation of a and b."""
    a: float = A_INIT
    b: float = B_INIT

    a_meas_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=AB_MEAS_WINDOW_A))
    b_meas_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=AB_MEAS_WINDOW_B))
    b_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=B_STABILITY_WINDOW))

    learn_ok_count: int = 0
    learn_last_reason: str = "init"
    
    # Internal diagnostics
    last_a_meas: Optional[float] = None
    last_b_meas: Optional[float] = None
    last_a_robust: Optional[float] = None
    last_b_robust: Optional[float] = None
    outliers_a: int = 0
    outliers_b: int = 0

    def learn(
        self,
        u_eff: float,
        t_in_prev: float,
        t_in_now: float,
        t_out_prev: float,
        dt_min: float,
    ) -> Tuple[bool, str, float]:
        if dt_min <= 1e-6:
            self.learn_last_reason = "skip:dt_small"
            return False, self.learn_last_reason, 0.0

        dT = t_in_now - t_in_prev
        dTpm = dT / dt_min
        
        # Thresholds mostly from regul6.py
        MIN_DTPM_FOR_LEARN = 0.003
        
        if abs(dTpm) < MIN_DTPM_FOR_LEARN:
            self.learn_last_reason = "skip:dTpm_low"
            return False, self.learn_last_reason, dTpm
            
        delta = t_in_prev - t_out_prev
        
        # Learn b (cooling)
        if u_eff < 0.05:
            if abs(delta) < 0.5:
                self.learn_last_reason = "skip:b:delta_temp_low"
                return False, self.learn_last_reason, dTpm
            b_meas = -dTpm / delta
            self.last_b_meas = b_meas
            if b_meas <= 0:
                self.learn_last_reason = "skip:b:neg"
                return False, self.learn_last_reason, dTpm
                
            self.b_meas_hist.append(b_meas)
            
            if len(self.b_meas_hist) < AB_MIN_SAMPLES:
                 self.b = clamp((1 - ALPHA_B) * self.b + ALPHA_B * b_meas, B_MIN, B_MAX)
                 self.b_hist.append(self.b)
                 self.learn_ok_count += 1
                 self.learn_last_reason = "update:b(warmup)"
                 return True, self.learn_last_reason, dTpm
                 
            b_robust, dbg = robust_huber_location(self.b_meas_hist)
            self.last_b_robust = b_robust
            
            # Outlier check
            m = dbg.get("median", b_robust)
            s = dbg.get("scale", 0.0)
            if s > 1e-12 and abs(b_meas - m) > AB_OUTLIER_K * s:
                self.outliers_b += 1
                self.learn_last_reason = "skip:b:outlier"
                return False, self.learn_last_reason, dTpm
                
            self.b = clamp((1 - ALPHA_B) * self.b + ALPHA_B * b_robust, B_MIN, B_MAX)
            self.b_hist.append(self.b)
            self.learn_ok_count += 1
            self.learn_last_reason = "update:b(robust)"
            return True, self.learn_last_reason, dTpm
            
        # Learn a (heating)
        if u_eff > 0.20:
             a_meas = (dTpm + self.b * delta) / max(u_eff, 1e-6)
             self.last_a_meas = a_meas
             if a_meas <= 0:
                 self.learn_last_reason = "skip:a:neg"
                 return False, self.learn_last_reason, dTpm
                 
             self.a_meas_hist.append(a_meas)
             
             if len(self.a_meas_hist) < AB_MIN_SAMPLES:
                 self.a = clamp((1 - ALPHA_A) * self.a + ALPHA_A * a_meas, A_MIN, A_MAX)
                 self.learn_ok_count += 1
                 self.learn_last_reason = "update:a(warmup)"
                 return True, self.learn_last_reason, dTpm
                 
             a_robust, dbg = robust_huber_location(self.a_meas_hist)
             self.last_a_robust = a_robust
             
             m = dbg.get("median", a_robust)
             s = dbg.get("scale", 0.0)
             if s > 1e-12 and abs(a_meas - m) > AB_OUTLIER_K * s:
                 self.outliers_a += 1
                 self.learn_last_reason = "skip:a:outlier"
                 return False, self.learn_last_reason, dTpm
                 
             self.a = clamp((1 - ALPHA_A) * self.a + ALPHA_A * a_robust, A_MIN, A_MAX)
             self.learn_ok_count += 1
             self.learn_last_reason = "update:a(robust)"
             return True, self.learn_last_reason, dTpm
             
        self.learn_last_reason = "skip:u_gray_zone"
        return False, self.learn_last_reason, dTpm

    def tau_reliability(self) -> TauReliability:
        """Judge reliability of tau=1/b."""
        tau = 1.0 / max(self.b, B_MIN)
        if self.learn_ok_count < LEARN_OK_MIN:
             return TauReliability(False, f"cnt<{LEARN_OK_MIN}", tau)
        if not (TAU_MIN_PLAUSIBLE_MIN <= tau <= TAU_MAX_PLAUSIBLE_MIN):
             return TauReliability(False, "range", tau)
        if len(self.b_hist) < max(6, B_STABILITY_WINDOW // 2):
             return TauReliability(False, "hist", tau)
             
        mean_b = statistics.mean(self.b_hist)
        if mean_b <= 0:
             return TauReliability(False, "b<=0", tau)
        std_b = statistics.pstdev(self.b_hist)
        cv = std_b / mean_b
        if cv > B_CV_MAX:
             return TauReliability(False, f"unstable(cv={cv:.2f})", tau)
        return TauReliability(True, "ok", tau)


class AutoPI:
    """
    Auto-adaptive PI controller + external feed-forward (Robust version).
    """

    def __init__(
        self,
        cycle_min: int,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: float = None,
        # Tuning parameters
        deadband_c: float = 0.05,
        min_useful: float = 0.05,
        max_step_per_min: float = 0.10,
        aggressiveness: float = 0.5,
        # Persistence data
        saved_state: Optional[Dict[str, Any]] = None
    ):
        self._name = name
        self._cycle_min = cycle_min
        self._minimal_activation_delay = minimal_activation_delay
        self._minimal_deactivation_delay = minimal_deactivation_delay
        self._max_on_percent = max_on_percent

        # Tuning
        self.deadband_c = deadband_c
        self.min_useful = min_useful
        self.max_step_per_min = max_step_per_min
        self.aggressiveness = aggressiveness

        # Robust Estimator
        self.est = ABEstimator()
        
        # Dead time management
        self.u_history = UHistory()
        self.deadtime_estimator = DeadtimeEstimator()
        
        # Actuator tracking for dead time learning and integrator hold
        self._pending_on_front_ts: Optional[float] = None
        self._last_actuator_change_ts: float = 0.0
        self._actuator_is_on: bool = False
        self._last_u_cycle: float = 0.0  # u_cycle at last ON front
        
        # Time tracking for dt_min calculation
        self._last_calculate_ts: float = 0.0
        self._boot_ts: float = time.time()
        self._last_setpoint_change_ts: float = 0.0
        
        # Temperature tracking for dead time learning
        self._prev_temp: Optional[float] = None
        self._prev_temp_ts: Optional[float] = None
        
        # PI State
        self.integral: float = 0.0
        self.u_prev: float = 0.0
        self._prev_error: float = 0.0
        self.e_filt: Optional[float] = None
        self.e_prev: Optional[float] = None
        
        # Current gains
        self.Kp: float = 0.8  # Will be smoothed
        self.Ki: float = 0.05
        
        # Integrator hold state
        self._integrator_hold: bool = False
        
        # Outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0
        
        # Diagnostics
        self._last_diag = {}

        if saved_state:
            self.load_state(saved_state)

        _LOGGER.debug("%s - Robust AutoPI initialized", self._name)

    @property
    def deadtime_s(self) -> int:
        """Return current dead time estimate."""
        return self.deadtime_estimator.get_estimate()

    def reset_learning(self):
        """Reset learning to default values."""
        self.est = ABEstimator()
        self.deadtime_estimator = DeadtimeEstimator()
        self.integral = 0.0
        self.u_prev = 0.0
        self._prev_error = 0.0
        self.e_filt = None
        self.Kp = 0.8
        self.Ki = 0.05
        self._pending_on_front_ts = None
        self._prev_temp = None
        self._prev_temp_ts = None
        _LOGGER.info("%s - AutoPI learning reset to defaults", self._name)

    def load_state(self, state: Dict[str, Any]):
        """Load persistent state"""
        if not state:
            return
        self.est.a = state.get('a', A_INIT)
        self.est.b = state.get('b', B_INIT)
        self.est.learn_ok_count = int(state.get('learn_ok_count', 0))
        b_hist = state.get('b_hist', [])
        if b_hist:
             self.est.b_hist = deque(b_hist, maxlen=B_STABILITY_WINDOW)
             
        self.integral = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        
        # Load dead time estimator state
        deadtime_samples = state.get('deadtime_samples', [])
        if deadtime_samples:
            self.deadtime_estimator.samples = deque(deadtime_samples, maxlen=DEADTIME_SAMPLES_MAX)
            self.deadtime_estimator._update_estimate()
        else:
            # Fallback to old format
            self.deadtime_estimator.current_estimate = int(state.get('deadtime_s', DEADTIME_INIT_S))
            
        # Keep existing gains as starting point
        self.Kp = state.get('Kp', 0.8)
        self.Ki = state.get('Ki', 0.05)
        
        _LOGGER.debug(
            "%s - AutoPI state loaded: a=%.6f, b=%.6f, dt=%ds, samples=%d", 
            self._name, self.est.a, self.est.b, self.deadtime_s, len(self.deadtime_estimator.samples)
        )

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence"""
        return {
            'a': self.est.a,
            'b': self.est.b,
            'learn_ok_count': self.est.learn_ok_count,
            'b_hist': list(self.est.b_hist),
            'integral': self.integral,
            'u_prev': self.u_prev,
            'deadtime_s': self.deadtime_s,
            'deadtime_samples': list(self.deadtime_estimator.samples),
            'Kp': self.Kp,
            'Ki': self.Ki,
        }
    
    def notify_setpoint_change(self, t_set_old: float, t_set_new: float, t_in: float, is_major: bool = False):
        """
        Handle setpoint changes.
        
        Args:
            t_set_old: Previous setpoint
            t_set_new: New setpoint
            t_in: Current temperature
            is_major: If True, reset integral completely (mode change like eco->comfort)
        """
        self._last_setpoint_change_ts = time.time()
        
        if is_major:
            # Major change: reset integral and filter
            self.integral = 0.0
            self.e_filt = None
            _LOGGER.debug("%s - Major setpoint change detected, integral reset", self._name)
        else:
            # Minor change: bumpless transfer
            self.bump_integral_for_setpoint_change(t_set_old, t_set_new, t_in)
            _LOGGER.debug("%s - Minor setpoint change, bumpless transfer applied", self._name)
        
    def bump_integral_for_setpoint_change(self, t_set_old: float, t_set_new: float, t_in: float):
        """Bumpless transfer logic."""
        if self.Ki <= 0: return
        e_old = t_set_old - t_in
        e_new = t_set_new - t_in
        dI = (self.Kp / self.Ki) * (e_old - e_new)
        dI_max = float(SETPOINT_BUMPLESS_MAX_DU) / max(self.Ki, KI_MIN)
        dI = clamp(dI, -dI_max, dI_max)
        self.integral += dI
        
    def notify_actuator_state(self, is_on: bool, u_cycle: float = 0.0):
        """
        Notify the controller of actuator state changes for dead time learning.
        
        Args:
            is_on: True if actuator just turned ON
            u_cycle: Current u_cycle value (for guard conditions)
        """
        now = time.time()
        was_on = self._actuator_is_on
        
        if is_on and not was_on:
            # Rising edge: actuator turned ON
            self._pending_on_front_ts = now
            self._last_u_cycle = u_cycle
            _LOGGER.debug("%s - Actuator ON front detected at %.0f", self._name, now)
            
        self._actuator_is_on = is_on
        self._last_actuator_change_ts = now
        
    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
    ):
        """
        Update model. Called once per cycle with data from valid previous cycle.
        Also handles dead time learning.
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            return
            
        now = time.time()
        
        # Dead time learning: detect response to ON front
        self._learn_deadtime(current_temp, now)
        
        # Update temperature tracking
        self._prev_temp = current_temp
        self._prev_temp_ts = now
            
        # We assume 1 cycle elapsed
        dt_min = float(self._cycle_min)
        
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        
        self.est.learn(
             u_eff=previous_power,
             t_in_prev=previous_temp,
             t_in_now=current_temp,
             t_out_prev=t_ext,
             dt_min=dt_min
        )
        
    def _learn_deadtime(self, current_temp: float, now: float):
        """Learn dead time from temperature response after ON front."""
        # Skip if no pending ON front
        if self._pending_on_front_ts is None:
            return
            
        # Skip if not enough data
        if self._prev_temp is None or self._prev_temp_ts is None:
            return
            
        # Calculate temperature slope
        dt_s = now - self._prev_temp_ts
        if dt_s < 60:  # Need at least 60s between samples
            return
            
        dT = current_temp - self._prev_temp
        dTpm = (dT / dt_s) * 60  # Convert to °C/min
        
        # Check if slope is positive enough to indicate heating response
        if dTpm > DEAD_DTPM_TH:
            # Guard conditions to avoid learning in inadequate situations
            time_since_setpoint_change = now - self._last_setpoint_change_ts
            time_since_boot = now - self._boot_ts
            
            # Skip if too close to setpoint change or boot
            if time_since_setpoint_change < 1200:  # 20 min
                _LOGGER.debug("%s - Deadtime learning skipped: too close to setpoint change", self._name)
                self._pending_on_front_ts = None
                return
                
            if time_since_boot < 600:  # 10 min
                _LOGGER.debug("%s - Deadtime learning skipped: too close to boot", self._name)
                self._pending_on_front_ts = None
                return
            
            # Check error is significant (not in maintenance mode near setpoint)
            if self.e_filt is not None and abs(self.e_filt) < DEADTIME_LEARN_E_MIN_C:
                _LOGGER.debug("%s - Deadtime learning skipped: error too small (%.2f)", self._name, self.e_filt)
                self._pending_on_front_ts = None
                return
                
            # Check u_cycle was significant
            if self._last_u_cycle < DEADTIME_LEARN_U_MIN:
                _LOGGER.debug("%s - Deadtime learning skipped: u_cycle too low (%.2f)", self._name, self._last_u_cycle)
                self._pending_on_front_ts = None
                return
                
            # Calculate and record delay
            delay_s = int(now - self._pending_on_front_ts)
            if 30 <= delay_s <= DEADTIME_MAX_S:
                old_estimate = self.deadtime_s
                self.deadtime_estimator.add_sample(delay_s)
                _LOGGER.info(
                    "%s - Dead time sample: %ds (estimate: %ds -> %ds, samples: %d)",
                    self._name, delay_s, old_estimate, self.deadtime_s, len(self.deadtime_estimator.samples)
                )
            
            # Consume the pending front
            self._pending_on_front_ts = None

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None,
        hvac_mode: VThermHvacMode,
    ):
        if target_temp is None or current_temp is None:
            self._on_percent = 0
            return

        if hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0
            self.u_prev = 0
            return
        
        now = time.time()
        
        # Calculate actual dt_min based on time since last calculate
        if self._last_calculate_ts > 0:
            elapsed_min = (now - self._last_calculate_ts) / 60.0
            dt_min = clamp(elapsed_min, 0.2, 8.0)  # Clamp to reasonable range
        else:
            dt_min = 1.0  # Default for first call
        self._last_calculate_ts = now
        
        # Determine integrator hold based on dead time after actuator change
        time_since_change = now - self._last_actuator_change_ts
        self._integrator_hold = time_since_change < self.deadtime_s
        
        # 1. Update Gains (Gain Scheduling + IMC)
        a = self.est.a
        b = self.est.b
        tau_info = self.est.tau_reliability()
        
        if tau_info.reliable and a > 2e-4 and b > B_MIN:
            tau = tau_info.tau_min
            K = a / max(b, B_MIN)
            theta = float(self.deadtime_s) / 60.0
            lam_base = max(LAMBDA_MIN_MIN, LAMBDA_FACTOR * theta)
            lam = lam_base
            
            # Lambda scheduling
            if LAMBDA_SCHED_ENABLE and self.e_filt is not None:
                ae = abs(self.e_filt)
                if ae <= GAIN_SCHED_E1_C:
                    f = LAMBDA_SCHED_NEAR_FACTOR
                elif ae >= GAIN_SCHED_E2_C:
                    f = 1.0
                else:
                    t = (ae - GAIN_SCHED_E1_C) / max(1e-9, (GAIN_SCHED_E2_C - GAIN_SCHED_E1_C))
                    f = LAMBDA_SCHED_NEAR_FACTOR + (1.0 - LAMBDA_SCHED_NEAR_FACTOR) * t
                lam = lam_base * f
            
            if K < 1e-6:
                kp_t, ki_t = 1.0, 0.003 # Safe
            else:
                kp_t = tau / (K * (lam + theta))
                Ti = max(tau, 1e-3)
                ki_t = kp_t / Ti
                kp_t = clamp(kp_t, KP_MIN, KP_MAX)
                ki_t = clamp(ki_t, KI_MIN, KI_MAX)
        else:
            kp_t, ki_t = 1.0, 0.003 # Safe
            
        # Smooth gains
        alpha_g = clamp(GAINS_SMOOTH_ALPHA, 0.0, 1.0)
        self.Kp = (1 - alpha_g) * self.Kp + alpha_g * kp_t
        self.Ki = (1 - alpha_g) * self.Ki + alpha_g * ki_t
        
        # 2. Control Loop
        e = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            e = -e
        self._prev_error = e

        # EMA Error
        ema_alpha = 0.35
        if self.e_filt is None:
            self.e_filt = e
        else:
            self.e_filt = (1 - ema_alpha) * self.e_filt + ema_alpha * e

        # Overshoot un-winding
        if self.e_prev is not None:
            if (self.e_prev > 0.0 and e < 0.0) or (self.e_prev < 0.0 and e > 0.0):
                 self.integral *= 0.75
        self.e_prev = e
        
        # Gain application
        kp_eff = self.Kp
        ki_eff = self.Ki
        if GAIN_SCHED_ENABLE and self.e_filt is not None:
            ae = abs(self.e_filt)
            if ae <= GAIN_SCHED_E1_C:
                 f = GAIN_SCHED_KP_MIN_FACTOR
            elif ae >= GAIN_SCHED_E2_C:
                 f = 1.0
            else:
                 f = GAIN_SCHED_KP_MIN_FACTOR + (1.0 - GAIN_SCHED_KP_MIN_FACTOR) * (ae - GAIN_SCHED_E1_C) / max(1e-6, GAIN_SCHED_E2_C - GAIN_SCHED_E1_C)
            kp_eff *= f
            ki_eff *= (f ** GAIN_SCHED_KI_EXP)
            
        kp = clamp(kp_eff, KP_MIN, KP_MAX)
        ki = clamp(ki_eff, KI_MIN, KI_MAX)
        
        # Feed-Forward
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        u_ff = 0.0
        if a > 2e-4:
            kff = clamp(b / a, 0.0, 3.0)
            u_ff = clamp(kff * (target_temp - t_ext), 0.0, 1.0)
            
        # PI calculation
        i_max = 2.0 / max(ki, KI_MIN)
        
        u_pi = 0.0
        sat_state = "NO_SAT"
        
        if abs(e) < self.deadband_c:
             self.integral *= INTEGRAL_LEAK
        elif self._integrator_hold:
            # Hold integrator during dead time
            u_pi = kp * e + ki * self.integral
        else:
             u_pi_pre = kp * e + ki * self.integral
             u_raw_pre = u_ff + u_pi_pre
             
             if u_raw_pre > 1.0: sat_state = "SAT_HI"
             elif u_raw_pre < 0.0: sat_state = "SAT_LO"
             
             if (sat_state == "SAT_HI" and e > 0) or (sat_state == "SAT_LO" and e < 0):
                 # Skip integration (conditional integration)
                 u_pi = u_pi_pre
             else:
                 self.integral += e * dt_min
                 self.integral = clamp(self.integral, -i_max, i_max)
                 u_pi = kp * e + ki * self.integral
                 
        u_raw = u_ff + u_pi
        u_sat = clamp(u_raw, 0.0, 1.0)
        
        # Anti-windup back-calculation
        if AW_BACKCALC_ENABLE and sat_state != "NO_SAT" and not self._integrator_hold:
            sat_err = u_sat - u_raw
            self.integral += AW_BACKCALC_GAIN * (sat_err / max(ki, KI_MIN))
            self.integral = clamp(self.integral, -i_max, i_max)
            
        # Rate limiting
        max_step = self.max_step_per_min * max(dt_min, 1e-3)
        u = clamp(u_sat, self.u_prev - max_step, self.u_prev + max_step)
        
        # Min useful
        if 0.0 < u < self.min_useful:
             u = 0.0
             
        self.u_prev = u
        self._on_percent = u
        
        self._calculate_times()
        
        # Capture diag
        self._last_diag = {
            "a": a, "b": b, "Kp": kp, "Ki": ki, "integral": self.integral,
            "u_bg": self.u_prev, "u_ff": u_ff, "error": e, 
            "integrator_hold": self._integrator_hold, "dt_min": dt_min
        }
        
    def _calculate_times(self):
        """Calculate on/off times from percent"""
        self._on_time_sec = int(self._on_percent * self._cycle_min * 60)
        if self._on_time_sec < self._minimal_activation_delay:
            self._on_time_sec = 0
        self._off_time_sec = int(self._cycle_min * 60 - self._on_time_sec)
        if self._off_time_sec < self._minimal_deactivation_delay:
            self._on_time_sec = int(self._cycle_min * 60)
            self._off_time_sec = 0

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information for attributes"""
        diag = self._last_diag.copy()
        diag.update({
            "a": self.est.a,
            "b": self.est.b,
            "Kp": self.Kp,
            "Ki": self.Ki,
            "integral_error": self.integral,
            "error": self.prev_error,
            "deadtime_s": self.deadtime_s,
            "deadtime_samples": len(self.deadtime_estimator.samples),
            "learn_ok_count": self.est.learn_ok_count,
            "tau_reliable": self.est.tau_reliability().reliable,
            "last_learn_reason": self.est.learn_last_reason,
            "outliers_count": self.est.outliers_a + self.est.outliers_b,
            "integrator_hold": self._integrator_hold,
        })
        return diag 

    @property
    def on_percent(self):
        return self._on_percent

    @property
    def calculated_on_percent(self):
        return self._on_percent

    @property
    def on_time_sec(self):
        return self._on_time_sec

    @property
    def off_time_sec(self):
        return self._off_time_sec

    @property
    def integral_error(self):
        return self.integral

    @property
    def prev_error(self):
        return getattr(self, '_prev_error', 0.0)

    @property
    def a(self):
        """Return 'a' parameter for compatibility."""
        return self.est.a

    @property
    def b(self):
        """Return 'b' parameter for compatibility."""
        return self.est.b

    @property
    def u_ff(self):
        """Return last u_ff value for diagnostics."""
        return self._last_diag.get('u_ff', 0.0)
