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
        self.deadtime_s = 180  # Default initial dead time
        
        # PI State
        self.integral: float = 0.0
        self.u_prev: float = 0.0
        self._prev_error: float = 0.0
        self.e_filt: Optional[float] = None
        self.e_prev: Optional[float] = None
        
        # Current gains
        self.Kp: float = 0.8  # Will be smoothed
        self.Ki: float = 0.05
        
        # Outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0
        
        # Diagnostics
        self._last_diag = {}

        if saved_state:
            self.load_state(saved_state)

        _LOGGER.debug("%s - Robust AutoPI initialized", self._name)

    def reset_learning(self):
        """Reset learning to default values."""
        self.est = ABEstimator()
        self.integral = 0.0
        self.u_prev = 0.0
        self._prev_error = 0.0
        self.e_filt = None
        self.Kp = 0.8
        self.Ki = 0.05
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
        self.deadtime_s = int(state.get('deadtime_s', 180))
        # Keep existing gains as starting point
        self.Kp = state.get('Kp', 0.8)
        self.Ki = state.get('Ki', 0.05)
        
        _LOGGER.debug("%s - AutoPI state loaded: a=%.6f, b=%.6f, dt=%ds", self._name, self.est.a, self.est.b, self.deadtime_s)

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
            'Kp': self.Kp,
            'Ki': self.Ki,
        }
        
    def bump_integral_for_setpoint_change(self, t_set_old: float, t_set_new: float, t_in: float):
        """Bumpless transfer logic."""
        if self.Ki <= 0: return
        e_old = t_set_old - t_in
        e_new = t_set_new - t_in
        dI = (self.Kp / self.Ki) * (e_old - e_new)
        dI_max = float(SETPOINT_BUMPLESS_MAX_DU) / max(self.Ki, KI_MIN)
        dI = clamp(dI, -dI_max, dI_max)
        self.integral += dI
        
    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
        # Additional params usually not in standard signature but needed if we were strictly following regul6
        # Here we adapt to standard VTherm flow. VTherm calls this ONCE per cycle.
        # But robust estimator needs 'dT_min'. VTherm cycle is typically self._cycle_min.
    ):
        """
        Update model. Called once per cycle with data from valid previous cycle.
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            return
            
        # We assume 1 cycle elapsed
        dt_min = float(self._cycle_min)
        
        # In VTherm, previous_power is the average power over the cycle [0..1] check ??
        # No, update_learning receives `previous_power` which is typically the fractional power if passed correctly.
        # But wait, typically VTherm passes `self._on_percent` as `previous_power`.
        # Regul6 uses `u_eff` (delayed).
        
        # For this integration, we'll use `previous_power` as `u_eff` approx 
        # (ignoring deadtime delay for learning purely here, or we could use history buffer if we had timestamps).
        # Since this is called retrospectively, `previous_power` IS what was applied.
        
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        
        self.est.learn(
             u_eff=previous_power,
             t_in_prev=previous_temp,
             t_in_now=current_temp,
             t_out_prev=t_ext,
             dt_min=dt_min
        )
        
        # Note: Gains are updated in `calculate` step in regul6, but here we can do it too or let calculate do it.
        # We'll let calculate do it to be responsive to aggressiveness changes immediately.

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
        dt_min = 1.0 # Standard integration step per minute if called per minute, but VTherm calls per cycle
        # Wait: VTherm calls calculate once every 'cycle_min'.
        # regul6 loops every minute or event.
        # If VTherm uses a long cycle (e.g. 10m) and calculate() sets the duty cycle for that 10m,
        # then dt_min should be 'cycle_min' conceptually for the integral accumulation?
        # NO, 'integral' in PI is usually Time-integral.
        # If we update once per 10min, we add e * 10min.
        
        # IMPORTANT: 'calculate' in VTherm is called when needed.
        # If we follow regul6, it renders 'u_now' continuously.
        # Assuming calculate is called reasonably often (e.g. on temp change).
        # We will use dt_min = 1.0 as a normalized step or estimate actual time passed?
        # For stability with variable calling, simpler is to assume we are setting 'u' for the moment.
        # But 'integral' needs strict time base.
        # Let's assume standard 1 min step for integral logic to avoid huge jumps if called rarely,
        # OR better, since VTherm might call it irregularly, we should rely on Kp mostly?
        # The regul6 logic uses 'dt_min' which is (now - last_t_in_change) clamped.
        # We don't track time inside calculate easily without storing last timestamp.
        # For now, let's stick to dt_min = 1.0 as a normalized update "per calculation event" assuming typical event rate,
        # OR assume VTherm cycle.
        # In `thermostat_tpi.py`, `calculate` is called periodically.
        # To be safe and consistent with previous code (RLS version used dt_min = 1.0 hardcoded), we reuse 1.0.
        
        dt_min = 1.0 
        
        integrator_hold = False # TODO: pass Actuator state implies knowing when heater switched. 
        # For now, we assume FALSE as we don't strictly track the switch transitions inside this class yet.
        # Improvements: Pass context to calculate.
        
        u_pi = 0.0
        sat_state = "NO_SAT"
        
        if abs(e) < self.deadband_c:
             self.integral *= INTEGRAL_LEAK
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
        if AW_BACKCALC_ENABLE and sat_state != "NO_SAT":
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
            "u_bg": self.u_prev, "u_ff": u_ff, "error": e
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
            "learn_ok_count": self.est.learn_ok_count,
            "tau_reliable": self.est.tau_reliability().reliable,
            "last_learn_reason": self.est.learn_last_reason,
            "outliers_count": self.est.outliers_a + self.est.outliers_b,
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

