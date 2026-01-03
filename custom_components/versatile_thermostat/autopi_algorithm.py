"""
AutoPI Algorithm - Auto-adaptive PI controller for Versatile Thermostat.

This module implements a PI controller with:
- Online model learning (EWMA conditional): dT_int (°C/min) ≈ a * u - b * (T_int - T_ext)
- Heuristic-based PI tuning with tau reliability check
- Feed-forward compensation for thermal losses
- Conditional integration anti-windup
- Integrator hold during dead time

The output is a power command between 0 and 1 (0-100%), to be applied as duty-cycle.
"""
import logging
import statistics
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Deque

from .vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL, VThermHvacMode_OFF, VThermHvacMode_HEAT

_LOGGER = logging.getLogger(__name__)

# Model parameters (EWMA)
A_INIT = 0.00050
B_INIT = 0.00100
A_MIN, A_MAX = 1e-5, 0.05
B_MIN, B_MAX = 1e-6, 0.02
ALPHA_A = 0.15  # EWMA factor for a
ALPHA_B = 0.10  # EWMA factor for b

# Learning thresholds
MIN_DTPM_FOR_LEARN = 0.003  # °C/min minimum for learning (noise rejection)
MIN_DELTA_T_FOR_B = 0.5  # Min |Tin-Text| for b learning

# PI tuning bounds
KP_MIN, KP_MAX = 0.2, 1.5
KI_MIN, KI_MAX = 0.0005, 0.25

# Safe gains when tau is unreliable
KP_SAFE = 1.0
KI_SAFE = 0.003

# Tau reliability settings
TAU_MIN_PLAUSIBLE_MIN = 30  # Minimum plausible tau (minutes)
TAU_MAX_PLAUSIBLE_MIN = 3000  # Maximum plausible tau (minutes)
B_STABILITY_WINDOW = 20  # Window size for b stability check
B_CV_MAX = 0.35  # Maximum coefficient of variation for b
LEARN_OK_MIN = 6  # Minimum successful learns for reliability

# PI settings
DEADBAND_C = 0.05  # Default deadband
INTEGRAL_LEAK = 0.985  # Integral leak factor in deadband
MAX_STEP_PER_MIN = 0.12  # Max u change per minute (rate limiting)


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value to the interval [lo, hi]."""
    return max(lo, min(hi, x))


@dataclass
class TauReliability:
    """Result of tau reliability check."""
    reliable: bool
    reason: str
    tau_min: float


@dataclass
class ABEstimator:
    """
    EWMA-based estimator for thermal model parameters.
    
    Model: dT_int = a * u - b * (T_int - T_ext)
    - a: heating efficiency (°C/min at 100% power)
    - b: thermal losses (1/min)
    - tau = 1/b: time constant (minutes)
    
    Learning strategy:
    - b is best identified when u ≈ 0 (cooling phase)
    - a is best identified when u > 0.20 (significant heating)
    """
    a: float = A_INIT
    b: float = B_INIT
    
    learn_ok_count: int = 0
    learn_last_reason: str = "init"
    b_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=B_STABILITY_WINDOW))
    
    def learn(
        self,
        u_eff: float,
        t_in_prev: float,
        t_in_now: float,
        t_out_prev: float,
        dt_min: float,
    ) -> tuple[bool, str, float]:
        """
        Update model parameters from observed data.
        
        Args:
            u_eff: Effective power applied during the period [0, 1]
            t_in_prev: Indoor temperature at start (°C)
            t_in_now: Indoor temperature at end (°C)
            t_out_prev: Outdoor temperature at start (°C)
            dt_min: Duration of the period (minutes)
            
        Returns:
            Tuple of (learn_ok, reason, dTpm)
        """
        if dt_min <= 1e-6:
            self.learn_last_reason = "skip:dt too small"
            return False, self.learn_last_reason, 0.0
        
        dT = t_in_now - t_in_prev
        dTpm = dT / dt_min  # Temperature change per minute
        
        # Reject noisy measurements (quantized sensors)
        if abs(dTpm) < MIN_DTPM_FOR_LEARN:
            self.learn_last_reason = "skip:|dT/min| too low"
            return False, self.learn_last_reason, dTpm
        
        delta = t_in_prev - t_out_prev  # Tin - Text at start
        
        # Learn b primarily during OFF periods (u ≈ 0)
        if u_eff < 0.05:
            if abs(delta) < MIN_DELTA_T_FOR_B:
                self.learn_last_reason = "skip:b:|Tin-Text| too low"
                return False, self.learn_last_reason, dTpm
            b_meas = -dTpm / delta
            if b_meas <= 0:
                self.learn_last_reason = "skip:b:measured b<=0"
                return False, self.learn_last_reason, dTpm
            self.b = clamp((1 - ALPHA_B) * self.b + ALPHA_B * b_meas, B_MIN, B_MAX)
            self.b_hist.append(self.b)
            self.learn_ok_count += 1
            self.learn_last_reason = "update:b(off)"
            return True, self.learn_last_reason, dTpm
        
        # Learn a primarily during ON periods (u > 0.20)
        if u_eff > 0.20:
            a_meas = (dTpm + self.b * delta) / max(u_eff, 1e-6)
            if a_meas <= 0:
                self.learn_last_reason = "skip:a:measured a<=0"
                return False, self.learn_last_reason, dTpm
            self.a = clamp((1 - ALPHA_A) * self.a + ALPHA_A * a_meas, A_MIN, A_MAX)
            self.learn_ok_count += 1
            self.learn_last_reason = "update:a(on)"
            return True, self.learn_last_reason, dTpm
        
        self.learn_last_reason = "skip:gray zone u"
        return False, self.learn_last_reason, dTpm
    
    def tau_reliability(self) -> TauReliability:
        """
        Check if tau (=1/b) is reliable.
        
        Criteria:
        - Enough successful learning updates
        - tau in plausible range
        - b is stable (low coefficient of variation)
        """
        tau = 1.0 / max(self.b, B_MIN)
        
        if self.learn_ok_count < LEARN_OK_MIN:
            return TauReliability(False, f"tau:NO (learn_ok<{LEARN_OK_MIN})", tau)
        
        if not (TAU_MIN_PLAUSIBLE_MIN <= tau <= TAU_MAX_PLAUSIBLE_MIN):
            return TauReliability(False, "tau:NO (out of plausible range)", tau)
        
        if len(self.b_hist) < max(6, B_STABILITY_WINDOW // 2):
            return TauReliability(False, "tau:NO (not enough b history)", tau)
        
        mean_b = statistics.mean(self.b_hist)
        if mean_b <= 0:
            return TauReliability(False, "tau:NO (mean(b)<=0)", tau)
        
        std_b = statistics.pstdev(self.b_hist)
        cv = std_b / mean_b
        if cv > B_CV_MAX:
            return TauReliability(False, f"tau:NO (b unstable, CV={cv:.2f})", tau)
        
        return TauReliability(True, f"tau:OK (CV(b)={cv:.2f})", tau)


class AutoPI:
    """
    Auto-adaptive PI controller with feed-forward compensation.
    
    Features:
    - Online model learning via EWMA (conditional on u value)
    - Heuristic-based PI tuning with reliability check
    - Feed-forward based on outdoor temperature
    - Conditional integration anti-windup
    - Integrator hold during dead time
    """

    def __init__(
        self,
        cycle_min: int,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: float = None,
        deadband_c: float = DEADBAND_C,
        aggressiveness: float = 0.5,
        saved_state: Optional[Dict[str, Any]] = None
    ):
        self._name = name
        self._cycle_min = cycle_min
        self._minimal_activation_delay = minimal_activation_delay
        self._minimal_deactivation_delay = minimal_deactivation_delay
        self._max_on_percent = max_on_percent
        
        # Tuning parameters
        self.deadband_c = deadband_c
        self.aggressiveness = aggressiveness
        
        # Model estimator
        self.est = ABEstimator()
        
        # PI state
        self.integral: float = 0.0
        self.u_prev: float = 0.0
        
        # Error filtering (EMA)
        self._e_filt: Optional[float] = None
        self._ema_alpha: float = 0.35
        
        # Current gains
        self.Kp: float = KP_SAFE
        self.Ki: float = KI_SAFE
        
        # Outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0
        
        # Diagnostics
        self._last_u_ff: float = 0.0
        self._last_error: float = 0.0
        self._last_i_mode: str = "init"
        self._tau_reliable: bool = False
        
        if saved_state:
            self.load_state(saved_state)
        
        _LOGGER.debug("%s - AutoPI initialized", self._name)

    @property
    def a(self) -> float:
        """Heating efficiency parameter."""
        return self.est.a

    @property
    def b(self) -> float:
        """Thermal losses parameter."""
        return self.est.b

    @property
    def on_percent(self) -> float:
        """Current power command [0, 1]."""
        return self._on_percent

    @property
    def calculated_on_percent(self) -> float:
        """Alias for on_percent."""
        return self._on_percent

    @property
    def on_time_sec(self) -> int:
        """ON time in seconds for duty-cycle."""
        return self._on_time_sec

    @property
    def off_time_sec(self) -> int:
        """OFF time in seconds for duty-cycle."""
        return self._off_time_sec

    @property
    def integral_error(self) -> float:
        """Current integral accumulator value."""
        return self.integral

    @property
    def u_ff(self) -> float:
        """Last feed-forward value."""
        return self._last_u_ff

    def reset_learning(self) -> None:
        """Reset all learned parameters to defaults."""
        self.est = ABEstimator()
        self.integral = 0.0
        self.u_prev = 0.0
        self._e_filt = None
        self.Kp = KP_SAFE
        self.Ki = KI_SAFE
        _LOGGER.info("%s - AutoPI learning reset to defaults", self._name)

    def load_state(self, state: Dict[str, Any]) -> None:
        """Load persistent state."""
        if not state:
            return
        self.est.a = state.get('a', A_INIT)
        self.est.b = state.get('b', B_INIT)
        self.est.learn_ok_count = state.get('learn_ok_count', 0)
        b_hist_data = state.get('b_hist', [])
        self.est.b_hist = deque(b_hist_data, maxlen=B_STABILITY_WINDOW)
        self.integral = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        _LOGGER.debug(
            "%s - AutoPI state loaded: a=%.6f, b=%.6f, learn_ok=%d",
            self._name, self.est.a, self.est.b, self.est.learn_ok_count
        )

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence."""
        return {
            'a': self.est.a,
            'b': self.est.b,
            'learn_ok_count': self.est.learn_ok_count,
            'b_hist': list(self.est.b_hist),
            'integral': self.integral,
            'u_prev': self.u_prev,
        }

    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
        cycle_dt: float = None,
    ) -> None:
        """
        Update the thermal model with observed data.
        
        Called once per cycle with data from the previous cycle.
        
        Args:
            current_temp: Current indoor temperature (°C)
            ext_current_temp: Current outdoor temperature (°C)
            previous_temp: Indoor temperature at start of cycle (°C)
            previous_power: Average power applied during cycle [0, 1]
            hvac_mode: Current HVAC mode
            cycle_dt: Cycle duration in minutes (defaults to self._cycle_min)
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return
        
        if hvac_mode != VThermHvacMode_HEAT:
            return
        
        dt_minutes = cycle_dt if cycle_dt is not None else self._cycle_min
        if dt_minutes <= 0:
            return
        
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        
        self.est.learn(
            u_eff=previous_power,
            t_in_prev=previous_temp,
            t_in_now=current_temp,
            t_out_prev=t_ext,
            dt_min=dt_minutes
        )

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None,
        hvac_mode: VThermHvacMode,
        integrator_hold: bool = False,
    ) -> None:
        """
        Calculate the power command.
        
        Args:
            target_temp: Setpoint temperature (°C)
            current_temp: Current indoor temperature (°C)
            ext_current_temp: Current outdoor temperature (°C)
            slope: Temperature slope (unused, kept for API compatibility)
            hvac_mode: Current HVAC mode
            integrator_hold: If True, freeze integrator (dead time period)
        """
        if target_temp is None or current_temp is None:
            self._on_percent = 0
            self._calculate_times()
            return
        
        if hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0
            self.u_prev = 0
            self._calculate_times()
            return
        
        # Get model parameters
        a = self.est.a
        b = self.est.b
        
        # Check tau reliability
        tau_info = self.est.tau_reliability()
        self._tau_reliable = tau_info.reliable
        
        # Calculate gains based on tau reliability
        if tau_info.reliable:
            tau = tau_info.tau_min
            # Heuristic: Kp scales with tau, Ki = Kp/tau
            kp_calc = 0.35 + 0.9 * (tau / 200.0)
            self.Kp = clamp(kp_calc, KP_MIN, KP_MAX)
            self.Ki = clamp(self.Kp / max(tau, 10.0), KI_MIN, KI_MAX)
        else:
            # Safe gains when model is unreliable
            self.Kp = KP_SAFE
            self.Ki = KI_SAFE
        
        # Calculate error
        e = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            e = -e
        self._last_error = e
        
        # EMA filtering of error (for quantized sensors)
        if self._e_filt is None:
            self._e_filt = e
        else:
            self._e_filt = (1 - self._ema_alpha) * self._e_filt + self._ema_alpha * e
        
        # Feed-forward calculation
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        if a < 2e-4:
            # a too low, FF unreliable
            u_ff = 0.0
        else:
            k_ff = clamp(b / a, 0.0, 3.0)
            u_ff = clamp(k_ff * (target_temp - t_ext), 0.0, 1.0)
        self._last_u_ff = u_ff
        
        # Dynamic integral limit: bound so |Ki * I| <= 2.0
        i_max = 2.0 / max(self.Ki, KI_MIN)
        
        # PI control with conditional integration anti-windup
        if abs(e) < self.deadband_c:
            # In deadband: leak integral slowly
            self.integral *= INTEGRAL_LEAK
            u_pi = 0.0
            self._last_i_mode = "I:LEAK(deadband)"
        else:
            if integrator_hold:
                # Dead time: don't integrate to avoid pumping
                u_pi = self.Kp * e + self.Ki * self.integral
                self._last_i_mode = "I:HOLD(dead time)"
            else:
                # Compute preliminary output without updating integral
                u_pi_pre = self.Kp * e + self.Ki * self.integral
                u_raw_pre = u_ff + u_pi_pre
                
                # Check saturation
                if u_raw_pre > 1.0:
                    sat_state = "SAT_HI"
                elif u_raw_pre < 0.0:
                    sat_state = "SAT_LO"
                else:
                    sat_state = "NO_SAT"
                
                # Conditional integration:
                # Skip integration if saturated AND error would make it worse
                if (sat_state == "SAT_HI" and e > 0) or (sat_state == "SAT_LO" and e < 0):
                    self._last_i_mode = f"I:SKIP({sat_state})"
                    u_pi = u_pi_pre
                else:
                    # Normal integration
                    self.integral += e * self._cycle_min
                    self.integral = clamp(self.integral, -i_max, i_max)
                    self._last_i_mode = "I:RUN"
                    u_pi = self.Kp * e + self.Ki * self.integral
        
        # Combine FF and PI
        u_raw = u_ff + u_pi
        u = clamp(u_raw, 0.0, 1.0)
        
        # Rate limiting
        max_step = MAX_STEP_PER_MIN * max(self._cycle_min, 1e-3)
        u = clamp(u, self.u_prev - max_step, self.u_prev + max_step)
        
        # Apply max_on_percent limit
        if self._max_on_percent is not None and u > self._max_on_percent:
            u = self._max_on_percent
        
        self.u_prev = u
        self._on_percent = u
        
        self._calculate_times()

    def _calculate_times(self) -> None:
        """Calculate ON/OFF times from power percentage."""
        self._on_time_sec = int(self._on_percent * self._cycle_min * 60)
        if self._on_time_sec < self._minimal_activation_delay:
            self._on_time_sec = 0
        self._off_time_sec = int(self._cycle_min * 60 - self._on_time_sec)
        if self._off_time_sec < self._minimal_deactivation_delay:
            self._on_time_sec = int(self._cycle_min * 60)
            self._off_time_sec = 0

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information for attributes."""
        tau_info = self.est.tau_reliability()
        
        return {
            "a": round(self.est.a, 6),
            "b": round(self.est.b, 6),
            "tau_min": round(tau_info.tau_min, 1),
            "tau_reliable": tau_info.reliable,
            "learn_ok_count": self.est.learn_ok_count,
            "learn_last_reason": self.est.learn_last_reason,
            "Kp": round(self.Kp, 3),
            "Ki": round(self.Ki, 4),
            "integral_error": round(self.integral, 2),
            "error": round(self._last_error, 2),
            "error_filtered": round(self._e_filt, 2) if self._e_filt is not None else None,
            "u_ff": round(self._last_u_ff, 3),
            "i_mode": self._last_i_mode,
            "on_percent": round(self._on_percent, 3),
        }
