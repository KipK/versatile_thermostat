"""
SmartPI Algorithm - Auto-adaptive PI controller for Versatile Thermostat (VTH).

This module implements a duty-cycle PI controller for slow thermal systems (heating),
with on-line identification of a simple 1st-order loss model and pragmatic safeguards.

Key features
------------
- Online learning of a and b in the model (discrete-time, per-minute form):

    dT_int ("C/min) = a * u - b * (T_int - T_ext)

  where:
    * u in [0, 1] is the power command (duty-cycle)
    * a ("C/min) is the heating effectiveness
    * b (1/min) is the loss coefficient

- Reliability check using tau = 1/b (minutes) inferred from the model; PI gains fall back
  to conservative values when the model is not reliable.

- Feed-forward compensation based on the learned model to reduce steady-state error
  and improve efficiency.

- Anti-windup (conditional integration) for saturated actuators.

- Servo-oriented overshoot reduction (change of setpoint):
    * 2DOF PI (setpoint weighting) on the proportional action
    * Near-setpoint gain scheduling (Kp/Ki reduction near target)
    * Optional integrator "leak" after error sign change (soft discharge)

The output is a power command between 0 and 1 (0-100%), applied as duty-cycle over a
fixed cycle period (e.g., 10 minutes). The controller also computes ON/OFF times in
seconds, optionally constrained by minimum activation/deactivation times.
"""

from __future__ import annotations

import logging
import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Deque, Dict, Optional

from .vtherm_hvac_mode import (
    VThermHvacMode,
    VThermHvacMode_COOL,
    VThermHvacMode_HEAT,
    VThermHvacMode_OFF,
)

_LOGGER = logging.getLogger(__name__)


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into [lo, hi]."""
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


# ------------------------------
# Default controller parameters
# ------------------------------

# Safe fallback gains when model is unreliable
KP_SAFE = 0.55
KI_SAFE = 0.010

# Allowed ranges for computed gains
KP_MIN = 0.10
KP_MAX = 2.50
KI_MIN = 0.001
KI_MAX = 0.050

# Cap for tau in Ki calculation (prevents extremely small Ki for slow systems)
TAU_CAP_FOR_KI = 200.0

# Anti-windup / integrator behavior
INTEGRAL_LEAK = 0.995  # leak factor per cycle when inside deadband
MAX_STEP_PER_MINUTE = 0.25  # max output change per minute (rate limit)

# Setpoint step boost: faster rate limit when setpoint changes significantly
# This allows quick power ramp-up when user increases setpoint
SETPOINT_BOOST_THRESHOLD = 0.3   # min setpoint change (°C) to trigger boost
SETPOINT_BOOST_ERROR_MIN = 0.3   # min error (°C) to keep boost active
SETPOINT_BOOST_RATE = 0.50       # boosted rate limit (/min) vs 0.15 normal

# Tracking anti-windup (back-calculation) tuned for slow thermal systems
AW_TRACK_TAU_S = 120.0        # tracking time constant in seconds (typ. 60-180s)
AW_TRACK_MAX_DELTA_I = 5.0    # safety clamp on integral correction per cycle

# Skip cycles after resume from interruption (window, etc.)
SKIP_CYCLES_AFTER_RESUME = 1

# Periodic recalculation interval (seconds) for SmartPI
# This ensures the rate-limit progresses even when temperature sensors don't update frequently
SMARTPI_RECALC_INTERVAL_SEC = 60

# Default deadband around setpoint (°C)
DEFAULT_DEADBAND_C = 0.05

# Absolute hysteresis for deadband exit (reduces oscillations at boundary)
# Enter deadband at |e| < deadband_c, exit only when |e| > deadband_c + hysteresis
# Using absolute value (not multiplicative) ensures consistent behavior across
# different deadband configurations and typical sensor noise levels.
DEADBAND_HYSTERESIS = 0.025

# Asymmetric setpoint EMA filter parameters
SP_ALPHA_SLOW = 0.05   # EMA alpha for small setpoint increases
SP_ALPHA_FAST = 0.40   # EMA alpha for large setpoint jumps
SP_BAND = 1.0          # Band for alpha interpolation (°C)
SP_BYPASS_ERROR_THRESHOLD = 0.8  # Bypass filter when error > this (°C) to avoid slow heating


# --- Robust learning / gating constants ---
# Window sizes
B_POINTS_MAX = 40        # OFF samples for b (tau)
A_POINTS_MAX = 25        # ON samples for a
RESIDUAL_HIST_MAX = 60   # Residual history for MAD estimation

# Robust gating
RESIDUAL_GATE_K = 3.5   # |r| > k * sigma_r  -> freeze learning

# Intercept coherence checks (dimensionless ratios)
INTERCEPT_SIGMA_FACTOR = 2.0   # |c| <= factor * sigma_r
INTERCEPT_SCALE_FACTOR = 0.30  # |c| <= factor * median(|y|)

# Tau stability check
B_STABILITY_MAD_RATIO_MAX = 0.60   # MAD(b) / median(b)
LEARN_BOOTSTRAP_COUNT = 10      # Number of learn cycles before applying strict residual gating


@dataclass(frozen=True)
class TauReliability:
    """Result of tau (time constant) reliability check."""
    reliable: bool
    tau_min: float  # minutes (min of candidates used)


class ABEstimator:
    """
    Robust Online Estimator for a and b using Theil-Sen regression and outlier gating.
    
    Model: dT/dt = a*u - b*(T_int - T_ext)
    
    Improvements over basic EWMA:
    - Uses sliding windows of points (Theil-Sen regression) for robust slope estimation.
    - Residual gating: skips learning if current point deviates too much from model (r > 3.5*sigma).
    - Intercept checks: ensures the linear fit passes near origin (physics constraint).
    - Checks stability of 'b' estimate logic.
    """

    def __init__(self, a_init: float = 0.0005, b_init: float = 0.0010):
        self.A_INIT = a_init
        self.B_INIT = b_init

        self.a = a_init
        self.b = b_init

        # Robust bounds (conservative defaults, can be overridden by learn call)
        self.A_MIN: float = 1e-5
        self.A_MAX: float = 0.1
        self.B_MIN: float = 1e-5
        self.B_MAX: float = 0.05

        # Robust learning buffers (Type: deque[Tuple[float, float]])
        # stored as (x, y)
        self._b_pts: Deque[Tuple[float, float]] = deque(maxlen=B_POINTS_MAX)
        self._a_pts: Deque[Tuple[float, float]] = deque(maxlen=A_POINTS_MAX)
        self._r_hist: Deque[float] = deque(maxlen=RESIDUAL_HIST_MAX)

        # Stability tracking for b (tau)
        self._b_hat_hist: Deque[float] = deque(maxlen=20)

        self.learn_ok_count = 0  # Total successful updates
        self.learn_ok_count_a = 0
        self.learn_ok_count_b = 0
        self.learn_skip_count = 0
        self.learn_last_reason: Optional[str] = "init"

    def reset(self) -> None:
        """Reset learned parameters and history to initial values."""
        self.a = self.A_INIT
        self.b = self.B_INIT
        self.learn_ok_count = 0
        self.learn_ok_count_a = 0
        self.learn_ok_count_b = 0
        self.learn_skip_count = 0
        self.learn_last_reason = "reset"
        self._b_pts.clear()
        self._a_pts.clear()
        self._r_hist.clear()
        self._b_hat_hist.clear()

    # ---------- Robust helpers ----------

    @staticmethod
    def _mad(values):
        if len(values) < 3:
            return None
        med = statistics.median(values)
        try:
            return statistics.median(abs(v - med) for v in values)
        except statistics.StatisticsError:
            return None

    @staticmethod
    def _theil_sen(points):
        """
        Robust Theil–Sen slope + intercept.
        points: list of (x, y)
        """
        slopes = []
        n = len(points)
        # Check for strictly identical x values to avoid division by zero
        # Although in physical systems noise usually prevents this.
        for i in range(n):
            x_i, y_i = points[i]
            for j in range(i + 1, n):
                x_j, y_j = points[j]
                dx = x_j - x_i
                if abs(dx) < 1e-6:
                    continue
                slopes.append((y_j - y_i) / dx)

        if not slopes:
            return None, None

        slope = statistics.median(slopes)
        intercepts = [y - slope * x for x, y in points]
        intercept = statistics.median(intercepts)

        return slope, intercept

    # ---------- Main learning ----------

    def learn(
        self,
        dT_int_per_min: float,
        u: float,
        t_int: float,
        t_ext: float,
        *,
        max_abs_dT_per_min: float = 0.35,
        min_dT_per_min: float = 0.003,
    ) -> None:
        """
        Update (a,b) using robust regression.
        
        Args:
            dT_int_per_min: dT/dt in °C/min
            u: Duty cycle [0,1]
            t_int: Indoor temp °C
            t_ext: Outdoor temp °C
        """
        dTdt = float(dT_int_per_min)
        delta = float(t_int - t_ext)
        
        # Outlier rejection: slope too large (sensor glitch)
        if abs(dTdt) > max_abs_dT_per_min:
            self.learn_skip_count += 1
            self.learn_last_reason = "skip: slope outlier"
            return

        # Noise rejection: slope too small to be informative
        if abs(dTdt) < min_dT_per_min:
            self.learn_skip_count += 1
            self.learn_last_reason = "skip: slope too small"
            return

        # Residual (model mismatch detector)
        # r = dTdt - (a*u - b*delta)
        # We use current a, b for this check
        r = dTdt - (self.a * u - self.b * delta)
        self._r_hist.append(r)

        mad_r = self._mad(self._r_hist)
        sigma_r = 0.0
        if mad_r is not None:
            sigma_r = 1.4826 * mad_r

        # Only gate if we have some confidence in the model (learned at least a few times)
        if mad_r is not None and self.learn_ok_count > LEARN_BOOTSTRAP_COUNT:
            if abs(r) > RESIDUAL_GATE_K * sigma_r:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: residual outlier"
                return

        # ---------- OFF phase : learn b ----------
        # b is loss coeff. OFF means u ~ 0.
        # dT/dt = -b * delta  =>  -dT/dt = b * delta
        # y = -dT/dt, x = delta. Slope is b.
        if u < 0.05 and abs(delta) >= 0.5:
            y = -dTdt
            self._b_pts.append((delta, y))

            if len(self._b_pts) < 6:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: collecting b points"
                return

            b_hat, c = self._theil_sen(self._b_pts)
            if b_hat is None or b_hat <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: invalid b estimate"
                return

            # Intercept coherence check
            # Ideally linear fit passes through origin.
            # Intercept c represents offset error.
            if self.learn_ok_count > LEARN_BOOTSTRAP_COUNT:
                if mad_r is not None:
                    if abs(c) > INTERCEPT_SIGMA_FACTOR * sigma_r:
                        self.learn_skip_count += 1
                        self.learn_last_reason = "skip: b intercept > sigma"
                        return
                        
                y_vals = [p[1] for p in self._b_pts]
                med_y = abs(statistics.median(y_vals))
                if med_y > 1e-4 and abs(c) > INTERCEPT_SCALE_FACTOR * med_y:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: b intercept scale mismatch"
                    return

            b_hat = clamp(b_hat, self.B_MIN, self.B_MAX)
            self.b = b_hat
            self._b_hat_hist.append(b_hat)
            self.learn_ok_count += 1
            self.learn_ok_count_b += 1
            self.learn_last_reason = "learned b"
            return

        # ---------- ON phase : learn a ----------
        # dT/dt = a*u - b*delta  =>  dT/dt + b*delta = a*u
        # y = dT/dt + b*delta, x = u. Slope is a.
        if u > 0.20 and abs(delta) >= 0.2:
            y = dTdt + self.b * delta
            self._a_pts.append((u, y))

            if len(self._a_pts) < 6:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: collecting a points"
                return

            a_hat, c = self._theil_sen(self._a_pts)
            if a_hat is None or a_hat <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip: invalid a estimate"
                return

            if self.learn_ok_count > LEARN_BOOTSTRAP_COUNT:
                if mad_r is not None:
                    if abs(c) > INTERCEPT_SIGMA_FACTOR * sigma_r:
                        self.learn_skip_count += 1
                        self.learn_last_reason = "skip: a intercept > sigma"
                        return
                
                y_vals = [p[1] for p in self._a_pts]
                med_y = abs(statistics.median(y_vals))
                if med_y > 1e-4 and abs(c) > INTERCEPT_SCALE_FACTOR * med_y:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: a intercept scale mismatch"
                    return

            a_hat = clamp(a_hat, self.A_MIN, self.A_MAX)
            self.a = a_hat
            self.learn_ok_count += 1
            self.learn_ok_count_a += 1
            self.learn_last_reason = "learned a"
            return

        self.learn_skip_count += 1
        self.learn_last_reason = "skip: low excitation"

    def tau_reliability(self) -> TauReliability:
        """
        Check if tau (1/b) is statistically stable and within bounds.
        """
        # Enough updates?
        if self.learn_ok_count_b < 10:
             return TauReliability(reliable=False, tau_min=9999.0)
        
        # Enough history for stability check?
        if len(self._b_hat_hist) < 6:
            # Fallback to simple bounds check if history not full but count ok (should match)
            # But let's require robust history
            return TauReliability(reliable=False, tau_min=9999.0)

        med_b = statistics.median(self._b_hat_hist)
        mad_b = self._mad(self._b_hat_hist)
        
        if mad_b is None or med_b <= 0:
            return TauReliability(reliable=False, tau_min=9999.0)

        # Stability check: Relative dispersion of b estimates
        if (mad_b / med_b) > B_STABILITY_MAD_RATIO_MAX:
             return TauReliability(reliable=False, tau_min=9999.0)
             
        # Value bounds check
        if med_b < self.B_MIN or med_b > self.B_MAX:
            return TauReliability(reliable=False, tau_min=9999.0)

        tau = 1.0 / med_b
        # 10 min to ~66 hours (4000 min)
        reliable = 10.0 <= tau <= 4000.0
        return TauReliability(reliable=reliable, tau_min=tau)



class SmartPI:
    """
    SmartPI Algorithm - Auto-adaptive PI controller for Versatile Thermostat (VTH).

    Public API
    ----------
    - calculate(...): compute the next duty-cycle command in [0,1]
    - get_diagnostics(): return a dict of key internal values for UI/attributes
    - update_learning(...): feed learning data (slope and previously applied u)

    Design intent
    -------------
    - Primary objective: minimize overshoot on setpoint changes (servo),
      accepting a slower approach to target.
    - Keep computational footprint small; no external libs.
    """

    def __init__(
        self,
        # Original integration arguments (positional)
        cycle_min: float,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: Optional[float] = None,
        # Tuning knobs (keyword)
        deadband_c: float = DEFAULT_DEADBAND_C,
        aggressiveness: float = 1.0,  # Note: logic changed, 1.0 is now default
        saved_state: Optional[Dict[str, Any]] = None,
        # --- Feed-forward (FF) progressive enablement ("smart warm-up") ---
        ff_warmup_ok_count: int = 30,
        ff_warmup_cycles: int = 6,
        ff_scale_unreliable_max: float = 0.30,
        # --- Servo overshoot suppression knobs ---
        setpoint_weight_b: float = 0.3,
        near_band_deg: float = 0.30,
        kp_near_factor: float = 0.80,
        ki_near_factor: float = 0.40,
        sign_flip_leak: float = 0.40,
        sign_flip_leak_cycles: int = 3,
        sign_flip_band_mult: float = 2.0,
        use_setpoint_filter: bool = True,
    ) -> None:
        
        self._name = name
        self._cycle_min = float(cycle_min)
        self.deadband_c = float(deadband_c)
        self.aggressiveness = float(aggressiveness)

        self._minimal_activation_delay = int(minimal_activation_delay)
        self._minimal_deactivation_delay = int(minimal_deactivation_delay)
        self._max_on_percent = max_on_percent

        # FF progressive enablement
        self.ff_warmup_ok_count = max(int(ff_warmup_ok_count), 1)
        self.ff_warmup_cycles = max(int(ff_warmup_cycles), 1)
        self.ff_scale_unreliable_max = clamp(float(ff_scale_unreliable_max), 0.0, 1.0)
        self._cycles_since_reset: int = 0

        # 2DOF / scheduling parameters
        self.setpoint_weight_b = clamp(float(setpoint_weight_b), 0.0, 1.0)
        self.near_band_deg = max(float(near_band_deg), 0.0)
        self.kp_near_factor = clamp(float(kp_near_factor), 0.1, 1.0)
        self.ki_near_factor = clamp(float(ki_near_factor), 0.1, 1.0)
        self.sign_flip_leak = clamp(float(sign_flip_leak), 0.0, 1.0)
        self.sign_flip_leak_cycles = max(int(sign_flip_leak_cycles), 0)
        self.sign_flip_band_mult = max(float(sign_flip_band_mult), 0.0)
        self._use_setpoint_filter = use_setpoint_filter

        # Model estimator
        self.est = ABEstimator()

        # PI state
        self.integral: float = 0.0
        self.u_prev: float = 0.0

        # Error filtering (EMA) - kept for diagnostics / potential future use
        self._e_filt: Optional[float] = None
        self._ema_alpha: float = 0.35

        # Current gains
        self.Kp: float = KP_SAFE
        self.Ki: float = KI_SAFE

        # Outputs (duty-cycle and realized timings)
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0

        # Diagnostics / status
        self._last_u_ff: float = 0.0
        self._last_u_pi: float = 0.0
        self._last_error: float = 0.0
        self._last_error_p: float = 0.0
        self._last_i_mode: str = "init"
        self._tau_reliable: bool = False
        self._last_sat: str = "init"

        # Sign-flip leak helper
        self._prev_error: Optional[float] = None
        self._sign_flip_leak_left: int = 0

        # Asymmetric setpoint EMA filter state
        self._filtered_setpoint: Optional[float] = None
        self._last_raw_setpoint: Optional[float] = None
        # Initial temperature when setpoint changed (for midpoint filter activation)
        self._initial_temp_for_filter: Optional[float] = None

        # Track last time calculate() was executed for dt-based integration
        self._last_calculate_time: Optional[datetime] = None
        # Accumulated time for cycle counting (used for FF warm-up)
        self._accumulated_dt: float = 0.0

        # Timestamp for robust learning dt calculation
        self._learn_last_ts: float | None = None

        # Learning start timestamp
        self._learning_start_date: Optional[datetime] = datetime.now()

        # Skip learning cycles after resume from interruption (window close, etc.)
        self._skip_learning_cycles_left: int = 0

        # Deadband state tracking for bumpless transfer on exit
        self._in_deadband: bool = False

        # Tracking anti-windup diagnostics
        self._last_u_cmd: float = 0.0       # command after [0,1] clamp
        self._last_u_limited: float = 0.0   # after rate-limit and max_on_percent
        self._last_u_applied: float = 0.0   # after timing constraints
        self._last_aw_du: float = 0.0       # tracking delta for diagnostics
        self._last_forced_by_timing: bool = False  # True when timing forced 0%/100%

        # Setpoint step boost state (for fast power ramp-up on setpoint change)
        self._setpoint_boost_active: bool = False
        self._prev_setpoint_for_boost: Optional[float] = None

        if saved_state:
            self.load_state(saved_state)
        
        _LOGGER.debug("%s - SmartPI initialized", self._name)

    # ------------------------------
    # Persistence
    # ------------------------------

    def reset_learning(self) -> None:
        """Reset all learned parameters to defaults."""
        self.est.reset()
        self.integral = 0.0
        self.u_prev = 0.0
        self._e_filt = None
        self.Kp = KP_SAFE
        self.Ki = KI_SAFE
        self._cycles_since_reset = 0
        self._accumulated_dt = 0.0
        self._prev_error = None
        self._sign_flip_leak_left = 0
        self._filtered_setpoint = None
        self._last_raw_setpoint = None
        self._initial_temp_for_filter = None
        self._last_calculate_time = None
        self._learn_last_ts = None
        self._learning_start_date = datetime.now()
        self._skip_learning_cycles_left = 0
        self._in_deadband = False
        self._setpoint_boost_active = False
        self._prev_setpoint_for_boost = None
        _LOGGER.info("%s - SmartPI learning and history reset", self._name)

    def notify_resume_after_interruption(self, skip_cycles: int = None) -> None:
        """Notify SmartPI that the thermostat is resuming after an interruption.

        This is called when the thermostat resumes after a window close event
        or similar interruption. It sets the skip counter so that learning
        ignores the first N cycles, allowing temperature to stabilize.

        Args:
            skip_cycles: Number of cycles to skip. Defaults to SKIP_CYCLES_AFTER_RESUME.
        """
        if skip_cycles is None:
            skip_cycles = SKIP_CYCLES_AFTER_RESUME
        self._skip_learning_cycles_left = max(int(skip_cycles), 0)
        # Also reset the learning timestamp to avoid using stale dt
        self._learn_last_ts = None
        _LOGGER.info(
            "%s - SmartPI notified of resume after interruption, skipping %d cycles",
            self._name,
            self._skip_learning_cycles_left
        )

    def load_state(self, state: Dict[str, Any]) -> None:
        """Load persistent state with validation."""
        if not state:
            return

        try:
            # Load estimator state with validation
            a_val = float(state.get("a", 0.0) or 0.0)
            b_val = float(state.get("b", 0.0) or 0.0)

            # Safety check: use init values if out of bounds or NaN
            if math.isnan(a_val) or not (self.est.A_MIN <= a_val <= self.est.A_MAX):
                a_val = self.est.A_INIT
                _LOGGER.warning("%s - Invalid 'a' in saved state, using default", self._name)
            if math.isnan(b_val) or not (self.est.B_MIN <= b_val <= self.est.B_MAX):
                b_val = self.est.B_INIT
                _LOGGER.warning("%s - Invalid 'b' in saved state, using default", self._name)

            self.est.a = a_val
            self.est.b = b_val

            self.est.learn_ok_count = int(state.get("learn_ok_count", 0) or 0)
            self.est.learn_ok_count_a = int(state.get("learn_ok_count_a", 0) or 0)
            self.est.learn_ok_count_b = int(state.get("learn_ok_count_b", 0) or 0)
            self.est.learn_skip_count = int(state.get("learn_skip_count", 0) or 0)

            # Load history queues - Robust estimator uses points (x,y) and residual hist
            a_pts_data = state.get("a_pts", [])
            b_pts_data = state.get("b_pts", [])
            r_hist_data = state.get("r_hist", [])
            b_hat_hist_data = state.get("b_hat_hist", [])

            # Safe loading of tuples for points
            def to_dq_tuples(data, dq_maxlen):
                d = deque(maxlen=dq_maxlen)
                if not data:
                    return d
                for item in data:
                    if isinstance(item, (list, tuple)) and len(item) == 2:
                        d.append(tuple(item))
                return d

            self.est._a_pts = to_dq_tuples(a_pts_data, self.est._a_pts.maxlen)
            self.est._b_pts = to_dq_tuples(b_pts_data, self.est._b_pts.maxlen)
            
            # Simple float queues
            self.est._r_hist = deque(r_hist_data, maxlen=self.est._r_hist.maxlen)
            self.est._b_hat_hist = deque(b_hat_hist_data, maxlen=self.est._b_hat_hist.maxlen)

            # Load PI state with validation
            integral_val = float(state.get("integral", 0.0) or 0.0)
            u_prev_val = float(state.get("u_prev", 0.0) or 0.0)

            if math.isnan(integral_val):
                integral_val = 0.0
                _LOGGER.warning("%s - Invalid 'integral' in saved state, using default", self._name)

            if math.isnan(u_prev_val) or not (0.0 <= u_prev_val <= 1.0):
                u_prev_val = 0.0
                _LOGGER.warning("%s - Invalid 'u_prev' in saved state, using default", self._name)

            self.integral = integral_val
            self.u_prev = u_prev_val
            self._cycles_since_reset = int(state.get("cycles_since_reset", 0) or 0)
            self._accumulated_dt = float(state.get("accumulated_dt", 0.0) or 0.0)

            # Load learning start date
            learning_start = state.get("learning_start_date")
            if learning_start:
                try:
                    self._learning_start_date = datetime.fromisoformat(learning_start)
                except (ValueError, TypeError):
                    self._learning_start_date = None

            # Load setpoint filter state
            filtered_sp = state.get("filtered_setpoint")
            last_raw_sp = state.get("last_raw_setpoint")
            if filtered_sp is not None:
                try:
                    self._filtered_setpoint = float(filtered_sp)
                except (ValueError, TypeError):
                    self._filtered_setpoint = None
            if last_raw_sp is not None:
                try:
                    self._last_raw_setpoint = float(last_raw_sp)
                except (ValueError, TypeError):
                    self._last_raw_setpoint = None

            # Load initial temp for filter midpoint calculation
            initial_temp = state.get("initial_temp_for_filter")
            if initial_temp is not None:
                try:
                    self._initial_temp_for_filter = float(initial_temp)
                except (ValueError, TypeError):
                    self._initial_temp_for_filter = None

            # Load skip cycles counter for resume after interruption
            self._skip_learning_cycles_left = int(state.get("skip_learning_cycles_left", 0) or 0)

            # Load deadband state
            self._in_deadband = bool(state.get("in_deadband", False))

            # Load setpoint boost state
            self._setpoint_boost_active = bool(state.get("setpoint_boost_active", False))
            prev_sp = state.get("prev_setpoint_for_boost")
            if prev_sp is not None:
                try:
                    self._prev_setpoint_for_boost = float(prev_sp)
                except (ValueError, TypeError):
                    self._prev_setpoint_for_boost = None

            # Mark that state was loaded (not fresh init)
            self.est.learn_last_reason = "loaded"

            _LOGGER.debug(
                "%s - SmartPI state loaded: a=%.6f, b=%.6f, learns=%d",
                self._name, self.est.a, self.est.b, self.est.learn_ok_count
            )

        except (TypeError, ValueError) as e:
            _LOGGER.warning("%s - Error loading SmartPI state, using defaults: %s", self._name, e)
            self.est.reset()

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence."""
        return {
            "a": self.est.a,
            "b": self.est.b,
            "learn_ok_count": self.est.learn_ok_count,
            "learn_ok_count_a": self.est.learn_ok_count_a,
            "learn_ok_count_b": self.est.learn_ok_count_b,
            "learn_skip_count": self.est.learn_skip_count,
            "a_pts": list(self.est._a_pts),
            "b_pts": list(self.est._b_pts),
            "r_hist": list(self.est._r_hist),
            "b_hat_hist": list(self.est._b_hat_hist),
            "integral": self.integral,
            "u_prev": self.u_prev,
            "cycles_since_reset": self._cycles_since_reset,
            "accumulated_dt": self._accumulated_dt,
            "learning_start_date": self._learning_start_date.isoformat() if self._learning_start_date else None,
            "filtered_setpoint": self._filtered_setpoint,
            "last_raw_setpoint": self._last_raw_setpoint,
            "initial_temp_for_filter": self._initial_temp_for_filter,
            "skip_learning_cycles_left": self._skip_learning_cycles_left,
            "in_deadband": self._in_deadband,
            "setpoint_boost_active": self._setpoint_boost_active,
            "prev_setpoint_for_boost": self._prev_setpoint_for_boost,
        }

    # ------------------------------
    # Learning entry point
    # ------------------------------

    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
        cycle_dt: float = None,
        ext_previous_temp: float = None,
    ) -> None:
        """
        Update the thermal model with observed data.

        Important:
        - Must use a REAL dt (minutes) matching (previous_temp -> current_temp).
        - Never assume dt == self._cycle_min unless we are sure this call is truly "once per cycle".
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            return

        # Skip learning cycles after resume from interruption (e.g., window close)
        if self._skip_learning_cycles_left > 0:
            self._skip_learning_cycles_left -= 1
            self.est.learn_last_reason = f"skip:resume({self._skip_learning_cycles_left + 1} left)"
            _LOGGER.debug(
                "%s - Skipping learning cycle after resume (%d cycles left)",
                self._name,
                self._skip_learning_cycles_left
            )
            return

        # Ensure u is always in [0..1]
        previous_power = clamp(float(previous_power), 0.0, 1.0)

        # Determine dt_minutes robustly
        dt_minutes = None

        # 1) Prefer explicit cycle_dt if provided and plausible
        if cycle_dt is not None:
            try:
                dt_candidate = float(cycle_dt)
            except (TypeError, ValueError):
                dt_candidate = None

            if dt_candidate is not None and dt_candidate > 1e-4:
                dt_minutes = dt_candidate

        # 2) Otherwise compute real dt from timestamp
        now_ts = time.time()
        if dt_minutes is None:
            if self._learn_last_ts is None:
                # First call: we cannot infer dt reliably
                self._learn_last_ts = now_ts
                self.est.learn_last_reason = "skip:dt:init"
                return

            dt_minutes = (now_ts - self._learn_last_ts) / 60.0
            self._learn_last_ts = now_ts

            # Guardrails: ignore too small/too large windows
            DT_MIN_OK = 0.5   # 30 s (avoid noise)
            DT_MAX_OK = 30.0  # 30 min (avoid long gaps)
            if dt_minutes < DT_MIN_OK:
                self.est.learn_last_reason = f"skip:dt:too_small({dt_minutes:.2f}m)"
                return
            if dt_minutes > DT_MAX_OK:
                self.est.learn_last_reason = f"skip:dt:too_large({dt_minutes:.2f}m)"
                return
        else:
            # If cycle_dt is provided, still keep timestamp in sync
            self._learn_last_ts = now_ts

        # External temperature at start of observation window
        t_ext_prev = ext_previous_temp if ext_previous_temp is not None else ext_current_temp
        if t_ext_prev is None:
            # Don't fake it with previous_temp (would force delta=0 => b not learnable)
            self.est.learn_last_reason = "skip:ext:none"
            return

        dT = float(current_temp) - float(previous_temp)
        dT_int_per_min = dT / dt_minutes

        self.est.learn(
            dT_int_per_min=dT_int_per_min,
            u=previous_power,
            t_int=float(previous_temp),
            t_ext=float(t_ext_prev),
        )

    # ------------------------------
    # Props / API Compat
    # ------------------------------
    
    @property
    def a(self) -> float:
        return self.est.a

    @property
    def b(self) -> float:
        return self.est.b

    @property
    def on_percent(self) -> float:
        return self._on_percent
        
    @property
    def calculated_on_percent(self) -> float:
        return self._on_percent

    @property
    def on_time_sec(self) -> int:
        return self._on_time_sec

    @property
    def off_time_sec(self) -> int:
        return self._off_time_sec
        
    @property
    def tpi_coef_int(self) -> float:
        return self.Kp

    @property
    def tpi_coef_ext(self) -> float:
        k_ext = self.est.b / max(self.est.a, 1e-6) if self.est.a > 1e-6 else 0
        return k_ext

    @property
    def integral_error(self) -> float:
        """Current integral accumulator value."""
        return self.integral

    @property
    def u_ff(self) -> float:
        """Last feed-forward value."""
        return self._last_u_ff

    # ------------------------------
    # Asymmetric setpoint filter
    # ------------------------------

    def _filter_setpoint(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode: VThermHvacMode,
        advance_ema: bool = True
    ) -> float:
        """
        Apply asymmetric EMA filter to setpoint with midpoint activation.

        Behavior:
        - HEAT mode: increases are filtered (slow ramp), decreases are instant
        - COOL mode: decreases are filtered (slow ramp), increases are instant
        - Midpoint activation: filter only activates once current_temp reaches the
          midpoint between initial temperature and target. This allows full power
          for the first half of the temperature rise, then smooth landing.

        Args:
            target_temp: The target temperature (raw setpoint)
            current_temp: Current room temperature
            hvac_mode: Current HVAC mode
            advance_ema: If True, advance the EMA toward target. If False, only
                         detect and handle setpoint changes (for rate-limited calls).
        """
        # First call or no previous setpoint: initialize
        if self._filtered_setpoint is None or self._last_raw_setpoint is None:
            self._filtered_setpoint = target_temp
            self._last_raw_setpoint = target_temp
            self._initial_temp_for_filter = None
            return target_temp

        # Detect if the RAW setpoint has changed since last cycle
        setpoint_changed = abs(target_temp - self._last_raw_setpoint) > 0.01

        if setpoint_changed:
            # Setpoint just changed - determine direction and action
            if hvac_mode == VThermHvacMode_HEAT:
                should_filter = target_temp > self._last_raw_setpoint
            elif hvac_mode == VThermHvacMode_COOL:
                should_filter = target_temp < self._last_raw_setpoint
            else:
                # OFF or unknown mode: no filtering
                self._filtered_setpoint = target_temp
                self._last_raw_setpoint = target_temp
                self._initial_temp_for_filter = None
                return target_temp

            if not should_filter:
                # Instant follow for decrease (energy saving)
                self._filtered_setpoint = target_temp
                self._last_raw_setpoint = target_temp
                self._initial_temp_for_filter = None
                return target_temp

            # Record initial temperature for midpoint calculation
            self._initial_temp_for_filter = current_temp
            self._last_raw_setpoint = target_temp

            # Initially, follow the raw setpoint (no filtering yet)
            # Filtering will only start once we reach the midpoint
            self._filtered_setpoint = target_temp
            return target_temp

        # Setpoint unchanged - check if filter should be active based on midpoint
        if self._initial_temp_for_filter is None or current_temp is None:
            # No initial temp recorded or no current temp, just follow setpoint
            self._filtered_setpoint = target_temp
            return target_temp

        # Calculate midpoint between initial temperature and target
        midpoint = (self._initial_temp_for_filter + target_temp) / 2.0

        # Check if we have reached the midpoint (direction-aware)
        if hvac_mode == VThermHvacMode_HEAT:
            reached_midpoint = current_temp >= midpoint
        elif hvac_mode == VThermHvacMode_COOL:
            reached_midpoint = current_temp <= midpoint
        else:
            reached_midpoint = False

        if not reached_midpoint:
            # Still in the "full power" phase - follow raw setpoint
            self._filtered_setpoint = target_temp
            return target_temp

        # We have reached the midpoint - now apply EMA filtering for smooth landing
        gap = abs(target_temp - self._filtered_setpoint)
        if gap <= 0.02:
            # Converged - snap to target and clear initial temp
            self._filtered_setpoint = target_temp
            self._initial_temp_for_filter = None
            return target_temp

        if advance_ema:
            w = min(gap / SP_BAND, 1.0)
            alpha = SP_ALPHA_SLOW + (SP_ALPHA_FAST - SP_ALPHA_SLOW) * w
            self._filtered_setpoint = alpha * target_temp + (1 - alpha) * self._filtered_setpoint

        return self._filtered_setpoint

    # ------------------------------
    # Main control law
    # ------------------------------

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
        Compute the next duty-cycle command.

        Notes
        -----
        - slope parameter is accepted for API compatibility (unused for now).
        - integrator_hold can be used during dead time / actuator constraints to avoid pumping.

        Side effects:
        - Updates internal duty-cycle (_on_percent) and ON/OFF timings
        - Updates diagnostics and PI state

        Dynamic dt integration: This method can be called on every temperature sensor
        update. The integral term is accumulated based on actual elapsed time (dt_min),
        making it robust to irregular call intervals. A minimum dt threshold prevents
        excessive noise from very rapid calls.
        """
        now = datetime.now()

        # Input validation
        if target_temp is None or current_temp is None:
            self._on_percent = 0.0
            self.u_prev = 0.0
            self._calculate_times()
            return

        if hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0.0
            self.u_prev = 0.0
            # Reset rate-limiting so next HEAT/COOL activation gets immediate calculation
            self._last_calculate_time = None
            self._calculate_times()
            return

        # Calculate elapsed time since last calculation for dt-based integration
        if self._last_calculate_time is not None:
            dt_min = (now - self._last_calculate_time).total_seconds() / 60.0
        else:
            # First call: use cycle_min as initial approximation
            dt_min = self._cycle_min

        # Minimum dt threshold to prevent noise from very rapid calls (< 3 seconds)
        MIN_DT_SECONDS = 3.0
        if dt_min < MIN_DT_SECONDS / 60.0:
            # Too soon since last calculation - keep existing outputs
            # BUT still capture setpoint changes to not miss user adjustments
            self._filter_setpoint(target_temp, current_temp, hvac_mode, advance_ema=False)
            self._calculate_times()
            return

        # Update timestamp for next dt calculation
        self._last_calculate_time = now

        # Count cycles (approximate) for FF warm-up logic
        # Increment when we've accumulated roughly one cycle worth of time
        self._accumulated_dt += dt_min
        if self._accumulated_dt >= self._cycle_min:
            self._cycles_since_reset += 1
            self._accumulated_dt -= self._cycle_min

        # Get model parameters
        a = self.est.a
        b = self.est.b

        # Check tau reliability
        tau_info = self.est.tau_reliability()
        self._tau_reliable = tau_info.reliable

        # Compute gains (simple heuristic based on tau)
        if tau_info.reliable:
            tau = tau_info.tau_min  # minutes
            # Heuristic: Kp scales with tau, Ki = Kp/tau
            kp_calc = 0.35 + 0.9 * math.sqrt(tau / 200.0)
            kp = clamp(kp_calc, KP_MIN, KP_MAX)
            # Normal Ki based on tau (conservative, avoids overshoot when far from target)
            ki = clamp(kp / max(tau, 10.0), KI_MIN, KI_MAX)
        else:
            kp = KP_SAFE
            ki = KI_SAFE

        # Apply global aggressiveness
        kp *= max(self.aggressiveness, 0.0)
        ki *= max(self.aggressiveness, 0.0)

        # Apply asymmetric setpoint filter to reduce overshoot on setpoint changes
        # Always call _filter_setpoint() to keep _filtered_setpoint synchronized for diagnostics
        # But only use the filtered value for PI control when tau is reliable
        # advance_ema=True here because we're in the main PI loop (once per cycle)
        filtered_result = self._filter_setpoint(target_temp, current_temp, hvac_mode, advance_ema=True)
        
        if self._use_setpoint_filter and self._tau_reliable:
            target_temp_internal = filtered_result
        else:
            # Use raw target for PI control but _filtered_setpoint is still updated for diagnostics
            target_temp_internal = target_temp

        # Compute errors using the filtered setpoint
        e = float(target_temp_internal - current_temp)
        # Invert error for COOL mode
        if hvac_mode == VThermHvacMode_COOL:
            e = -e

        self._last_error = e

        # 2DOF (setpoint weighting) for proportional action:
        # e_p = b_sp * e, with e = (target_temp - current_temp)
        # This reduces the proportional kick on setpoint changes while preserving error sign
        e_p = float(self.setpoint_weight_b * e)
        # if hvac_mode == VThermHvacMode_COOL:
        #     # Cooling mode: invert the weighted error
        #     e_p = -e_p

        self._last_error_p = e_p

        # Sign-flip detection (for optional integral discharge)
        if self._prev_error is not None and (e * self._prev_error) < 0.0:
            # Only apply if we are close enough to setpoint (avoid messing with large disturbances)
            band = self.sign_flip_band_mult * max(self.near_band_deg, 1e-6)
            if abs(e) <= band and self.sign_flip_leak_cycles > 0 and self.sign_flip_leak > 0.0:
                self._sign_flip_leak_left = self.sign_flip_leak_cycles
        self._prev_error = e

        # Near-setpoint gain scheduling (soft landing)
        in_near_band = (self.near_band_deg > 0.0) and (abs(e) <= self.near_band_deg)
        if in_near_band:
            kp_base = kp  # Save original Kp before reduction

            # Reduce Kp for softer proportional action
            kp = kp_base * self.kp_near_factor

            # Recalculate Ki from ORIGINAL Kp (kp_base), not reduced Kp
            # This avoids double attenuation (kp_near_factor * ki_near_factor)
            if tau_info.reliable:
                tau_capped = clamp(tau_info.tau_min, 10.0, TAU_CAP_FOR_KI)
                ki = clamp(kp_base / tau_capped, KI_MIN, KI_MAX)

            # Apply attenuation factor to Ki
            ki *= self.ki_near_factor

            # Re-clamp to ensure gains stay within bounds after reduction
            kp = clamp(kp, KP_MIN, KP_MAX)
            ki = clamp(ki, KI_MIN, KI_MAX)

        # Store current gains (for diagnostics)
        self.Kp = kp
        self.Ki = ki

        # EMA filtering of error (for diagnostics / future options)
        if self._e_filt is None:
            self._e_filt = e
        else:
            self._e_filt = (1 - self._ema_alpha) * self._e_filt + self._ema_alpha * e

        # Feed-forward calculation
        # Use target_temp_internal for coherence with the PI controller (filtered setpoint)
        # For HEAT: positive command; For COOL: invert sign of error/FF (very simplistic)
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        if a < 2e-4:
            u_ff = 0.0
        else:
            k_ff = clamp(b / a, 0.0, 3.0)
            u_ff = clamp(k_ff * (target_temp_internal - t_ext), 0.0, 1.0)

        if hvac_mode == VThermHvacMode_COOL:
            # Cooling: map to "cooling effort" (this is kept for compatibility; tune as needed)
            u_ff = 0.0  # FF is usually not valid for COOL in this simple model

        # Progressive "smart" FF warm-up:
        # - Scale up with learning confidence (learn_ok_count)
        # - Scale up with time since start (cycles)
        # - Cap when the model is not reliable yet (tau check)
        learn_scale = clamp(self.est.learn_ok_count / float(self.ff_warmup_ok_count), 0.0, 1.0)
        time_scale = clamp(self._cycles_since_reset / float(self.ff_warmup_cycles), 0.0, 1.0)
        reliable_cap = 1.0 if self._tau_reliable else self.ff_scale_unreliable_max
        ff_scale = clamp(reliable_cap * learn_scale * time_scale, 0.0, 1.0)
        u_ff *= ff_scale

        self._last_u_ff = u_ff

        # Dynamic integral limit: bound so |Ki * I| <= 2.0
        i_max = 2.0 / max(self.Ki, KI_MIN)

        # Optional "sign flip leak" soft discharge
        if self._sign_flip_leak_left > 0:
            self.integral *= (1.0 - self.sign_flip_leak)
            self._sign_flip_leak_left -= 1
            # Keep integral bounded
            self.integral = clamp(self.integral, -i_max, i_max)

        # --- PI control with anti-windup ---
        # Deadband with hysteresis to reduce oscillations at boundary:
        # - Enter deadband when |e| < deadband_c
        # - Exit deadband only when |e| > deadband_c + DEADBAND_HYSTERESIS
        # - In the hysteresis zone, maintain previous state
        abs_e = abs(e)
        db_entry = self.deadband_c
        db_exit = self.deadband_c + DEADBAND_HYSTERESIS

        if abs_e < db_entry:
            # Clearly inside deadband
            in_deadband_now = True
        elif abs_e > db_exit:
            # Clearly outside deadband
            in_deadband_now = False
        else:
            # In hysteresis zone: maintain previous state
            in_deadband_now = self._in_deadband

        # Bumpless transfer: on deadband exit, re-initialize integral
        # so that u_ff + Kp*e_p + Ki*I = u_prev (no output discontinuity)
        if self._in_deadband and not in_deadband_now:
            # Exiting deadband: compute integral for bumpless transfer
            if self.Ki > KI_MIN:
                # I_new = (u_prev - u_ff - Kp * e_p) / Ki
                i_bumpless = (self.u_prev - u_ff - self.Kp * e_p) / self.Ki
                self.integral = clamp(i_bumpless, -i_max, i_max)
                _LOGGER.debug(
                    "%s - Bumpless deadband exit: I adjusted to %.4f (u_prev=%.3f)",
                    self._name, self.integral, self.u_prev
                )

        # Update deadband state
        self._in_deadband = in_deadband_now

        if in_deadband_now:
            # In deadband: leak integral slowly and output zero (helps avoid chatter)
            self.integral *= INTEGRAL_LEAK
            self.integral = clamp(self.integral, -i_max, i_max)
            u_pi = 0.0
            self._last_i_mode = "I:LEAK(deadband)"
        else:
            if integrator_hold:
                # Dead time: don't integrate to avoid pumping
                u_pi = self.Kp * e_p + self.Ki * self.integral
                self._last_i_mode = "I:HOLD"
            else:
                # Preview output without updating integral
                u_pi_pre = self.Kp * e_p + self.Ki * self.integral
                u_raw_pre = u_ff + u_pi_pre

                if u_raw_pre > 1.0:
                    sat_state = "SAT_HI"
                elif u_raw_pre < 0.0:
                    sat_state = "SAT_LO"
                else:
                    sat_state = "NO_SAT"
                self._last_sat = sat_state

                # Conditional integration:
                # Skip integration if saturated AND error would make it worse.
                # IMPORTANT: windup is determined by the *integrator-driving error* (e), not e_p.
                if (sat_state == "SAT_HI" and e > 0) or (sat_state == "SAT_LO" and e < 0):
                    self._last_i_mode = f"I:SKIP({sat_state})"
                    u_pi = u_pi_pre
                else:
                    # Normal integration using actual elapsed time (dt_min)
                    self.integral += e * dt_min
                    self.integral = clamp(self.integral, -i_max, i_max)
                    self._last_i_mode = "I:RUN"
                    u_pi = self.Kp * e_p + self.Ki * self.integral

        # Combine FF and PI, then clamp to [0,1]
        u_raw = u_ff + u_pi
        u_cmd = clamp(u_raw, 0.0, 1.0)

        # Store PI term for diagnostics
        self._last_u_pi = u_pi
        self._last_u_cmd = u_cmd

        # ------------------------------
        # Apply constraints (rate-limit, max_on_percent), then timing enforcement
        # ------------------------------

        # Setpoint step boost: detect significant setpoint increase (HEAT) or decrease (COOL)
        # and apply faster rate limit to allow quick power ramp-up
        if self._prev_setpoint_for_boost is None:
            self._prev_setpoint_for_boost = target_temp

        sp_delta = target_temp - self._prev_setpoint_for_boost

        # Detect setpoint change that should trigger boost
        if hvac_mode == VThermHvacMode_HEAT and sp_delta >= SETPOINT_BOOST_THRESHOLD:
            # Setpoint increased in HEAT mode - activate boost
            self._setpoint_boost_active = True
            self._prev_setpoint_for_boost = target_temp
            _LOGGER.debug(
                "%s - Setpoint boost activated: setpoint +%.2f°C",
                self._name, sp_delta
            )
        elif hvac_mode == VThermHvacMode_COOL and sp_delta <= -SETPOINT_BOOST_THRESHOLD:
            # Setpoint decreased in COOL mode - activate boost
            self._setpoint_boost_active = True
            self._prev_setpoint_for_boost = target_temp
            _LOGGER.debug(
                "%s - Setpoint boost activated: setpoint %.2f°C",
                self._name, sp_delta
            )
        elif abs(sp_delta) > 0.01:
            # Setpoint changed but not in boost direction - just track it
            self._prev_setpoint_for_boost = target_temp
            self._setpoint_boost_active = False

        # Deactivate boost when error becomes small enough
        if self._setpoint_boost_active and abs(e) < SETPOINT_BOOST_ERROR_MIN:
            self._setpoint_boost_active = False
            _LOGGER.debug(
                "%s - Setpoint boost deactivated: error %.3f°C < threshold",
                self._name, abs(e)
            )

        # Select rate limit based on boost state
        if self._setpoint_boost_active:
            rate_limit = SETPOINT_BOOST_RATE
        else:
            rate_limit = MAX_STEP_PER_MINUTE

        # Rate limiting: bound command delta per cycle (prevents abrupt power changes)
        max_step = rate_limit * dt_min
        u_limited = clamp(u_cmd, self.u_prev - max_step, self.u_prev + max_step)

        # Apply max_on_percent limit if configured
        if self._max_on_percent is not None and u_limited > self._max_on_percent:
            u_limited = self._max_on_percent

        self._last_u_limited = u_limited

        # Feed the timing layer with requested duty-cycle; it may enforce 0%/100%
        # due to min ON/OFF constraints.
        self._on_percent = u_limited

        # Update timings; this function may change self._on_percent
        self._calculate_times()

        # What is actually realized after timing enforcement
        u_applied = self._on_percent
        self._last_u_applied = u_applied

        # ------------------------------
        # Tracking anti-windup (back-calculation)
        #
        # Rationale: conditional integration prevents windup on [0..1] saturation,
        # but does NOT account for extra constraints (rate limit, max_on_percent,
        # min ON/OFF timing) that make applied command differ from PI+FF model.
        # This step re-aligns the integrator to the applied actuator command.
        # Only apply when NOT in deadband (deadband uses INTEGRAL_LEAK instead).
        # ------------------------------

        # Detect if timing constraints forced an extreme value (Option A fix).
        # When _calculate_times() pushes _on_percent to 0 or 1 due to min ON/OFF
        # constraints while u_limited was in a "reasonable" range, we should NOT
        # inject this artificial delta into the integral. This prevents integral
        # windup caused by timing quantization artifacts (e.g., min_off_delay
        # forcing 100% when u_limited was ~89%).
        forced_by_timing = (
            (u_applied == 0.0 and u_limited > 0.01) or  # min_on_delay forced 0%
            (u_applied == 1.0 and u_limited < 0.99)     # min_off_delay forced 100%
        )
        self._last_forced_by_timing = forced_by_timing

        if (not integrator_hold) and (self.Ki > KI_MIN) and (not self._in_deadband):
            if forced_by_timing:
                # Skip tracking: timing quantization should not influence integral
                self._last_aw_du = 0.0
            else:
                # Model-predicted command using current integrator state
                u_model = u_ff + (self.Kp * e_p + self.Ki * self.integral)

                # Tracking error between applied command and model command
                du = u_applied - u_model
                self._last_aw_du = du

                # Discrete tracking gain beta = dt / Tt (bounded 0..1)
                dt_sec = dt_min * 60.0
                beta = clamp(dt_sec / max(AW_TRACK_TAU_S, dt_sec), 0.0, 1.0)

                # Update integral so that Ki * I compensates du
                d_integral = beta * (du / self.Ki)

                # Safety clamp (per cycle)
                d_integral = clamp(d_integral, -AW_TRACK_MAX_DELTA_I, AW_TRACK_MAX_DELTA_I)

                self.integral += d_integral
                self.integral = clamp(self.integral, -i_max, i_max)
        else:
            self._last_aw_du = 0.0

        # Store final applied command for next cycle rate-limiting
        self.u_prev = u_applied

    # ------------------------------
    # Timings and diagnostics
    # ------------------------------

    def _calculate_times(self) -> None:
        """Compute ON/OFF times from duty-cycle, respecting minimal ON/OFF delays."""
        cycle_sec = int(round(self._cycle_min * 60.0))
        cycle_sec = max(cycle_sec, 0)

        # Base
        self._on_time_sec = int(round(self._on_percent * cycle_sec))

        # Enforce minimal activation duration
        if self._on_time_sec < self._minimal_activation_delay:
            self._on_time_sec = 0

        self._off_time_sec = cycle_sec - self._on_time_sec

        # Enforce minimal deactivation duration
        if self._off_time_sec < self._minimal_deactivation_delay:
            self._on_time_sec = cycle_sec
            self._off_time_sec = 0

        # CRITICAL: resync on_percent with realized timing after constraints
        if cycle_sec > 0:
            self._on_percent = self._on_time_sec / cycle_sec
        else:
            self._on_percent = 0.0

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information (suitable for attributes/UI)."""
        tau_info = self.est.tau_reliability()

        return {
            # Model
            "a": round(self.est.a, 6),
            "b": round(self.est.b, 6),
            "tau_min": round(tau_info.tau_min, 1),
            "tau_reliable": tau_info.reliable,
            "learn_ok_count": int(self.est.learn_ok_count),
            "learn_ok_count_a": int(self.est.learn_ok_count_a),
            "learn_ok_count_b": int(self.est.learn_ok_count_b),
            "learn_skip_count": int(self.est.learn_skip_count),
            "learn_last_reason": str(self.est.learn_last_reason),
            # Learning metadata
            "learning_start_dt": self._learning_start_date,
            # PI
            "Kp": round(self.Kp, 6),
            "Ki": round(self.Ki, 6),
            "integral_error": round(self.integral, 6),
            "i_mode": self._last_i_mode,
            "sat": self._last_sat,
            # Errors
            "error": round(self._last_error, 4),
            "error_p": round(self._last_error_p, 4),
            "error_filtered": None if self._e_filt is None else round(self._e_filt, 4),
            # 2DOF/scheduling
            "setpoint_weight_b": round(self.setpoint_weight_b, 3),
            "near_band_deg": round(self.near_band_deg, 3),
            "kp_near_factor": round(self.kp_near_factor, 3),
            "ki_near_factor": round(self.ki_near_factor, 3),
            "sign_flip_leak": round(self.sign_flip_leak, 3),
            "sign_flip_leak_left": int(self._sign_flip_leak_left),
            # Output
            "u_ff": round(self._last_u_ff, 6),
            "u_pi": round(self._last_u_pi, 6),
            "ff_warmup_ok_count": int(self.ff_warmup_ok_count),
            "ff_warmup_cycles": int(self.ff_warmup_cycles),
            "ff_scale_unreliable_max": round(self.ff_scale_unreliable_max, 3),
            "cycles_since_reset": int(self._cycles_since_reset),
            "on_percent": round(self._on_percent, 6),
            "on_time_sec": int(self._on_time_sec),
            "off_time_sec": int(self._off_time_sec),
            "cycle_min": round(self._cycle_min, 3),
            # Setpoint filter
            "filtered_setpoint": None if self._filtered_setpoint is None else round(self._filtered_setpoint, 2),
            # Resume skip
            "skip_learning_cycles_left": int(self._skip_learning_cycles_left),
            # Anti-windup tracking diagnostics
            "u_cmd": round(self._last_u_cmd, 6),
            "u_limited": round(self._last_u_limited, 6),
            "u_applied": round(self._last_u_applied, 6),
            "aw_du": round(self._last_aw_du, 6),
            "forced_by_timing": self._last_forced_by_timing,
            # Deadband state
            "in_deadband": self._in_deadband,
            # Setpoint boost state
            "setpoint_boost_active": self._setpoint_boost_active,
        }
