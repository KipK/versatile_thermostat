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
MAX_STEP_PER_CYCLE = 0.15  # max output change per minute (rate limit)

# Skip cycles after resume from interruption (window, etc.)
SKIP_CYCLES_AFTER_RESUME = 1

# Default deadband around setpoint ("C)
DEFAULT_DEADBAND_C = 0.05

# Asymmetric setpoint EMA filter parameters
SP_ALPHA_SLOW = 0.05   # EMA alpha for small setpoint increases
SP_ALPHA_FAST = 0.40   # EMA alpha for large setpoint jumps
SP_BAND = 1.0          # Band for alpha interpolation (°C)
SP_BYPASS_ERROR_THRESHOLD = 0.8  # Bypass filter when error > this (°C) to avoid slow heating


@dataclass(frozen=True)
class TauReliability:
    """Result of tau (time constant) reliability check."""
    reliable: bool
    tau_min: float  # minutes (min of candidates used)


@dataclass
class ABEstimator:
    """
    Online estimator for a and b in:

        dT_int approx a*u - b*(T_int - T_ext)

    Learning strategy (ON/OFF separation):
    - b is learned during OFF phases (u < 0.05) when cooling is observable
    - a is learned during ON phases (u > 0.20) when heating is significant
    - Gray zone (0.05 <= u <= 0.20) is skipped as data quality is poor

    This separation avoids circular coupling between a and b estimates.
    """

    # Initial values for faster convergence (matching regul6.py)
    # A_INIT: heating effectiveness ~0.05°C/min at 100% for typical radiator
    # B_INIT: loss coefficient giving tau ~ 1000 min (reasonable room)
    A_INIT: float = 0.0005
    B_INIT: float = 0.0010

    a: float = 0.0005  # = A_INIT
    b: float = 0.0010  # = B_INIT

    # Learning diagnostics
    learn_ok_count: int = 0
    learn_ok_count_a: int = 0
    learn_ok_count_b: int = 0
    learn_skip_count: int = 0
    learn_last_reason: str = "init"

    # Internal windows for robust median estimation
    _a_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=96))
    _b_hist: Deque[float] = field(default_factory=lambda: deque(maxlen=96))

    # Robust bounds (conservative; tune to your installation)
    A_MIN: float = 1e-5
    A_MAX: float = 0.05
    B_MIN: float = 1e-5
    B_MAX: float = 0.05

    # EWMA smoothing factor for parameter updates
    ALPHA_A: float = 0.15
    ALPHA_B: float = 0.12

    def reset(self) -> None:
        """Reset learned parameters and history to initial values."""
        self.a = self.A_INIT
        self.b = self.B_INIT
        self.learn_ok_count = 0
        self.learn_ok_count_a = 0
        self.learn_ok_count_b = 0
        self.learn_skip_count = 0
        self.learn_last_reason = "reset"
        self._a_hist.clear()
        self._b_hist.clear()


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
        Update (a,b) using a single observation with ON/OFF separation.

        Parameters
        ----------
        dT_int_per_min:
            Indoor temperature slope over the last cycle (°C/min)
        u:
            Applied power duty-cycle over the last cycle in [0,1]
        t_int:
            Current indoor temperature (°C)
        t_ext:
            External/ambient temperature (°C)
        max_abs_dT_per_min:
            Outlier rejection threshold for impossible slopes
        min_dT_per_min:
            Minimum slope magnitude to consider (below = noise)
        """
        dt = float(dT_int_per_min)
        delta = float(t_int - t_ext)

        # Outlier rejection: slope too large (sensor glitch)
        if abs(dt) > max_abs_dT_per_min:
            self.learn_skip_count += 1
            self.learn_last_reason = "skip: slope outlier"
            return

        # Noise rejection: slope too small to be informative
        if abs(dt) < min_dT_per_min:
            self.learn_skip_count += 1
            self.learn_last_reason = "skip: slope too small (noise)"
            return

        # === PHASE OFF: Learn b when u < 0.05 ===
        # Model simplifies to: dT/dt = -b * (T_int - T_ext)
        # => b = -dT/dt / (T_int - T_ext)
        if u < 0.05:
            if abs(delta) < 0.5:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip:b: |Tin-Text| too small"
                return

            b_meas = -dt / delta

            if b_meas <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip:b: measured b <= 0"
                return

            # Store raw measurement for median calculation (no clamp here)
            # Clamping before median would bias it toward bounds
            self._b_hist.append(b_meas)

            # Update b using EWMA with median for robustness
            if len(self._b_hist) >= 3:
                b_median = float(statistics.median(self._b_hist))
                self.b = clamp(
                    (1 - self.ALPHA_B) * self.b + self.ALPHA_B * b_median,
                    self.B_MIN,
                    self.B_MAX,
                )
            else:
                # Warmup: direct EWMA
                self.b = clamp(
                    (1 - self.ALPHA_B) * self.b + self.ALPHA_B * b_meas,
                    self.B_MIN,
                    self.B_MAX,
                )

            self.learn_ok_count += 1
            self.learn_ok_count_b += 1
            self.learn_last_reason = "update:b(off)"
            return

        # === PHASE ON: Learn a when u > 0.20 ===
        # Using known b: a = (dT/dt + b * (T_int - T_ext)) / u
        if u > 0.20:
            if abs(delta) < 0.2:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip:a: |Tin-Text| too small"
                return

            a_meas = (dt + self.b * delta) / u

            if a_meas <= 0:
                self.learn_skip_count += 1
                self.learn_last_reason = "skip:a: measured a <= 0"
                return

            # Store raw measurement for median calculation (no clamp here)
            # Clamping before median would bias it toward bounds
            self._a_hist.append(a_meas)

            # Update a using EWMA with median for robustness
            if len(self._a_hist) >= 3:
                a_median = float(statistics.median(self._a_hist))
                self.a = clamp(
                    (1 - self.ALPHA_A) * self.a + self.ALPHA_A * a_median,
                    self.A_MIN,
                    self.A_MAX,
                )
            else:
                # Warmup: direct EWMA
                self.a = clamp(
                    (1 - self.ALPHA_A) * self.a + self.ALPHA_A * a_meas,
                    self.A_MIN,
                    self.A_MAX,
                )

            self.learn_ok_count += 1
            self.learn_ok_count_a += 1
            self.learn_last_reason = "update:a(on)"
            return

        # === GRAY ZONE: 0.05 <= u <= 0.20 ===
        # Data quality is poor, skip learning
        self.learn_skip_count += 1
        self.learn_last_reason = "skip: gray zone (0.05 <= u <= 0.20)"

    def tau_reliability(self) -> TauReliability:
        """
        Determine whether the estimated tau (=1/b) is reliable.

        Heuristic:
        - Need enough valid samples
        - b must be in a plausible range
        - tau must be within reasonable bounds

        Returns
        -------
        TauReliability
        """
        if self.learn_ok_count_b < 10:
            return TauReliability(reliable=False, tau_min=9999.0)

        b = self.b
        if b < self.B_MIN or b > self.B_MAX:
            return TauReliability(reliable=False, tau_min=9999.0)

        tau = 1.0 / max(b, 1e-9)  # minutes
        # Plausible thermal tau for a room is usually tens to hundreds of minutes.
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
        near_band_deg: float = 0.50,
        kp_near_factor: float = 0.60,
        ki_near_factor: float = 0.85,
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

            # Load history queues
            a_hist = state.get("a_hist", [])
            b_hist = state.get("b_hist", [])

            # Reconstruct deque objects
            self.est._a_hist = deque(a_hist, maxlen=self.est._a_hist.maxlen)
            self.est._b_hist = deque(b_hist, maxlen=self.est._b_hist.maxlen)

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
            "a_hist": list(self.est._a_hist),
            "b_hist": list(self.est._b_hist),
            "integral": self.integral,
            "u_prev": self.u_prev,
            "cycles_since_reset": self._cycles_since_reset,
            "accumulated_dt": self._accumulated_dt,
            "learning_start_date": self._learning_start_date.isoformat() if self._learning_start_date else None,
            "filtered_setpoint": self._filtered_setpoint,
            "last_raw_setpoint": self._last_raw_setpoint,
            "initial_temp_for_filter": self._initial_temp_for_filter,
            "skip_learning_cycles_left": self._skip_learning_cycles_left,
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

            # Recalculate Ki from the ORIGINAL Kp (avoids double reduction)
            if tau_info.reliable:
                tau_capped = clamp(tau_info.tau_min, 10.0, TAU_CAP_FOR_KI)
                ki = clamp(kp / tau_capped, KI_MIN, KI_MAX)

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
        # For HEAT: positive command; For COOL: invert sign of error/FF (very simplistic)
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        if a < 2e-4:
            u_ff = 0.0
        else:
            k_ff = clamp(b / a, 0.0, 3.0)
            u_ff = clamp(k_ff * (target_temp - t_ext), 0.0, 1.0)

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
        if abs(e) < self.deadband_c:
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
        u = clamp(u_raw, 0.0, 1.0)
        
        # Store u_pi for diagnostics
        self._last_u_pi = u_pi

        # Rate limiting: bound command delta per cycle (prevents abrupt power changes)
        max_step = MAX_STEP_PER_CYCLE
        u = clamp(u, self.u_prev - max_step, self.u_prev + max_step)

        # Apply max_on_percent limit if configured
        if self._max_on_percent is not None and u > self._max_on_percent:
            u = self._max_on_percent

        # Store final command
        self.u_prev = u
        self._on_percent = u

        # Update timings, ensuring percent matches the enforced timings
        self._calculate_times()

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
        }
