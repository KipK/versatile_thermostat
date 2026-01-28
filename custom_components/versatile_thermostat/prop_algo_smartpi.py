########################################################################
#                                                                      #
#                      Smart-PI Algorithm                              #
#                      ------------------                              #
#  Auto-adaptive PI controller for Versatile Thermostat (VTH).         #
#                                                                      #
########################################################################

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
from typing import Any, Deque, Dict, Optional, Tuple
from enum import Enum

from .vtherm_hvac_mode import (
    VThermHvacMode,
    VThermHvacMode_COOL,
    VThermHvacMode_HEAT,
    VThermHvacMode_OFF,
)
from .cycle_manager import CycleManager
from homeassistant.core import HomeAssistant

_LOGGER = logging.getLogger(__name__)


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x into [lo, hi]."""
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


########################################################################
#                                                                      #
#                      CONSTANTS & ENUMS                               #
#                                                                      #
########################################################################

class SmartPIPhase(str, Enum):
    """Phases of the Smart-PI algorithm."""
    BOOTSTRAP = "Bootstrap" # Waiting for first data
    WARMUP = "Warmup"  # Accumulating initial history, FF scaled down
    TUNING = "Tuning"  # Active learning, but model not reliable yet
    STABLE = "Stable"  # Model reliable, optimized gains

# ------------------------------
# Default controller parameters
# ------------------------------

# Safe fallback gains when model is unreliable
KP_SAFE = 0.55
KI_SAFE = 0.010

# Allowed ranges for computed gains
KP_MIN = 0.10
KP_MAX = 5.0
KI_MIN = 0.001
KI_MAX = 0.050



# Anti-windup / integrator behavior
INTEGRAL_LEAK = 0.995  # leak factor per cycle when inside deadband
MAX_STEP_PER_MINUTE = 0.25  # max output change per minute (rate limit)

# Setpoint step boost: faster rate limit when setpoint changes significantly
# This allows quick power ramp-up when user increases setpoint
SETPOINT_BOOST_THRESHOLD = 0.3   # min setpoint change (°C) to trigger boost
SETPOINT_BOOST_ERROR_MIN = 0.3   # min error (°C) to keep boost active
SETPOINT_BOOST_RATE = 0.50       # boosted rate limit (/min) vs 0.15 normal

# Setpoint change handling (mode change vs adjustment)
# - Large change (>= threshold): mode change (eco ↔ comfort) -> reset PI state
# - Small change (< threshold): minor adjustment -> bumpless transfer with limited output jump
SETPOINT_MODE_DELTA_C = 0.5      # °C threshold for mode change detection
SETPOINT_BUMPLESS_MAX_DU = 0.12  # Max allowed output change (0..1) for bumpless transfer
OVERSHOOT_I_CLAMP_EPS_C = 0.10  # Guard band below setpoint where integral cannot increase (°C)

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

# --- Asymmetric Deadband / Near-band (HEAT only) ---
# Intent (thermal "rule of thumb"):
# - Make the "quiet zone" a bit wider when slightly below the setpoint (e>0) so the controller
#   does not wait too long before restarting after a setpoint decrease.
# - Make the zone tighter above the setpoint (e<0) to reduce overshoot/hunting.
# Guardrails: asymmetry is applied only in HEAT; COOL keeps symmetric logic.

# Deadband (°C) and its hysteresis (°C)
DEADBAND_BELOW_C = 0.06
DEADBAND_ABOVE_C = 0.04
DEADBAND_HYST_BELOW_C = 0.02
DEADBAND_HYST_ABOVE_C = 0.02

# Deadband+ (DB+): minimum holding power when slightly below setpoint inside deadband
DEADBAND_PLUS_MIN_U = 0.08   # 8% duty-cycle
DEADBAND_PLUS_MAX_U = 0.20   # hard cap (safety)

# Micro-leak on integral while in deadband (dt-aware). Value is per "cycle".
INTEGRAL_DEADBAND_MICROLEAK = 0.999

# Near-band asymmetry:
# - below setpoint: use configured near_band_deg (self.near_band_deg)
# - above setpoint: scale it down with a factor
NEAR_BAND_ABOVE_FACTOR = 0.60
NEAR_BAND_HYSTERESIS_C = 0.03

# Asymmetric setpoint EMA filter parameters
# Asymmetric setpoint EMA filter parameters (Time Constants in minutes)
# Alpha = 1 - exp(-dt / Tau)
# Old alphas: 0.05 (slow), 0.40 (fast) for ~10-15 min cycles
SP_TAU_SLOW = 200.0    # Minutes
SP_TAU_FAST = 20.0     # Minutes
SP_BAND = 1.0          # Band for alpha interpolation (°C)
SP_BYPASS_ERROR_THRESHOLD = 0.8  # Bypass filter when error > this (°C)

# Error filter time constant
ERROR_FILTER_TAU = 25.0 # Minutes (matches alpha ~0.35 at 10min)


# --- Robust learning / gating constants ---
# Window sizes
B_POINTS_MAX = 40        # OFF samples for b (tau)
A_POINTS_MAX = 25        # ON samples for a
RESIDUAL_HIST_MAX = 60   # Residual history for MAD estimation

# Robust gating
RESIDUAL_GATE_K = 4.5   # |r| > k * sigma_r  -> freeze learning

# Intercept coherence checks (dimensionless ratios)
INTERCEPT_SIGMA_FACTOR = 2.0   # |c| <= factor * sigma_r
INTERCEPT_SCALE_FACTOR = 0.30  # |c| <= factor * median(|y|)

# Tau stability check
B_STABILITY_MAD_RATIO_MAX = 0.60   # MAD(b) / median(b)
LEARN_BOOTSTRAP_COUNT = 10      # Number of learn cycles before applying strict residual gating

# --- SmartPI Robust Learning Constants ---
# Median+MAD Strategy Constants
AB_HISTORY_SIZE = 31      # Keep last 31 (ODD) values
AB_MIN_SAMPLES = 11       # Start learning after 11 (ODD) values
AB_MAD_SIGMA_MULT = 3.0   # Outlier rejection threshold (sigma)

AB_MAD_K = 1.4826         # Sigma scaling factor for MAD
AB_VAL_TOLERANCE = 1e-12  # Small epsilon
LEARN_SAMPLE_MAX = 240          # Max samples history (e.g. 4h @ 1min)
LEARN_Q_HIST_MAX = 200          # History for quantization estimation
DT_MIN_OK = 0.5                 # Min dt (minutes) for valid derivative window
DT_MAX_OK = 30.0                # Max dt (minutes) for valid derivative window
DT_DERIVATIVE_MIN_ABS = 0.03    # Min absolute dT (°C) if quantization unknown
LEARN_QUALITY_THRESHOLD = 0.25  # Min QI quality to accept learning
QUANTIZATION_ROUND_TO = 0.001   # Rounding / binning for quantization detection

# --- SmartPI Learning Window Constants ---
# Learning requires at least one full cycle (measured by _cycle_min)
DT_MAX_MIN = 30
MIN_ABS_DT = 0.03      # °C
DELTA_MIN = 0.2        # °C (Matches DELTA_MIN_ON)
U_OFF_MAX = 0.05
U_ON_MIN = 0.20
DELTA_MIN_OFF = 0.5        # °C
DELTA_MIN_ON = 0.2         # °C

# --- SmartPI Near Band Defaults ---
DEFAULT_NEAR_BAND_DEG = 0.30
DEFAULT_KP_NEAR_FACTOR = 0.80
DEFAULT_KI_NEAR_FACTOR = 0.6

# --- Dead Time Learning Constants (Phase 1) ---
# Step detection thresholds
DT_SP_STEP_MIN_DEG = 1.0          # Minimum setpoint step to trigger learning (°C)
DT_SP_STABLE_TOL_DEG = 0.5        # Tolerance for setpoint stability (°C)

# Stability windows
DT_PRE_SP_STABLE_MIN_S = 1800     # 30 min - SP must be stable before step
DT_POST_SP_STABLE_MIN_S = 1200    # 20 min - SP must be stable after step

# Temperature rise detection
DT_SLOPE_WIN_S = 600              # 10 min - window for slope calculation
DT_SLOPE_MIN_DEG_PER_H = 0.06     # Minimum slope to detect heating response
DT_SLOPE_PERSIST_S = 600          # 10 min - slope must persist

# Dead time bounds
DT_DEADTIME_MIN_S = 120           # 2 min minimum
DT_DEADTIME_MAX_S = 7200          # 2 hours maximum

# Reliability
DT_MIN_SAMPLES_RELIABLE = 3       # Minimum samples for reliable estimate

# Freeze periods
DT_FREEZE_AFTER_SETPOINT_S = 1800 # 30 min - freeze NEW triggers after SP change
DT_FREEZE_AFTER_BOOT_S = 3600     # 60 min - freeze learning after boot

# History sizes
DT_SP_HISTORY_SIZE = 200          # ~1.5h @ 30s per sample
DT_TIN_HISTORY_SIZE = 200         # ~1.5h @ 30s per sample
DT_MAX_SAMPLES = 11               # Keep last 11 dead time samples


@dataclass(frozen=True)
class TauReliability:
    """Result of tau (time constant) reliability check."""
    reliable: bool
    tau_min: float  # minutes (min of candidates used)


@dataclass
class DeadTimeEvent:
    """A pending dead time measurement event."""
    ts: float          # Timestamp when step occurred (monotonic)
    sp_before: float   # Setpoint before step
    sp_after: float    # Setpoint after step


class DeadTimeEstimator:
    """
    Estimates thermal dead time (θ_heat) by observing the delay between
    setpoint steps and temperature response.

    Learning is triggered by significant positive setpoint changes in HEAT mode.
    The pending measurement continues even if a freeze is applied after the trigger.
    """

    def __init__(self, max_samples: int = DT_MAX_SAMPLES):
        self._max_samples = max_samples
        self._samples: Deque[float] = deque(maxlen=max_samples)
        self._deadtime_s: Optional[float] = None
        self._reliable: bool = False

        # Pending measurement state (survives freeze)
        self._pending_event: Optional[DeadTimeEvent] = None
        self._pending_tin_history: Deque[Tuple[float, float]] = deque(maxlen=DT_TIN_HISTORY_SIZE)

        # Setpoint history for stability checks
        self._sp_history: Deque[Tuple[float, float]] = deque(maxlen=DT_SP_HISTORY_SIZE)

        # Freeze state (prevents NEW triggers, not pending completion)
        self._freeze_until_ts: Optional[float] = None

    def reset(self) -> None:
        """Reset all learned dead time data."""
        self._samples.clear()
        self._deadtime_s = None
        self._reliable = False
        self._pending_event = None
        self._pending_tin_history.clear()
        self._sp_history.clear()
        self._freeze_until_ts = None

    @property
    def deadtime_s(self) -> Optional[float]:
        """Dead time estimate in seconds, or None if not yet learned."""
        return self._deadtime_s

    @property
    def reliable(self) -> bool:
        """True if dead time estimate is reliable (enough samples)."""
        return self._reliable

    @property
    def sample_count(self) -> int:
        """Number of valid dead time samples collected."""
        return len(self._samples)

    @property
    def pending(self) -> bool:
        """True if a measurement is in progress."""
        return self._pending_event is not None

    def freeze_learning(self, duration_s: float) -> None:
        """
        Freeze NEW dead time triggers for the specified duration.
        Does NOT cancel pending measurements already started.
        """
        now = time.monotonic()
        self._freeze_until_ts = now + duration_s

    def _is_frozen(self) -> bool:
        """Check if new triggers are frozen."""
        if self._freeze_until_ts is None:
            return False
        if time.monotonic() >= self._freeze_until_ts:
            self._freeze_until_ts = None
            return False
        return True

    def _check_sp_stable(self, ts_end: float, duration_s: float, tolerance: float) -> bool:
        """Check if setpoint was stable in [ts_end - duration_s, ts_end]."""
        ts_start = ts_end - duration_s
        points = [(ts, sp) for ts, sp in self._sp_history
                  if ts_start <= ts <= ts_end]
        if len(points) < 3:  # Need minimum samples
            return False
        sp_values = [sp for _, sp in points]
        return (max(sp_values) - min(sp_values)) <= tolerance

    def _detect_tin_rise(self, after_ts: float) -> Optional[float]:
        """
        Detect when Tin starts rising persistently after the given timestamp.
        Returns the timestamp of rise start, or None if not detected.
        """
        samples = [(ts, tin) for ts, tin in self._pending_tin_history if ts >= after_ts]
        if len(samples) < 10:
            return None

        min_slope = DT_SLOPE_MIN_DEG_PER_H / 3600.0  # Convert to °C/s

        for i in range(len(samples) - 1):
            t0, tin0 = samples[i]

            # Find samples in slope window [t0, t0 + SLOPE_WIN_S]
            win_samples = [(ts, tin) for ts, tin in samples
                           if t0 <= ts <= t0 + DT_SLOPE_WIN_S]
            if len(win_samples) < 2:
                continue

            t_last, tin_last = win_samples[-1]
            dt = t_last - t0
            if dt < 60:  # Need at least 1 min
                continue

            slope = (tin_last - tin0) / dt
            if slope < min_slope:
                continue

            # Check persistence in [t0, t0 + SLOPE_WIN_S + PERSIST_S]
            persist_samples = [(ts, tin) for ts, tin in samples
                               if t0 <= ts <= t0 + DT_SLOPE_WIN_S + DT_SLOPE_PERSIST_S]
            if len(persist_samples) < 3:
                continue

            t_persist, tin_persist = persist_samples[-1]
            total_dt = t_persist - t0
            if total_dt < DT_SLOPE_WIN_S:
                continue

            avg_slope = (tin_persist - tin0) / total_dt
            if avg_slope >= min_slope:
                return t0

        return None

    def record_sample(self, setpoint: float, tin: float) -> Optional[str]:
        """
        Record a setpoint and temperature sample for dead time learning.
        Should be called at each calculate() call.

        Returns a status string for diagnostics, or None.
        """
        now = time.monotonic()

        # Always record SP history
        self._sp_history.append((now, setpoint))

        # If pending measurement, record Tin and check for completion
        if self._pending_event is not None:
            self._pending_tin_history.append((now, tin))
            event = self._pending_event

            # Timeout check: abort if no result after 2x max deadtime
            if now > event.ts + DT_DEADTIME_MAX_S * 2:
                self._pending_event = None
                self._pending_tin_history.clear()
                return "timeout"

            # Check post-step SP stability
            elapsed = now - event.ts
            if elapsed >= DT_POST_SP_STABLE_MIN_S:
                post_stable = self._check_sp_stable(
                    now, DT_POST_SP_STABLE_MIN_S, DT_SP_STABLE_TOL_DEG
                )
                if not post_stable:
                    self._pending_event = None
                    self._pending_tin_history.clear()
                    return "abort:sp_unstable"

            # Detect temperature rise
            rise_ts = self._detect_tin_rise(event.ts)
            if rise_ts is not None:
                dt_sample = rise_ts - event.ts

                if DT_DEADTIME_MIN_S <= dt_sample <= DT_DEADTIME_MAX_S:
                    self._samples.append(dt_sample)
                    self._update_estimate()
                    self._pending_event = None
                    self._pending_tin_history.clear()
                    return f"learned:{dt_sample:.0f}s"
                else:
                    self._pending_event = None
                    self._pending_tin_history.clear()
                    return f"reject:out_of_bounds({dt_sample:.0f}s)"

        return None

    def on_setpoint_change(self, sp_old: float, sp_new: float, hvac_mode: str) -> str:
        """
        Handle setpoint change event.
        Triggers dead time measurement if conditions are met.

        Returns status string for diagnostics.
        """
        # Check if new triggers are frozen
        if self._is_frozen():
            return "skip:frozen"

        # Only learn in HEAT mode on positive steps
        if hvac_mode == VThermHvacMode_COOL:
            return "skip:cool_mode"

        step = sp_new - sp_old
        if step < DT_SP_STEP_MIN_DEG:
            return f"skip:step_too_small({step:.2f})"

        now = time.monotonic()

        # Check pre-step stability (SP was stable BEFORE this change)
        pre_stable = self._check_sp_stable(
            now, DT_PRE_SP_STABLE_MIN_S, DT_SP_STABLE_TOL_DEG
        )
        if not pre_stable:
            return "skip:sp_unstable_before"

        # Cancel any previous pending (shouldn't happen often)
        if self._pending_event is not None:
            self._pending_event = None
            self._pending_tin_history.clear()

        # Start new pending measurement
        self._pending_event = DeadTimeEvent(
            ts=now,
            sp_before=sp_old,
            sp_after=sp_new
        )
        self._pending_tin_history.clear()

        return f"trigger:step={step:.2f}"

    def _update_estimate(self) -> None:
        """Update the dead time estimate from collected samples."""
        if len(self._samples) == 0:
            self._deadtime_s = None
            self._reliable = False
            return

        self._deadtime_s = statistics.median(self._samples)
        self._reliable = len(self._samples) >= DT_MIN_SAMPLES_RELIABLE

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence."""
        freeze_wall_ts = None
        if self._freeze_until_ts is not None:
            remaining = self._freeze_until_ts - time.monotonic()
            if remaining > 0:
                freeze_wall_ts = time.time() + remaining

        return {
            "deadtime_samples": list(self._samples),
            "deadtime_s": self._deadtime_s,
            "deadtime_reliable": self._reliable,
            "deadtime_freeze_ts": freeze_wall_ts,
        }

    def load_state(self, state: Dict[str, Any]) -> None:
        """Load state from persistence."""
        if not state:
            return

        samples = state.get("deadtime_samples", [])
        self._samples = deque(samples, maxlen=self._max_samples)
        self._deadtime_s = state.get("deadtime_s")
        self._reliable = state.get("deadtime_reliable", False)

        freeze_wall_ts = state.get("deadtime_freeze_ts")
        if freeze_wall_ts is not None:
            remaining = freeze_wall_ts - time.time()
            if remaining > 0:
                self._freeze_until_ts = time.monotonic() + remaining

        # Clear pending state (don't restore across restarts)
        self._pending_event = None
        self._pending_tin_history.clear()
        self._sp_history.clear()


########################################################################
#                                                                      #
#                      ESTIMATOR CLASS                                 #
#                                                                      #
########################################################################

class ABEstimator:
    """
    Robust Online Estimator for a and b using Hybrid approach:
    1. Early stage (unreliable tau): Median/MAD + Huber location on raw samples.
    2. Stable stage (reliable tau): Theil-Sen regression on (x,y) points.

    Model: dT/dt = a*u - b*(T_int - T_ext)
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

        # Stability tracking for b (tau) - used for reliability check
        self._b_hat_hist: Deque[float] = deque(maxlen=20)

        # Counters
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
        self._b_hat_hist.clear()
        self.a_meas_hist.clear()
        self.b_meas_hist.clear()

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
                sigma_b = AB_MAD_K * mad_b
                if abs(b_meas - med_b) > AB_MAD_SIGMA_MULT * sigma_b:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: b_meas outlier"
                    return

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
                sigma_a = AB_MAD_K * mad_a
                if abs(a_meas - med_a) > AB_MAD_SIGMA_MULT * sigma_a:
                    self.learn_skip_count += 1
                    self.learn_last_reason = "skip: a_meas outlier"
                    return

            new_a = med_a  # Use median directly
            new_a = clamp(new_a, self.A_MIN, self.A_MAX)

            self.a = new_a
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
        # 10 min to ~66 hours (6000 min)
        reliable = 10.0 <= tau <= 6000.0
        return TauReliability(reliable=reliable, tau_min=tau)


########################################################################
#                                                                      #
#                      MAIN ALGO CLASS                                 #
#                                                                      #
########################################################################

class SmartPI(CycleManager):
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
        hass: HomeAssistant,
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
        near_band_deg: float = DEFAULT_NEAR_BAND_DEG,
        kp_near_factor: float = DEFAULT_KP_NEAR_FACTOR,
        ki_near_factor: float = DEFAULT_KI_NEAR_FACTOR,
        sign_flip_leak: float = 0.40,
        sign_flip_leak_cycles: int = 3,
        sign_flip_band_mult: float = 2.0,
        use_setpoint_filter: bool = True,
    ) -> None:
        super().__init__(hass, name, cycle_min, minimal_deactivation_delay)

        self._name = name
        # self._cycle_min is managed by CycleManager
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
        # self._ema_alpha: float = 0.35  # Deprecated: using ERROR_FILTER_TAU

        # Current gains
        self.Kp: float = KP_SAFE
        self.Ki: float = KI_SAFE
        self._kp: float = KP_SAFE
        self._ki: float = KI_SAFE

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
        self._sign_flip_active: bool = False
        self._sign_flip_end_ts: Optional[float] = None

        # Asymmetric setpoint EMA filter state
        self._filtered_setpoint: Optional[float] = None
        self._last_raw_setpoint: Optional[float] = None
        # Initial temperature when setpoint changed (for midpoint filter activation)
        self._initial_temp_for_filter: Optional[float] = None

        # Track last time calculate() was executed for dt-based integration
        self._last_calculate_time: Optional[float] = None
        # Accumulated time for cycle counting (used for FF warm-up)
        self._accumulated_dt: float = 0.0

        # Timestamp for robust learning dt calculation
        self._learn_last_ts: float | None = None

        # Learning window state (multi-cycle learning)
        self.learn_win_active: bool = False
        self.learn_win_start_ts: float | None = None
        self.learn_T_int_start: float = 0.0
        self.learn_T_ext_start: float = 0.0
        self.learn_u_int: float = 0.0

        self.learn_t_int_s: float = 0.0
        # For strict power check during window extension
        self.learn_u_first: float | None = None

        # Learning start timestamp
        self._learning_start_date: Optional[datetime] = datetime.now()

        # Skip learning cycles after resume from interruption (window close, etc.)
        self._learning_resume_ts: Optional[float] = None

        # Deadband state tracking for bumpless transfer on exit
        self._in_deadband: bool = False
        # Near-band hysteresis state (for stable gain scheduling)
        self._in_near_band: bool = False

        # Tracking anti-windup diagnostics
        self._last_u_cmd: float = 0.0       # command after [0,1] clamp
        self._last_u_limited: float = 0.0   # after rate-limit and max_on_percent
        self._last_u_applied: float = 0.0   # after timing constraints
        self._last_aw_du: float = 0.0       # tracking delta for diagnostics
        self._last_forced_by_timing: bool = False  # True when timing forced 0%/100%

        # Setpoint step boost state (for fast power ramp-up on setpoint change)
        self._setpoint_boost_active: bool = False
        self._setpoint_changed_in_cycle: bool = False  # Track if setpoint changed during cycle
        self._prev_setpoint_for_boost: Optional[float] = None

        # Enhanced A/B Learning: Start-of-cycle snapshot
        # self._cycle_start_state is now managed by CycleManager via _current_cycle_params

        # Thermal Guard for setpoint decrease
        # Prevents integral accumulation while T > T_set after a decrease
        self._hysteresis_thermal_guard: bool = False

        # Dead time estimator (Phase 1: learning only)
        self._deadtime_est = DeadTimeEstimator()
        self._deadtime_last_status: Optional[str] = None

        if saved_state:
            self.load_state(saved_state)

        _LOGGER.debug("%s - SmartPI initialized", self._name)

    # ------------------------------
    # Persistence
    # ------------------------------

    ########################################################################
    #                                                                      #
    #                      PERSISTENCE & STATE                             #
    #                                                                      #
    ########################################################################

    def reset_learning(self) -> None:
        """Reset all learned parameters to defaults."""
        self.est.reset()
        self.integral = 0.0
        self.u_prev = 0.0
        self._on_percent = 0.0
        self._on_time_sec = 0
        self._off_time_sec = 0
        self._last_u_ff = 0.0
        self._last_u_pi = 0.0
        self._last_u_cmd = 0.0
        self._last_u_limited = 0.0
        self._last_u_applied = 0.0
        self._last_aw_du = 0.0
        self._e_filt = None
        self.Kp = KP_SAFE
        self.Ki = KI_SAFE
        self._cycles_since_reset = 0
        self._accumulated_dt = 0.0
        self._prev_error = None
        self._sign_flip_active = False
        self._sign_flip_end_ts = None
        self._filtered_setpoint = None
        self._last_raw_setpoint = None
        self._initial_temp_for_filter = None
        self._last_calculate_time = None
        self._learn_last_ts = None
        self._learning_start_date = datetime.now()
        self._learning_resume_ts = None
        self._in_deadband = False
        self._in_near_band = False
        self._setpoint_boost_active = False
        self._setpoint_changed_in_cycle = False
        self._prev_setpoint_for_boost = None
        self.learn_win_active = False
        self.learn_win_start_ts = None
        self.learn_T_int_start = 0.0
        self.learn_T_ext_start = 0.0
        self.learn_u_int = 0.0
        self.learn_t_int_s = 0.0
        self.learn_t_int_s = 0.0
        self.learn_u_first = None
        self._hysteresis_thermal_guard = False
        self._deadtime_est.reset()
        self._deadtime_last_status = None
        _LOGGER.info("%s - SmartPI learning and history reset", self._name)

    def notify_resume_after_interruption(self, skip_cycles: int = None) -> None:
        """Notify SmartPI that the thermostat is resuming after an interruption.

        This is called when the thermostat resumes after a window close event
        or similar interruption. It sets a deadline before which learning is ignored.

        Args:
            skip_cycles: Legacy argument (count of cycles). Converted to duration approx.
        """
        if skip_cycles is None:
            skip_cycles = SKIP_CYCLES_AFTER_RESUME

        # Robust conversion: assume at least 15 min per cycle equivalent if cycle_min is small,
        # or use cycle_min. This is a heuristic.
        duration_min = float(skip_cycles) * max(self._cycle_min, 15.0)
        self._learning_resume_ts = time.monotonic() + (duration_min * 60.0)

        # Also reset the learning timestamp to avoid using stale dt
        self._learn_last_ts = None

        # Compute wall-clock time for logging/diagnostics
        try:
            # Just for logging
            resume_dt_log = datetime.now().timestamp() + (duration_min * 60.0)
            resume_dt_iso = datetime.fromtimestamp(resume_dt_log).isoformat()
        except Exception:
            resume_dt_iso = "unknown"

        _LOGGER.info("%s - SmartPI notified of resume after interruption, skipping learning until (approx) %s", self._name, resume_dt_iso)

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
            b_hat_hist_data = state.get("b_hat_hist", [])

            # Simple float queues
            self.est._b_hat_hist = deque(b_hat_hist_data, maxlen=self.est._b_hat_hist.maxlen)

            # Hybrid learning raw histories
            a_meas_data = state.get("a_meas_hist", [])
            b_meas_data = state.get("b_meas_hist", [])
            self.est.a_meas_hist = deque(a_meas_data, maxlen=self.est.a_meas_hist.maxlen)
            self.est.b_meas_hist = deque(b_meas_data, maxlen=self.est.b_meas_hist.maxlen)

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

            # Load learning window state
            # FORCE RESET ON LOAD: We do not restore the active learning window.
            # A reboot/reload implies an interruption of unknown duration.
            # We must discard the partial cycle to avoid corrupting learning with invalid time deltas.
            self.learn_win_active = False
            self.learn_win_start_ts = None
            self.learn_T_int_start = 0.0
            self.learn_T_ext_start = 0.0
            self.learn_u_int = 0.0
            self.learn_t_int_s = 0.0

            # (Legacy code removed: previously we tried to restore these fields)

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
            # Convert legacy cycle count to absolute ts if needed
            resume_ts = state.get("learning_resume_ts")
            if resume_ts is not None:
                try:
                    # Stored as wall clock timestamp. Convert to monotonic.
                    stored_wall_ts = float(resume_ts)
                    now_wall = time.time()
                    remaining = stored_wall_ts - now_wall
                    if remaining > 0:
                        self._learning_resume_ts = time.monotonic() + remaining
                    else:
                        self._learning_resume_ts = None
                except (ValueError, TypeError):
                    self._learning_resume_ts = None
            else:
                # Check legacy key
                legacy_skip = int(state.get("skip_learning_cycles_left", 0) or 0)
                if legacy_skip > 0:
                    # Convert to approx duration
                    duration_min = float(legacy_skip) * max(self._cycle_min, 15.0)
                    self._learning_resume_ts = time.monotonic() + (duration_min * 60.0)
                else:
                    self._learning_resume_ts = None

            # Load deadband state
            self._in_deadband = bool(state.get("in_deadband", False))
            self._in_near_band = bool(state.get("in_near_band", False))


            # Load setpoint boost state
            self._setpoint_boost_active = bool(state.get("setpoint_boost_active", False))
            prev_sp = state.get("prev_setpoint_for_boost")
            if prev_sp is not None:
                try:
                    self._prev_setpoint_for_boost = float(prev_sp)
                except (ValueError, TypeError):
                    self._prev_setpoint_for_boost = None

            # Load thermal guard state
            self._hysteresis_thermal_guard = bool(state.get("hysteresis_thermal_guard", False))

            # Load dead time state
            deadtime_state = state.get("deadtime_state", {})
            self._deadtime_est.load_state(deadtime_state)
            # Apply boot freeze for new triggers
            self._deadtime_est.freeze_learning(DT_FREEZE_AFTER_BOOT_S)

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
        # Convert monotonic learning_resume_ts back to wall clock for persistence
        resume_wall_ts = None
        if self._learning_resume_ts is not None:
            remaining = self._learning_resume_ts - time.monotonic()
            if remaining > 0:
                resume_wall_ts = time.time() + remaining

        return {
            "a": self.est.a,
            "b": self.est.b,
            "learn_ok_count": self.est.learn_ok_count,
            "learn_ok_count_a": self.est.learn_ok_count_a,
            "learn_ok_count_b": self.est.learn_ok_count_b,
            "learn_skip_count": self.est.learn_skip_count,
            "b_hat_hist": list(self.est._b_hat_hist),
            "a_meas_hist": list(self.est.a_meas_hist),
            "b_meas_hist": list(self.est.b_meas_hist),
            "integral": self.integral,
            "u_prev": self.u_prev,
            "cycles_since_reset": self._cycles_since_reset,
            "accumulated_dt": self._accumulated_dt,
            "learning_start_date": self._learning_start_date.isoformat() if self._learning_start_date else None,
            "learn_win_active": self.learn_win_active,
            "learn_win_start_ts": self.learn_win_start_ts,
            "learn_T_int_start": self.learn_T_int_start,
            "learn_T_ext_start": self.learn_T_ext_start,
            "learn_u_int": self.learn_u_int,
            "learn_t_int_s": self.learn_t_int_s,
            "filtered_setpoint": self._filtered_setpoint,
            "last_raw_setpoint": self._last_raw_setpoint,
            "initial_temp_for_filter": self._initial_temp_for_filter,
            "learning_resume_ts": resume_wall_ts,
            "in_deadband": self._in_deadband,
            "in_near_band": self._in_near_band,
            "setpoint_boost_active": self._setpoint_boost_active,
            "prev_setpoint_for_boost": self._prev_setpoint_for_boost,
            "hysteresis_thermal_guard": self._hysteresis_thermal_guard,
            "deadtime_state": self._deadtime_est.save_state(),
            "phase": self.phase,
        }

    @property
    def phase(self) -> str:
        """Current phase of the algorithm."""
        if self.cycles_since_reset == 0:
            return SmartPIPhase.BOOTSTRAP

        # Warmup condition:
        # Either strictly based on time (cycles) OR learning count confidence
        # We stay in warmup until we have BOTH enough cycles AND enough learning samples
        # effectively scaling up FF progressively.
        if (
            self.cycles_since_reset < self.ff_warmup_cycles
            or self.est.learn_ok_count_a < 10
            or self.est.learn_ok_count_b < 10
        ):
            return SmartPIPhase.WARMUP

        if self.tau_reliable:
            return SmartPIPhase.STABLE

        return SmartPIPhase.TUNING

    @property
    def meas_count_a(self) -> int:
        """Return number of collected 'a' measurements in the buffer."""
        return len(self.est.a_meas_hist)

    @property
    def meas_count_b(self) -> int:
        """Return number of collected 'b' measurements in the buffer."""
        return len(self.est.b_meas_hist)

    # ------------------------------
    # Learning entry point
    # ------------------------------

    def _reset_learning_window(self) -> None:
        """Reset the multi-cycle learning window state."""
        self.learn_win_active = False
        self.learn_win_start_ts = None
        self.learn_u_int = 0.0
        self.learn_t_int_s = 0.0
        self.learn_u_first = None

    async def on_cycle_started(self, on_time_sec: float, off_time_sec: float, on_percent: float, hvac_mode: str) -> None:
        """Called when a cycle starts."""
        await super().on_cycle_started(on_time_sec, off_time_sec, on_percent, hvac_mode)
        self._setpoint_changed_in_cycle = False
        # Update internal state to match applied values (legacy support for diagnostics)
        self._on_time_sec = int(on_time_sec)
        self._off_time_sec = int(off_time_sec)
        self._on_percent = on_percent

    async def on_cycle_completed(self, new_params: dict, prev_params: dict | None) -> bool:
        """Handle end of cycle (learning). Return False to extend window."""
        await super().on_cycle_completed(new_params, prev_params)

        if prev_params is None:
             # First cycle or check-in, nothing to learn yet
             return True

        # 1. Retrieve Context
        t_int_start = prev_params.get("temp_in")
        t_ext_start = prev_params.get("temp_ext")
        u_applied = prev_params.get("on_percent") 
        ts_start = prev_params.get("timestamp")
        
        current_temp = new_params.get("temp_in")
        ext_current_temp = new_params.get("temp_ext")
        timestamp = new_params.get("timestamp")
        
        # Calculate dt using timestamps from CycleManager params
        if timestamp is None or ts_start is None:
             # Should not happen if data provider is correct
             return True
             
        dt_s = (timestamp - ts_start).total_seconds()
        dt_min = dt_s / 60.0 # Used for learning checks
        now = time.monotonic() # For valid _learning_resume_ts check

        # Capture flags relevant to THIS cycle
        setpoint_changed_in_this_cycle = self._setpoint_changed_in_cycle

        # 2. Interruption / Resume Check
        if self._learning_resume_ts:
            if now < self._learning_resume_ts:
                self.est.learn_skip_count += 1
                self.est.learn_last_reason = "skip: resume cool-down"
                self._reset_learning_window()
                return True # Cut cycle (don't extend bad conditions)
            else:
                self._learning_resume_ts = None

        # 3. Validation
        if u_applied is None:
            self.est.learn_skip_count += 1
            self.est.learn_last_reason = "skip: cycle power undefined"
            return True

        if ext_current_temp is None:
            self.est.learn_skip_count += 1
            self.est.learn_last_reason = "skip: no external temp"
            self._reset_learning_window()
            return True

        if setpoint_changed_in_this_cycle:
            self.est.learn_skip_count += 1
            self.est.learn_last_reason = "skip: setpoint change"
            self._reset_learning_window()
            return True

        # 4. Multi-cycle learning window logic
        if not self.learn_win_active:
            self.learn_win_active = True
            # Use data from the START of the agglomerated window
            self.learn_win_start_ts = time.monotonic() - dt_s # Approx start time in monotonic
            self.learn_T_int_start = t_int_start
            self.learn_T_ext_start = t_ext_start
            self.learn_u_int = 0.0
            self.learn_t_int_s = 0.0
            self.learn_u_first = u_applied
            self.est.learn_last_reason = "learn: window start"
        else:
            # Window extension: check power stability
            if self.learn_u_first is not None and abs(u_applied - self.learn_u_first) > 1e-3:
                self.est.learn_skip_count += 1
                self.est.learn_last_reason = "skip: power instability"
                self._reset_learning_window()
                return True

        # Integrate u_applied over the cycle duration
        self.learn_u_int += clamp(u_applied, 0.0, 1.0) * dt_s
        self.learn_t_int_s += dt_s
        
        # Calculate actual dt for the window
        # Note: We rely on learn_t_int_s as a proxy for window duration if monotonic time is tricky?
        # But we need wall time for dT calculation? No, just dt.
        # Let's use accumulated seconds / 60
        window_dt_min = self.learn_t_int_s / 60.0

        dT = current_temp - self.learn_T_int_start
        abs_dT = abs(dT)
        delta_T = self.learn_T_int_start - self.learn_T_ext_start
        
        # 5. Extension Checks
        if abs(delta_T) < DELTA_MIN:
            self._reset_learning_window()
            self.est.learn_last_reason = "skip: delta too small"
            return True

        if abs_dT < MIN_ABS_DT and window_dt_min < DT_MAX_MIN:
            self.est.learn_last_reason = "skip: extending window (dT too small)"
            return False # <--- EXTEND WINDOW

        if abs_dT < MIN_ABS_DT and window_dt_min >= DT_MAX_MIN:
            self._reset_learning_window()
            self.est.learn_last_reason = "skip: window timeout (dT too small)"
            return True

        if self.learn_t_int_s <= 0.0:
            self._reset_learning_window()
            self.est.learn_last_reason = "skip: window duty invalid"
            return True

        # 6. Learning Submission
        u_eff = self.learn_u_int / self.learn_t_int_s
        dT_dt = dT / window_dt_min

        if u_eff < U_OFF_MAX:
            self.est.learn(
                dT_int_per_min=dT_dt,
                u=0.0,
                t_int=self.learn_T_int_start,
                t_ext=self.learn_T_ext_start,
            )
        elif u_eff > U_ON_MIN:
            self.est.learn(
                dT_int_per_min=dT_dt,
                u=u_eff,
                t_int=self.learn_T_int_start,
                t_ext=self.learn_T_ext_start,
            )
        else:
             self.est.learn_last_reason = "skip: low excitation (u mid)"

        self._reset_learning_window()
        
        # Cycle accepted -> Count it
        # Logic from calculate() moved here:
        self._cycles_since_reset += 1
        return True



    ########################################################################
    #                                                                      #
    #                      API & PROPERTIES                                #
    #                                                                      #
    ########################################################################

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
    def kp(self) -> float:
        return self._kp

    @property
    def ki(self) -> float:
        return self._ki

    @property
    def kp_reel(self) -> float:
        """Return the actual Kp used for calculation (after near-band adjustment)."""
        return self.Kp

    @property
    def ki_reel(self) -> float:
        """Return the actual Ki used for calculation (after near-band adjustment)."""
        return self.Ki

    @property
    def u_ff(self) -> float:
        """Last feed-forward value."""
        return self._last_u_ff

    @property
    def u_pi(self) -> float:
        return self._last_u_pi

    @property
    def tau_min(self) -> float:
        return self.est.tau_reliability().tau_min

    @property
    def tau_reliable(self) -> bool:
        return self._tau_reliable

    @property
    def learn_ok_count(self) -> int:
        return self.est.learn_ok_count

    @property
    def learn_ok_count_a(self) -> int:
        return self.est.learn_ok_count_a

    @property
    def learn_ok_count_b(self) -> int:
        return self.est.learn_ok_count_b

    @property
    def learn_skip_count(self) -> int:
        return self.est.learn_skip_count

    @property
    def learn_last_reason(self) -> str:
        return self.est.learn_last_reason

    @property
    def learning_start_dt(self) -> str:
        return self._learning_start_date.isoformat() if self._learning_start_date else None

    @property
    def i_mode(self) -> str:
        return self._last_i_mode

    @property
    def sat(self) -> str:
        return self._last_sat

    @property
    def error(self) -> float:
        return self._last_error

    @property
    def error_p(self) -> float:
        return self._last_error_p

    @property
    def error_filtered(self) -> float:
        return self._e_filt if self._e_filt is not None else 0.0

    @property
    def sign_flip_active(self) -> bool:
        return self._sign_flip_active

    @property
    def cycles_since_reset(self) -> int:
        return self._cycles_since_reset

    @property
    def filtered_setpoint(self) -> float:
        return self._filtered_setpoint

    @property
    def learning_resume_ts(self) -> float:
        # Return wall time estimation for diagnostics
        if self._learning_resume_ts is None:
            return None
        remaining = self._learning_resume_ts - time.monotonic()
        if remaining > 0:
            return time.time() + remaining
        return None

    @property
    def u_cmd(self) -> float:
        return self._last_u_cmd

    @property
    def u_limited(self) -> float:
        return self._last_u_limited

    @property
    def u_applied(self) -> float:
        return self._last_u_applied

    @property
    def aw_du(self) -> float:
        return self._last_aw_du

    @property
    def forced_by_timing(self) -> bool:
        return self._last_forced_by_timing

    @property
    def in_deadband(self) -> bool:
        return self._in_deadband

    @property
    def in_near_band(self) -> bool:
        return self._in_near_band

    @property
    def setpoint_boost_active(self) -> bool:
        return self._setpoint_boost_active

    @property
    def cycle_start_dt(self) -> str | None:
        """Return the start time of the current cycle."""
        if self._cycle_start_date:
            return self._cycle_start_date.isoformat()
        return None

    @property
    def deadtime_s(self) -> Optional[float]:
        """Dead time estimate in seconds."""
        return self._deadtime_est.deadtime_s

    @property
    def deadtime_reliable(self) -> bool:
        """True if dead time estimate is reliable."""
        return self._deadtime_est.reliable

    # ------------------------------
    # Asymmetric setpoint filter
    # ------------------------------

    ########################################################################
    #                                                                      #
    #                      SETPOINT FILTERING                              #
    #                                                                      #
    ########################################################################

    def _filter_setpoint(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode: VThermHvacMode,
        dt_min: float,
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
            dt_min: Elapsed time since last update (minutes)
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
            # Calculate dynamic alpha based on dt and Tau
            # alpha = 1 - exp(-dt / tau)
            # Interpolate Tau based on gap? No, interpolate the alpha-equivalents.

            # Use band to interpolate between fast and slow time constants
            w = min(gap / SP_BAND, 1.0)

            # Linear interpolation of Tau (simpler/safer than interpolating alphas directly)
            # Far from target (large gap) -> w=1 -> FAST Tau (small)
            # Close to target (small gap) -> w=0 -> SLOW Tau (large)
            tau = SP_TAU_SLOW + (SP_TAU_FAST - SP_TAU_SLOW) * w

            # Robust alpha calculation
            alpha = 1.0 - math.exp(-max(dt_min, 0.0) / max(tau, 1.0)) if tau > 0 else 1.0

            # Fallback for very small dt (Taylor expansion approx: alpha ~= dt/tau)
            if tau > 0:
                alpha = 1.0 - math.exp(-max(dt_min, 0.001) / tau)
            else:
                alpha = 1.0

            self._filtered_setpoint = alpha * target_temp + (1 - alpha) * self._filtered_setpoint

        return self._filtered_setpoint


    def _bump_integral_for_setpoint_change(
        self,
        t_set_old: float,
        t_set_new: float,
        t_in: float,
    ) -> None:
        """Adjust integral to minimize output jump on small setpoint changes (bumpless transfer).
        
        Theory: To keep u_pi ≈ Kp*e + Ki*I constant when setpoint changes:
            ΔI = (Kp/Ki) * (e_old - e_new)
        
        IMPORTANT: For thermal systems with low Ki, this ΔI can be huge.
        We limit the impact by capping the output variation: |Ki * ΔI| <= SETPOINT_BUMPLESS_MAX_DU
        
        Only use for small setpoint changes (< SETPOINT_MODE_DELTA_C).
        
        Args:
            t_set_old: Previous setpoint (°C)
            t_set_new: New setpoint (°C)
            t_in: Current indoor temperature (°C)
        """
        if self.Ki <= KI_MIN:
            return
        
        e_old = t_set_old - t_in
        e_new = t_set_new - t_in
        
        # Theoretical ΔI to keep u_pi constant
        dI = (self.Kp / self.Ki) * (e_old - e_new)
        
        # Overshoot I-clamp: when we are close to or above the (new) setpoint,
        # do not allow bumpless logic to increase the integral (would push heating in the wrong direction).
        if t_in >= (t_set_new - OVERSHOOT_I_CLAMP_EPS_C) and dI > 0.0:
            dI = 0.0
        
        # Limit bumpless: bound the output variation due to integral: Δu_I = Ki * ΔI
        # => |ΔI| <= SETPOINT_BUMPLESS_MAX_DU / Ki
        dI_max = SETPOINT_BUMPLESS_MAX_DU / max(self.Ki, KI_MIN)
        dI = clamp(dI, -dI_max, dI_max)
        
        old_integral = self.integral
        self.integral = self.integral + dI
        
        # Clamp to same limits as in calculate() (dynamic integral limit)
        i_max = 2.0 / max(self.Ki, KI_MIN)
        self.integral = clamp(self.integral, -i_max, i_max)
        
        _LOGGER.debug(
            "%s - Bumpless setpoint change (Δ=%.3f°C): integral %.4f → %.4f (ΔI=%.4f, capped=%.4f)",
            self._name, t_set_new - t_set_old, old_integral, self.integral, 
            (self.Kp / self.Ki) * (e_old -e_new), dI
        )

    # ------------------------------
    # Main control law
    # ------------------------------

    ########################################################################
    #                                                                      #
    #                      CONTROL LAW                                     #
    #                                                                      #
    ########################################################################

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None,
        hvac_mode: VThermHvacMode,
        integrator_hold: bool = False,
        power_shedding: bool = False,
    ) -> None:
        """
        Compute the next duty-cycle command.

        Notes
        -----
        - slope parameter is accepted for API compatibility (unused for now).
        - integrator_hold can be used during dead time / actuator constraints to avoid pumping.
        - power_shedding: if True, forces the output to 0 and resets internal state (e.g. overloading)

        Side effects:
        - Updates internal duty-cycle (_on_percent) and ON/OFF timings
        - Updates diagnostics and PI state

        Dynamic dt integration: This method can be called on every temperature sensor
        update. The integral term is accumulated based on actual elapsed time (dt_min),
        making it robust to irregular call intervals. A minimum dt threshold prevents
        excessive noise from very rapid calls.
        """
        ########################################################################
        #                                                                      #
        #                      STEP 1 - Input Validation               #
        #                      -------------------------               #
        #  Validation and dt calculation                                       #
        #                                                                      #
        ########################################################################

        now = time.monotonic()

        # Input validation
        if target_temp is None or current_temp is None:
            self._on_percent = 0.0
            self.u_prev = 0.0
            self._calculate_times()
            return

        if hvac_mode == VThermHvacMode_OFF or power_shedding:
            self._on_percent = 0.0
            self.u_prev = 0.0
            # Reset rate-limiting so next HEAT/COOL activation gets immediate calculation
            self._last_calculate_time = None
            
            # Reset PI integral to prevent windup during OFF/Overpowering
            if self.integral != 0.0:
                _LOGGER.debug("%s - Resetting integral to 0.0 (HVAC OFF or Power Shedding)", self._name)
                self.integral = 0.0
            
            self._calculate_times()
            return

        # Calculate elapsed time since last calculation for dt-based integration
        is_first_run = False
        if self._last_calculate_time is not None:
            dt_min = (now - self._last_calculate_time) / 60.0
        else:
            # First call: We use 0.0 to prevent integral windup (kickstart)
            # The first calculation will be P-only (proportional term) which is safer.
            dt_min = 0.0
            is_first_run = True

        # Minimum dt threshold to prevent noise from very rapid calls (< 3 seconds)
        # BUT we must bypass this if the setpoint has changed to ensure immediate reaction
        # We also bypass if it's the first run (dt=0) to ensure immediate output generation.
        MIN_DT_SECONDS = 3.0

        setpoint_changed = self._last_raw_setpoint is not None and abs(target_temp - self._last_raw_setpoint) > 0.01
        sp_delta = abs(target_temp - self._last_raw_setpoint) if self._last_raw_setpoint is not None else 0.0

        if setpoint_changed:
            self._setpoint_changed_in_cycle = True

            # Dead time: notify of setpoint change BEFORE freeze
            if hvac_mode != VThermHvacMode_OFF:
                dt_status = self._deadtime_est.on_setpoint_change(
                    self._last_raw_setpoint, target_temp, hvac_mode
                )
                _LOGGER.info("%s - DeadTime on_setpoint_change: %s", self._name, dt_status)
                # Freeze NEW triggers (pending measurement continues)
                self._deadtime_est.freeze_learning(DT_FREEZE_AFTER_SETPOINT_S)

            # Two-tier setpoint change handling (from regul6.py)
            if sp_delta >= SETPOINT_MODE_DELTA_C:
                # Large change: mode change (eco ↔ comfort) -> reset PI state
                old_integral = self.integral
                self.integral = 0.0

                # Force reset error state to avoid "wrong" error history (e.g. sign flip)
                # Calculate new error immediately
                new_e = float(target_temp - current_temp)
                if hvac_mode == VThermHvacMode_COOL:
                     new_e = -new_e

                self._e_filt = new_e
                self._last_error = new_e
                self._prev_error = new_e

                # Note: No need to reset u_prev (rate limiter still applies)
                # The setpoint_changed flag will bypass rate limit in the current cycle
                _LOGGER.info(
                    "%s - Mode change detected (Δ=%.2f°C >= %.2f°C): PI state reset (integral %.4f → 0.0, errors -> %.2f)",
                    self._name, sp_delta, SETPOINT_MODE_DELTA_C, old_integral, new_e
                )
                
                # Check for decrease to activate thermal guard (even on large change)
                if (target_temp < self._last_raw_setpoint) and hvac_mode != VThermHvacMode_COOL:
                    self._hysteresis_thermal_guard = True
                    _LOGGER.info("%s - Thermal guard activated (Large Decrease)", self._name)
                elif (target_temp > self._last_raw_setpoint) and hvac_mode != VThermHvacMode_COOL:
                    self._hysteresis_thermal_guard = False

            else:
                # Small change logic
                is_decrease = (target_temp < self._last_raw_setpoint)
                
                if is_decrease and hvac_mode != VThermHvacMode_COOL:
                     # Small decrease (Heating):
                     # 1. Activate Thermal Guard
                     self._hysteresis_thermal_guard = True
                     # 2. Skip Bumpless Transfer (to prevent artificial increase)
                     _LOGGER.info("%s - Small decrease detected (Δ=%.2f°C): Guard ON, Bumpless SKIPPED", self._name, sp_delta)
                elif not is_decrease and hvac_mode != VThermHvacMode_COOL:
                     # Increase (Heating):
                     # 1. Deactivate Thermal Guard
                     self._hysteresis_thermal_guard = False
                     # 2. Apply Bumpless Transfer
                     self._bump_integral_for_setpoint_change(
                        t_set_old=self._last_raw_setpoint,
                        t_set_new=target_temp,
                        t_in=current_temp,
                    )
                else:
                    # COOL mode or other: Default behavior (bumpless)
                     self._bump_integral_for_setpoint_change(
                        t_set_old=self._last_raw_setpoint,
                        t_set_new=target_temp,
                        t_in=current_temp,
                    )

        if dt_min < MIN_DT_SECONDS / 60.0 and not setpoint_changed and not is_first_run:

            # Too soon since last calculation - keep existing outputs
            # BUT still capture setpoint changes to not miss user adjustments (in case we didn't catch it above?
            # actually if setpoint_changed is True we don't be here, so this fallback is mostly for safe internal state updates)
            self._filter_setpoint(target_temp, current_temp, hvac_mode, dt_min, advance_ema=False)
            self._calculate_times()
            return

        # Update timestamp for next dt calculation
        self._last_calculate_time = now

        # Cycle management moved to CycleManager
        # self._accumulated_dt logic removed from here as counting is done in on_cycle_completed


        ########################################################################
        #                                                                      #
        #                      STEP 2 - Model Data                     #
        #                      -------------------                     #
        #  Retrieve current a, b and gain candidates                            #
        #                                                                      #
        ########################################################################

        # Get model parameters
        a = self.est.a
        b = self.est.b

        # Check tau reliability
        tau_info = self.est.tau_reliability()
        self._tau_reliable = tau_info.reliable

        # Force unreliable if in WARMUP phase (insufficient learning samples)
        # This ensures we stay in safe mode until we have enough data (a>=10, b>=10)
        if self.phase == SmartPIPhase.WARMUP:
            self._tau_reliable = False

        # Compute gains (simple heuristic based on tau)
        if self._tau_reliable:
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

        self._kp = kp
        self._ki = ki

        ########################################################################
        #                                                                      #
        #                      STEP 3 - Setpoint Filtering             #
        #                      ---------------------------             #
        #  Apply smoothing to setpoint for soft-landing                        #
        #                                                                      #
        ########################################################################

        # Apply asymmetric setpoint filter to reduce overshoot on setpoint changes
        # Always call _filter_setpoint() to keep _filtered_setpoint synchronized for diagnostics
        # But only use the filtered value for PI control when tau is reliable
        # advance_ema=True here because we're in the main PI loop (once per cycle)
        filtered_result = self._filter_setpoint(target_temp, current_temp, hvac_mode, dt_min, advance_ema=True)

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
        if self._setpoint_boost_active or not self._tau_reliable:
            # Bypass weighting for fast response if boosting or if model is not yet reliable (compensates lack of FF)
            e_p = float(e)
        else:
            e_p = float(self.setpoint_weight_b * e)
        # if hvac_mode == VThermHvacMode_COOL:
        #     # Cooling mode: invert the weighted error
        #     e_p = -e_p

        self._last_error_p = e_p

        ########################################################################
        #                                                                      #
        #                      STEP 3.5 - Error Filtering              #
        #                      --------------------------              #
        #  EMA filtering of error to reduce noise transparency                 #
        #                                                                      #
        ########################################################################

        # EMA filtering of error (for diagnostics and gain scheduling stability)
        # Using time constant equivalent
        alpha_err = 1.0 - math.exp(-max(dt_min, 0.001) / ERROR_FILTER_TAU)
        if self._e_filt is None:
            self._e_filt = e
        else:
            self._e_filt = (1 - alpha_err) * self._e_filt + alpha_err * e

        ########################################################################
        #                                                                      #
        #                      STEP 4 - Gain Scheduling                #
        #                      ------------------------                #
        #  Adapt gains based on band and boost state                           #
        #                                                                      #
        ########################################################################

        # Sign-flip detection (for optional integral discharge)
        if self._prev_error is not None and (e * self._prev_error) < 0.0:
            # Only apply if we are close enough to setpoint (avoid messing with large disturbances)
            band = self.sign_flip_band_mult * max(self.near_band_deg, 1e-6)
            if abs(e) <= band and self.sign_flip_leak_cycles > 0 and self.sign_flip_leak > 0.0:
                # Convert cycles to duration approx (using current cycle_min)
                # This makes it robust to irregular updates
                duration_min = float(self.sign_flip_leak_cycles) * max(self._cycle_min, 1.0)
                self._sign_flip_end_ts = time.monotonic() + (duration_min * 60.0)
                self._sign_flip_active = True

        self._prev_error = e

        # Near-setpoint gain scheduling (soft landing)
        # Rule: near-band must never be more aggressive than classic gains.
        # In particular, Ki must not increase in near-band (reduces overshoot / hunting).
        kp_classic = kp
        ki_classic = ki

        # Near-band detection using FILTERED error if available
        # This prevents gain fluctuation due to sensor noise
        err_for_band = self._e_filt if self._e_filt is not None else e

        # Near-band hysteresis (stable gain scheduling).
        # For HEAT mode only, we allow an asymmetric near-band:
        # - slightly wider below setpoint (e>0) to start soft-landing earlier on re-heat
        # - slightly tighter above setpoint (e<0) to reduce overshoot tendency
        if not self._tau_reliable:
            in_near_band_now = False
        else:
            if hvac_mode != VThermHvacMode_COOL:
                nb_below = max(self.near_band_deg, 0.0)
                nb_above = max(self.near_band_deg * NEAR_BAND_ABOVE_FACTOR, 0.0)
                nb_hyst = max(NEAR_BAND_HYSTERESIS_C, 0.0)

                nb_entry = nb_below if err_for_band >= 0.0 else nb_above
                nb_exit = nb_entry + nb_hyst

                abs_err = abs(err_for_band)
                if abs_err <= nb_entry:
                    in_near_band_now = True
                elif abs_err >= nb_exit:
                    in_near_band_now = False
                else:
                    in_near_band_now = self._in_near_band
            else:
                # COOL: keep symmetric near-band behavior (safer)
                in_near_band_now = (self.near_band_deg > 0.0) and (abs(err_for_band) <= self.near_band_deg)

        self._in_near_band = in_near_band_now

        if in_near_band_now:
# Softer proportional action near target
            kp = clamp(kp_classic * self.kp_near_factor, KP_MIN, KP_MAX)

            # Reduce integral action near target
            ki_near = ki_classic * self.ki_near_factor
            ki = min(ki_near, ki_classic)
            ki = clamp(ki, KI_MIN, KI_MAX)

        # DEBUG: Log logic flow for near-band coefficient selection
        if self._tau_reliable:
            # Only trace when active to avoid flooding logs in WARMUP unless important
            # Using debug level 5 equivalent (standard debug)
            _LOGGER.debug(
                "%s - Calc SmartPI gains: in_near_band=%s, error=%.4f (band=%.4f), "
                "Classic[Kp=%.4f, Ki=%.4f], "
                "Applied[Kp=%.4f, Ki=%.4f]",
                self._name, in_near_band_now, e, self.near_band_deg,
                kp_classic, ki_classic,
                kp, ki
            )

        # Store current gains (for diagnostics)
        self.Kp = kp
        self.Ki = ki

        ########################################################################
        #                                                                      #
        #                      STEP 5 - Feed-Forward                   #
        #                      ---------------------                   #
        #  Predictive power based on loss model                                #
        #                                                                      #
        ########################################################################


        # Feed-forward calculation
        # Use target_temp_internal for coherence with the PI controller (filtered setpoint)
        # For HEAT: positive command; For COOL: invert sign of error/FF (very simplistic)
        if ext_current_temp is None:
            # No outdoor temperature available: disable feed-forward to avoid
            # effectively doubling the proportional action with indoor-only data.
            u_ff = 0.0
        else:
            t_ext = ext_current_temp
            if self.est.learn_ok_count_a < 10 or not self._tau_reliable:
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

        # Optional "sign flip leak" soft discharge (dt-aware)
        if self._sign_flip_active:
            now_ts = time.monotonic()
            if self._sign_flip_end_ts is not None and now_ts < self._sign_flip_end_ts:
                # Apply leak scaled by dt
                # leak = base_leak ** dt_min  (assuming sign_flip_leak is "per cycle" ~ "per 10-15min"?)
                # To keep it simple and consistent: interpret sign_flip_leak as "leak per cycle"
                # so we scale it: leak_factor = leak ** (dt / cycle_min)
                # This preserves the original tuning meaning.
                cycle_ref = max(self._cycle_min, 1.0)
                leak_exponent = dt_min / cycle_ref
                leak_factor = (1.0 - self.sign_flip_leak) ** leak_exponent

                self.integral *= leak_factor
                # Keep integral bounded
                self.integral = clamp(self.integral, -i_max, i_max)
            else:
                self._sign_flip_active = False
                self._sign_flip_end_ts = None

        ########################################################################
        #                                                                      #
        #                      STEP 6 - PI Controller Stage            #
        #                      ----------------------------            #
        #  Compute error-driven proportional and integral terms                #
        #                                                                      #
        ########################################################################

        # --- PI control with anti-windup ---
        # Deadband with hysteresis to reduce oscillations at boundary:
        # - Enter deadband when |e| < deadband_c
        # - Exit deadband only when |e| > deadband_c + DEADBAND_HYSTERESIS
        # - In the hysteresis zone, maintain previous state
        abs_e = abs(e)

        # Deadband with hysteresis.
        # Default: symmetric around 0. For HEAT, we optionally use an asymmetric deadband:
        # - slightly wider below setpoint (e>0) to keep a stable "hold" zone and avoid late restart
        # - slightly tighter above setpoint (e<0) to avoid overshoot.
        if not self._tau_reliable:
            in_deadband_now = False
        else:
            if hvac_mode != VThermHvacMode_COOL:
                db_below = max(DEADBAND_BELOW_C, 0.0)
                db_above = max(DEADBAND_ABOVE_C, 0.0)
                h_below = max(DEADBAND_HYST_BELOW_C, 0.0)
                h_above = max(DEADBAND_HYST_ABOVE_C, 0.0)

                db_entry = db_below if e >= 0.0 else db_above
                db_exit = (db_below + h_below) if e >= 0.0 else (db_above + h_above)

                if abs_e < db_entry:
                    in_deadband_now = True
                elif abs_e > db_exit:
                    in_deadband_now = False
                else:
                    in_deadband_now = self._in_deadband
            else:
                # COOL: keep symmetric behavior (safer)
                db_entry = self.deadband_c
                db_exit = self.deadband_c + DEADBAND_HYSTERESIS
                if abs_e < db_entry:
                    in_deadband_now = True
                elif abs_e > db_exit:
                    in_deadband_now = False
                else:
                    in_deadband_now = self._in_deadband

        # Bumpless transfer: on deadband exit, re-initialize integral
        # so that u_ff + Kp*e_p + Ki*I = u_prev (no output discontinuity)
        # Note: Do not apply bumpless transfer if setpoint changed (we want immediate reaction)
        if self._in_deadband and not in_deadband_now and not setpoint_changed:
            if self.Ki > KI_MIN:
                i_bumpless = (self.u_prev - u_ff - self.Kp * e_p) / self.Ki
                self.integral = clamp(i_bumpless, -i_max, i_max)
                _LOGGER.debug(
                    "%s - Bumpless deadband exit: I adjusted to %.4f (u_prev=%.3f)",
                    self._name, self.integral, self.u_prev
                )

        # Update deadband state
        self._in_deadband = in_deadband_now

        if in_deadband_now:
            # In deadband: no PI action; allow FF and (optionally) a small "hold" power when slightly below SP.
            # This prevents "late restart" after a setpoint decrease on slow thermal systems.
            u_pi = 0.0
            self._last_i_mode = "I:FREEZE(deadband)"

            # Optional micro-leak on integral to avoid long-memory bias (dt-aware).
            cycle_ref = max(float(self._cycle_min), 1.0)
            leak_factor = INTEGRAL_DEADBAND_MICROLEAK ** (dt_min / cycle_ref)
            self.integral *= leak_factor
            self.integral = clamp(self.integral, -i_max, i_max)

            # Deadband+ (DB+): if we are slightly below SP (HEAT, e>0), add a minimum holding power.
            # Guardrails:
            # - only when model is reliable (so FF has meaning)
            # - only if user already has some FF/model confidence (learn_ok_count_a)
            # - never apply above setpoint (e<=0), never in COOL
            if (
                hvac_mode != VThermHvacMode_COOL
                and e > 0.0
                and self._tau_reliable
                and self.est.learn_ok_count_a >= 10
            ):
                u_hold = clamp(DEADBAND_PLUS_MIN_U, 0.0, DEADBAND_PLUS_MAX_U)
                # Make sure we never reduce an existing FF command.
                u_total = max(u_ff, u_hold)
                u_pi = u_total - u_ff

        else:
            if integrator_hold:
                # Dead time: don't integrate to avoid pumping
                u_pi = self.Kp * e_p + self.Ki * self.integral
                self._last_i_mode = "I:HOLD"

                # Overshoot / near-overshoot zone (heating): Tin >= SP_int - eps
                if hvac_mode != VThermHvacMode_COOL and current_temp >= (
                    target_temp_internal - OVERSHOOT_I_CLAMP_EPS_C
                ):
                    # Only bleed if I>0; bleeding negative I would increase u (bad in overshoot)
                    if self.integral > 0.0:
                        # Use existing INTEGRAL_LEAK (per cycle) as a controlled bleed toward 0.
                        # Convert to the actual elapsed fraction of a cycle:
                        leak_eff = INTEGRAL_LEAK ** (
                            dt_min / max(1e-9, float(self._cycle_min))
                        )
                        self.integral *= leak_eff
                        self._last_i_mode = "I:BLEED(hold_ovr)"
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
                    d_integral = e * dt_min
                    self._last_i_mode = "I:RUN"
                    
                    # Overshoot / near-overshoot I-clamp (heating):
                    # If we are close to or above the setpoint, do not integrate positively.
                    # This prevents the integrator from increasing heat when we should be reducing it.
                    if hvac_mode != VThermHvacMode_COOL and current_temp >= (target_temp_internal - OVERSHOOT_I_CLAMP_EPS_C):
                        if d_integral > 0.0:
                            d_integral = 0.0
                            self._last_i_mode = "I:CLAMP(near_ovr)"

                    # Thermal Guard: freeze/drop integral if in hysteresis after decrease
                    if self._hysteresis_thermal_guard:
                        if current_temp > target_temp: # Overshoot condition
                            # Only allow decrease (negative d_integral)
                            if d_integral > 0:
                                d_integral = 0.0
                                self._last_i_mode = "I:GUARD(freeze)"
                            else:
                                self._last_i_mode = "I:GUARD(drop)"
                        else:
                            # Temperature has dropped below setpoint -> Release guard
                            self._hysteresis_thermal_guard = False
                            self._last_i_mode = "I:RUN"
                        
                    self.integral += d_integral
                    self.integral = clamp(self.integral, -i_max, i_max)
                    u_pi = self.Kp * e_p + self.Ki * self.integral

        # Combine FF and PI, then clamp to [0,1]
        u_raw = u_ff + u_pi
        u_cmd = clamp(u_raw, 0.0, 1.0)

        # Store PI term for diagnostics
        self._last_u_pi = u_pi
        self._last_u_cmd = u_cmd

        ########################################################################
        #                                                                      #
        #                      STEP 7 - Soft Constraints               #
        #                      -------------------------               #
        #  Rate limits and setpoint boost                                      #
        #                                                                      #
        ########################################################################

        # ------------------------------
        # Apply constraints (rate-limit, max_on_percent), then timing enforcement
        # ------------------------------

        # Setpoint step boost: detect significant setpoint increase (HEAT) or decrease (COOL)
        # and apply faster rate limit to allow quick power ramp-up
        if self._prev_setpoint_for_boost is None:
            self._prev_setpoint_for_boost = target_temp

        sp_delta = target_temp - self._prev_setpoint_for_boost

        # Detect setpoint change that should trigger boost
        if abs(sp_delta) >= SETPOINT_BOOST_THRESHOLD:
            # Setpoint changed significantly (up or down) - activate boost
            self._setpoint_boost_active = True
            self._prev_setpoint_for_boost = target_temp
            _LOGGER.debug(
                "%s - Setpoint boost activated: setpoint change %+.2f°C",
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
        # Exception: if setpoint has just changed significantly, allow immediate jump
        # This prevents locking the output when dt is small (e.g. user interaction)
        # Also allow immediate jump on first run to ensure we output something (since dt=0 -> max_step=0)
        if setpoint_changed or is_first_run:
            u_limited = u_cmd
        else:
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

        ########################################################################
        #                                                                      #
        #                      STEP 8 - Anti-Windup Tracking           #
        #                      -----------------------------           #
        #  Back-calculation for actuator saturation                            #
        #                                                                      #
        ########################################################################

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

        # Detect if timing constraints forced an extreme value.
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

        if (not integrator_hold) and (self.Ki > KI_MIN) and (not self._in_deadband) and (not str(self._last_i_mode).startswith("I:CLAMP")):
            if forced_by_timing:
                # Skip tracking: timing quantization should not influence integral
                self._last_aw_du = 0.0
            else:
                # Model-predicted command using current integrator state
                u_model = u_ff + (self.Kp * e_p + self.Ki * self.integral)

                # Tracking error between applied command and model command
                u_aw_ref = u_limited

                # If max_on_percent is active and clamps the requested command,
                # track against the unclamped request to prevent integral windup.
                if self._max_on_percent is not None and u_cmd > self._max_on_percent + 1e-9:
                    u_aw_ref = u_cmd

                du = u_applied - u_aw_ref
                self._last_aw_du = du

                # Discrete tracking gain beta = dt / Tt (bounded 0..1)
                dt_sec = dt_min * 60.0
                beta = clamp(dt_sec / max(AW_TRACK_TAU_S, dt_sec), 0.0, 1.0)

                # Update integral so that Ki * I compensates du
                d_integral = beta * (du / self.Ki)

                # Safety clamp must be time-scaled to remain invariant when dt varies.
                # Interpret AW_TRACK_MAX_DELTA_I as "per minute" and scale by dt_min.
                max_di = AW_TRACK_MAX_DELTA_I * max(dt_min, 0.0)
                d_integral = clamp(d_integral, -max_di, max_di)
                
                # Thermal Guard: freeze/drop integral if in hysteresis after decrease
                if self._hysteresis_thermal_guard and (current_temp is not None and target_temp is not None):
                     if current_temp > target_temp:
                         d_integral = min(0.0, d_integral)

                self.integral += d_integral
                self.integral = clamp(self.integral, -i_max, i_max)
        else:
            self._last_aw_du = 0.0

        # Store final applied command for next cycle rate-limiting
        # Store final applied command for next cycle rate-limiting
        self.u_prev = u_applied

        # Dead time: record sample for learning
        if target_temp is not None and current_temp is not None:
            self._deadtime_last_status = self._deadtime_est.record_sample(target_temp, current_temp)
            if self._deadtime_last_status and self._deadtime_last_status.startswith("learned"):
                _LOGGER.info("%s - DeadTime %s", self._name, self._deadtime_last_status)

        # Update timing properties (_on_time_sec, etc.)
        self._on_percent = u_applied
        self._calculate_times()

    # ------------------------------
    # Timings and diagnostics
    # ------------------------------

    ########################################################################
    #                                                                      #
    #                      DIAGNOSTICS & TIMINGS                           #
    #                                                                      #
    ########################################################################

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
            "sign_flip_active": self._sign_flip_active,
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
            "learning_resume_ts": int(self._learning_resume_ts) if self._learning_resume_ts else None,
            # Anti-windup tracking diagnostics
            "u_cmd": round(self._last_u_cmd, 6),
            "u_limited": round(self._last_u_limited, 6),
            "u_applied": round(self._last_u_applied, 6),
            "aw_du": round(self._last_aw_du, 6),
            "forced_by_timing": self._last_forced_by_timing,
            # Deadband state
            "in_deadband": self._in_deadband,
            "in_near_band": self._in_near_band,
            "in_near_band": self._in_near_band,
            # Setpoint boost state
            "setpoint_boost_active": self._setpoint_boost_active,
            "hysteresis_thermal_guard": self._hysteresis_thermal_guard,
            # Dead time (Phase 1)
            "deadtime_s": None if self._deadtime_est.deadtime_s is None else round(self._deadtime_est.deadtime_s, 0),
            "deadtime_reliable": self._deadtime_est.reliable,
            "deadtime_sample_count": self._deadtime_est.sample_count,
            "deadtime_pending": self._deadtime_est.pending,
            "deadtime_last_status": self._deadtime_last_status,
        }
