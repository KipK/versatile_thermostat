########################################################################
#                                                                      #
#                      Smart-PI Algorithm                              #
#                      ------------------                              #
#  Auto-adaptive PI controller for Versatile Thermostat (VTH).         #
#                                                                      #
########################################################################

"""
SmartPI Algorithm (v2) - Auto-adaptive PI controller for Versatile Thermostat.

This module implements a duty-cycle PI controller for slow thermal systems (heating),
with on-line identification of a 1st-order loss model + dead time.

Key features (v2)
-----------------
1. Hybrid Learning Phases:
   - Hysteresis: Initial ON/OFF control to generate strong signal for learning A/B.
   - Stable: Adaptive PI control once model is reliable (31+ samples).

2. Measurement & Modeling:
   - Online learning of heating efficacy (a) and loss (b) via robust Median + MAD estimation.
   - Automatic Dead Time (L) estimation using Takeoff (Primary) and SK (Backup).

3. Adaptive Control:
   - Feed-forward compensation based on outdoor temperature.
   - PI gains automatically tuned based on inertia (Tau) and Dead Time (IMC-like).
   - Auto-adaptive Near-Band: Reduces gains near setpoint to prevent overshoot,
     sized dynamically based on Dead Time.

4. Comfort & Protections:
   - Setpoint Boost: Faster reaction to manual setpoint increases (>0.3°C).
   - Thermal Guard: Prevents integral windup on setpoint decreases.
   - Asymmetric Setpoint Filter: Soft-landing on temperature rise.
   - Anti-windup: Conditional integration + Tracking anti-windup.

The output is a power command between 0 and 1 (0-100%).
"""

from __future__ import annotations

import logging
import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Deque, Dict, Literal, Optional, Tuple
from enum import Enum

from .vtherm_hvac_mode import (
    VThermHvacMode,
    VThermHvacMode_COOL,
    VThermHvacMode_HEAT,
    VThermHvacMode_OFF,
)
from .cycle_manager import CycleManager
from homeassistant.core import HomeAssistant

from .smartpi.const import (
    SmartPIPhase,
    GovernanceRegime,
    FreezeReason,
    GovernanceDecision,
    SmartPICalibrationPhase,
    GOVERNANCE_MATRIX,
    KP_SAFE,
    KI_SAFE,
    KP_MIN,
    KP_MAX,
    KI_MIN,
    KI_MAX,
    INTEGRAL_LEAK,
    MAX_STEP_PER_MINUTE,
    SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN,
    SETPOINT_BOOST_RATE,
    SETPOINT_MODE_DELTA_C,
    SETPOINT_BUMPLESS_MAX_DU,
    OVERSHOOT_I_CLAMP_EPS_C,
    AW_TRACK_TAU_S,
    AW_TRACK_MAX_DELTA_I,
    SKIP_CYCLES_AFTER_RESUME,
    LEARNING_PAUSE_RESUME_MIN,
    SMARTPI_RECALC_INTERVAL_SEC,
    HYST_UPPER_C,
    HYST_LOWER_C,
    DEFAULT_DEADBAND_C,
    DEADBAND_HYSTERESIS,
    DEADBAND_BELOW_C,
    DEADBAND_ABOVE_C,
    DEADBAND_HYST_BELOW_C,
    DEADBAND_HYST_ABOVE_C,
    DEADBAND_PLUS_MIN_U,
    DEADBAND_PLUS_MAX_U,
    INTEGRAL_DEADBAND_MICROLEAK,
    NEAR_BAND_ABOVE_FACTOR,
    NEAR_BAND_HYSTERESIS_C,
    SP_TAU_SLOW,
    SP_TAU_FAST,
    SP_BAND,
    SP_BYPASS_ERROR_THRESHOLD,
    ERROR_FILTER_TAU,
    B_POINTS_MAX,
    A_POINTS_MAX,
    RESIDUAL_HIST_MAX,
    RESIDUAL_GATE_K,
    INTERCEPT_SIGMA_FACTOR,
    INTERCEPT_SCALE_FACTOR,
    B_STABILITY_MAD_RATIO_MAX,
    LEARN_BOOTSTRAP_COUNT,
    AB_HISTORY_SIZE,
    AB_MIN_SAMPLES,
    AB_MAD_SIGMA_MULT,
    AB_MAD_K,
    AB_VAL_TOLERANCE,
    LEARN_SAMPLE_MAX,
    LEARN_Q_HIST_MAX,
    DT_MIN_OK,
    DT_MAX_OK,
    DT_DERIVATIVE_MIN_ABS,
    LEARN_QUALITY_THRESHOLD,
    QUANTIZATION_ROUND_TO,
    DT_MAX_MIN,
    MIN_ABS_DT,
    DELTA_MIN,
    U_OFF_MAX,
    U_ON_MIN,
    DELTA_MIN_OFF,
    DELTA_MIN_ON,
    EPISODE_MIN_DURATION_ON_S,
    EPISODE_MIN_DURATION_OFF_S,
    DEFAULT_NEAR_BAND_DEG,
    DEFAULT_KP_NEAR_FACTOR,
    DEFAULT_KI_NEAR_FACTOR,
    FORCE_CALIBRATION_INTERVAL_HOURS,
    CALIBRATION_RETRY_MAX,
    CALIBRATION_TIMEOUT_MIN,
    clamp
)
from .smartpi.learning import DeadTimeEstimator, ABEstimator, TauReliability
from .smartpi.diagnostics import build_diagnostics
from .smartpi.governance import SmartPIGovernance
from .smartpi.setpoint import SmartPISetpointManager
from .smartpi.controller import SmartPIController
from .smartpi.learning_window import LearningWindowManager
from .smartpi.deadband import DeadbandManager
from .smartpi.calibration import CalibrationManager
from .smartpi.gains import GainScheduler

_LOGGER = logging.getLogger(__name__)





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

        # --- Sub-Systems ---
        self.gov = SmartPIGovernance(name)
        self.sp_mgr = SmartPISetpointManager(name, enabled=use_setpoint_filter)
        self.ctl = SmartPIController(name)

        # Model estimator
        self.est = ABEstimator()

        # --- New Component Managers (Phase 2.5 refactoring) ---
        self.learn_win = LearningWindowManager(name)
        self.deadband_mgr = DeadbandManager(name, near_band_deg)
        self.calibration_mgr = CalibrationManager(name)
        self.gain_scheduler = GainScheduler(name)

        # PI state (delegated to self.ctl)
        # self.integral and self.u_prev are now properties

        # Error filtering (EMA) - kept for diagnostics / potential future use
        self._e_filt: Optional[float] = None
        # self._ema_alpha: float = 0.35  # Deprecated: using ERROR_FILTER_TAU

        # Current gains
        self.Kp: float = KP_SAFE
        self.Ki: float = KI_SAFE
        self._kp: float = KP_SAFE
        self._ki: float = KI_SAFE
        self._prev_kp: float = KP_SAFE
        self._prev_ki: float = KI_SAFE

        # Outputs (duty-cycle only, timing calculated by handler)
        self._on_percent: float = 0.0

        # Diagnostics / status
        self._last_u_ff: float = 0.0
        self._last_u_pi: float = 0.0
        self._last_error: float = 0.0
        self._last_error_p: float = 0.0
        self._last_i_mode: str = "init"
        self._tau_reliable: bool = False
        self._last_sat: str = "init"
        self._sign_flip_active: bool = False

        # Sign-flip leak helper (removed/simplified)

        # Asymmetric setpoint EMA filter state (delegated to self.sp_mgr)
        
        # Track last time calculate() was executed for dt-based integration
        self._last_calculate_time: Optional[float] = None
        # Accumulated time for cycle counting (used for FF warm-up)
        self._accumulated_dt: float = 0.0

        # Timestamp for robust learning dt calculation
        self._learn_last_ts: float | None = None
        
        # Track last target temp for learning invalidation
        self._last_target_temp = None

        # Learning window state (multi-cycle learning)
        # Properties delegate to learn_win manager

        # Learning start timestamp
        self._learning_start_date: Optional[datetime] = datetime.now()

        # Skip learning cycles after resume from interruption (window close, etc.)
        self._learning_resume_ts: Optional[float] = None
        # Helper to distinguish Startup (Init) from Resume (OFF->ON)
        # We want to pause learning on Resume, but NOT on Startup/Reboot
        self._startup_grace_period: bool = True

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
        self._output_initialized: bool = False # True once calculate() runs successfully

        # Setpoint step boost state (delegated to self.sp_mgr)

        # Enhanced A/B Learning: Start-of-cycle snapshot managed by CycleManager

        # Thermal Guard (delegated to self.ctl)
        
        # Hysteresis state (delegated to self.ctl)

        # --- Dead Time (L) Support (Smart-PI v2) ---
        self.dt_est = DeadTimeEstimator()
        self._heat_request_prev: bool = False
        self._t_heat_episode_start: float | None = None
        self._t_cool_episode_start: float | None = None
        self._deadtime_skip_count_a: int = 0
        self._deadtime_skip_count_b: int = 0
        
        # Feature flag for integral freeze during deadtime (Default OFF)
        self.feature_integral_freeze: bool = False
        self._kp_source: str = "heuristic" # "heuristic" or "imc_deadtime"

        # --- Near-Band Auto-Tuning (Phase 2) ---
        self._near_band_below_deg: float = self.near_band_deg
        self._near_band_above_deg: float = self.near_band_deg * NEAR_BAND_ABOVE_FACTOR
        self._near_band_source: str = "manual" # "manual", "auto", "fallback"

        # --- Forced Calibration State ---
        self._last_calibration_time: float | None = None
        self._calibration_state: SmartPICalibrationPhase = SmartPICalibrationPhase.IDLE
        self._calibration_start_time: float | None = None
        # _force_calibration_requested is now a property delegating to calibration_mgr
        self._calibration_retry_count: int = 0

        # --- Safety-First Governance (Delegated to self.gov) ---


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
        if self.ctl: self.ctl.reset()
        if self.sp_mgr: self.sp_mgr.reset()
        if self.gov: self.gov.reset()
        
        self._on_percent = 0.0
        self._output_initialized = False
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
        
        # Reset learning states
        self._last_calculate_time = None
        self._learn_last_ts = None
        self._last_target_temp = None
        self._learning_start_date = datetime.now()
        self._learning_resume_ts = None
        self._in_deadband = False
        self._in_near_band = False
        
        # Learning window state is managed by learn_win component
        
        # Reset Dead Time Estimator
        self.dt_est.reset()
        self._heat_request_prev = False
        self._t_heat_episode_start = None
        self._deadtime_skip_count_a = 0
        self._deadtime_skip_count_b = 0
        self._kp_source = "heuristic"
        
        # Reset Phase 2
        self._near_band_below_deg = self.near_band_deg 
        self._near_band_above_deg = self.near_band_deg * NEAR_BAND_ABOVE_FACTOR
        self._near_band_source = "manual"

        # Reset Calibration
        self._last_calibration_time = None
        self._calibration_state = SmartPICalibrationPhase.IDLE
        self._calibration_start_time = None
        # _force_calibration_requested is now a property, no need to reset
        self._calibration_retry_count = 0

        # Governance is reset above (self.gov.reset())

        # Reset new component managers (Phase 2.5 refactoring)
        if self.learn_win: self.learn_win.reset()
        if self.deadband_mgr: self.deadband_mgr.reset()
        if self.calibration_mgr: self.calibration_mgr.reset()
        if self.gain_scheduler: self.gain_scheduler.reset()

        _LOGGER.info("%s - SmartPI learning and history reset", self._name)

        _LOGGER.info("%s - SmartPI learning and history reset", self._name)

    @property
    def calibration_state(self) -> SmartPICalibrationPhase:
        """Return current calibration state, delegating to CalibrationManager."""
        return self.calibration_mgr.state if self.calibration_mgr else self._calibration_state

    @property
    def _force_calibration_requested(self) -> bool:
        """Delegate to CalibrationManager for sync access."""
        return self.calibration_mgr.calibration_requested if self.calibration_mgr else False

    def force_calibration(self) -> None:
        """Force a calibration cycle to refresh Dead Time estimation."""
        # Delegate to CalibrationManager component
        self.calibration_mgr.request_calibration()


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

        # Notify DeadTimeEstimator of interruption



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

    # ------------------------------
    # Property Mappings (Legacy Support)
    # ------------------------------

    @property
    def _filtered_setpoint(self) -> float | None:
        return self.sp_mgr.filtered_setpoint

    @_filtered_setpoint.setter
    def _filtered_setpoint(self, value: float | None):
        self.sp_mgr.filtered_setpoint = value

    @property
    def _last_raw_setpoint(self) -> float | None:
        return self.sp_mgr.last_raw_setpoint
        
    @_last_raw_setpoint.setter
    def _last_raw_setpoint(self, value: float | None):
         self.sp_mgr.last_raw_setpoint = value

    @property
    def _initial_temp_for_filter(self) -> float | None:
        return self.sp_mgr.initial_temp_for_filter
        
    @_initial_temp_for_filter.setter
    def _initial_temp_for_filter(self, value: float | None):
         self.sp_mgr.initial_temp_for_filter = value

    @property
    def _setpoint_boost_active(self) -> bool:
        return self.sp_mgr.boost_active
        
    @_setpoint_boost_active.setter
    def _setpoint_boost_active(self, value: bool):
        self.sp_mgr.boost_active = value

    @property
    def _prev_setpoint_for_boost(self) -> float | None:
        return self.sp_mgr.prev_setpoint_for_boost
        
    @_prev_setpoint_for_boost.setter
    def _prev_setpoint_for_boost(self, value: float | None):
        self.sp_mgr.prev_setpoint_for_boost = value
        
    @property
    def _hysteresis_thermal_guard(self) -> bool:
        return self.ctl.hysteresis_thermal_guard
        
    @_hysteresis_thermal_guard.setter
    def _hysteresis_thermal_guard(self, value: bool):
        self.ctl.hysteresis_thermal_guard = value

    @property
    def _cycle_regimes(self):
        return self.gov._cycle_regimes
        
    @property
    def _current_governance_regime(self):
        return self.gov._current_regime
        
    @property
    def _governance_decision_thermal(self):
        return self.gov.last_decision_thermal
        
    @property
    def _governance_reason_thermal(self):
        return self.gov.last_reason_thermal
        
    @property
    def _governance_decision_gains(self):
        return self.gov.last_decision_gains
        
    @property
    def _governance_reason_gains(self):
        return self.gov.last_reason_gains

    @property
    def _hysteresis_state(self) -> str:
        return self.ctl.hysteresis_state

    @property
    def phase(self) -> str:
        """Current phase of the algorithm."""
        if self._calibration_state != SmartPICalibrationPhase.IDLE:
            return SmartPIPhase.CALIBRATION
        # Hysteresis until we have AB_HISTORY_SIZE (31) measurements for both A and B
        if len(self.est.a_meas_hist) < AB_HISTORY_SIZE or len(self.est.b_meas_hist) < AB_HISTORY_SIZE:
            return SmartPIPhase.HYSTERESIS
        return SmartPIPhase.STABLE

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
        """Reset the multi-cycle learning window state.
        
        Delegates to the LearningWindowManager component.
        """
        self.learn_win.reset()

    def update_learning(
        self,
        dt_min: float,
        current_temp: float,
        ext_temp: float,
        u_active: float,
        setpoint_changed: bool = False
    ) -> None:
        """
        Feed learning data from the heartbeat tick.
        
        Delegates to the LearningWindowManager component.
        
        Args:
            dt_min: Elapsed time in minutes since last update
            current_temp: Current indoor temperature
            ext_temp: Current outdoor temperature
            u_active: Power applied during this interval (0..1)
            setpoint_changed: True if setpoint changed during this interval
        """
        now = time.monotonic()
        
        # Delegate to LearningWindowManager
        self._deadtime_skip_count_a, self._deadtime_skip_count_b = self.learn_win.update(
            dt_min=dt_min,
            current_temp=current_temp,
            ext_temp=ext_temp,
            u_active=u_active,
            setpoint_changed=setpoint_changed,
            estimator=self.est,
            dt_est=self.dt_est,
            governance=self.gov,
            learning_resume_ts=self._learning_resume_ts,
            now=now,
            in_deadband=self._in_deadband,
            in_near_band=self._in_near_band,
            t_heat_episode_start=self._t_heat_episode_start,
            t_cool_episode_start=self._t_cool_episode_start,
            deadtime_skip_count_a=self._deadtime_skip_count_a,
            deadtime_skip_count_b=self._deadtime_skip_count_b,
        )
        
        # Sync learning_resume_ts from component (may have been cleared)
        self._learning_resume_ts = self.learn_win.learning_resume_ts
    async def on_cycle_started(self, on_time_sec: float, off_time_sec: float, on_percent: float, hvac_mode: str) -> None:
        """Called when a cycle starts."""
        await super().on_cycle_started(on_time_sec, off_time_sec, on_percent, hvac_mode)
        self._setpoint_changed_in_cycle = False
        # Update internal on_percent to match applied value
        self._on_percent = on_percent
        # Reset governance regime tracking for new cycle
        self._cycle_regimes.clear()

    async def on_cycle_completed(self, new_params: dict, prev_params: dict | None) -> bool:
        """Handle end of cycle (learning). Return False to extend window."""
        await super().on_cycle_completed(new_params, prev_params)

        if prev_params is None:
            # First cycle or check-in, nothing to learn yet
            return True

        # 1. Retrieve Context
        # Note: on_cycle_completed is now mainly used for cycle counting loops.
        # Learning accumulation is done via update_learning() in calculate().
        
        # We can perform checks here if we want to invalidate the WHOLE cycle retroactively
        # but update_learning handles it better in real-time.
        
        # Just return True to allow logging/counting.
        pass        # 5. Multi-cycle learning window logic
        # MOVED TO update_learning() called by calculate() heartbeat.
        # This method now only handles cycle counting/stats if needed.
        
        # Cycle accepted -> Count it
        self._cycles_since_reset += 1

        # NOTE: Slope collection for Near-Band is also moved to update_learning
        # or calculate() if needed.
        
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
    def last_governance_decision_thermal(self) -> str:
        return self.gov.last_decision_thermal.value

    @property
    def last_decision_thermal(self) -> str:
        return self.gov.last_decision_thermal.value

    @property
    def last_freeze_reason_thermal(self) -> str:
        return self.gov.last_freeze_reason_thermal.value

    @property
    def freeze_reason_thermal(self) -> str:
        return self.gov.last_freeze_reason_thermal.value

    @property
    def freeze_reason_thermal(self) -> str:
        return self.gov.last_freeze_reason_thermal.value

    @property
    def last_governance_decision_gains(self) -> str:
        return self.gov.last_decision_gains.value

    @property
    def last_decision_gains(self) -> str:
        return self.gov.last_decision_gains.value

    @property
    def freeze_reason_gains(self) -> str:
        return self.gov.last_freeze_reason_gains.value

    @property
    def last_freeze_reason_gains(self) -> str:
        return self.gov.last_freeze_reason_gains.value

    @property
    def freeze_reason_gains(self) -> str:
        return self.gov.last_freeze_reason_gains.value

    @property
    def i_mode(self) -> str:
        return self.ctl.last_i_mode

    @property
    def last_i_mode(self) -> str:
        return self.ctl.last_i_mode

    @property
    def integral(self) -> float:
        """Current integral accumulator value (delegated to controller)."""
        return self.ctl.integral

    @integral.setter
    def integral(self, value: float) -> None:
        self.ctl.integral = value

    @property
    def u_prev(self) -> float:
        """Previously applied power (last_on_percent)."""
        return self.ctl.u_prev

    @u_prev.setter
    def u_prev(self, value: float) -> None:
        self.ctl.u_prev = value
    @property
    def sat(self) -> str:
        return self.ctl.last_sat

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
        """Return monotonic resume timestamp."""
        return self._learning_resume_ts

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

    def update_timing_constraints(self, u_prev: float, u_cmd: float) -> float:
        """Apply min on/off delays and update forced_by_timing status."""
        self._last_forced_by_timing = False
        
        u_final = u_cmd
        cycle_sec = self._cycle_min * 60
        on_time_sec = u_cmd * cycle_sec
        
        # Case: Switching ON
        if u_prev <= 0.001 and u_cmd > 0.001:
            if 0.001 < on_time_sec < self._minimal_activation_delay:
                u_final = 0.0
                self._last_forced_by_timing = True
        
        # Case: Switching OFF (simplified, usually handled by handler)
        # but if we force it, we should report it.
        
        return u_final

    @property
    def in_deadband(self) -> bool:
        return self._in_deadband

    @property
    def in_near_band(self) -> bool:
        return self._in_near_band

    # Learning window properties - delegate to learn_win component
    @property
    def learn_win_active(self) -> bool:
        """Return True if a learning window is currently active."""
        return self.learn_win.active if self.learn_win else False

    @property
    def learn_win_start_ts(self) -> float | None:
        """Return the monotonic timestamp when the current window started."""
        return self.learn_win.start_ts if self.learn_win else None

    @property
    def learn_T_int_start(self) -> float:
        """Return the indoor temperature at window start."""
        return self.learn_win.T_int_start if self.learn_win else 0.0

    @property
    def learn_T_ext_start(self) -> float:
        """Return the outdoor temperature at window start."""
        return self.learn_win.T_ext_start if self.learn_win else 0.0

    @property
    def learn_u_int(self) -> float:
        """Return the accumulated power integral (u * dt)."""
        return self.learn_win.u_int if self.learn_win else 0.0

    @property
    def learn_t_int_s(self) -> float:
        """Return the accumulated time integral in seconds."""
        return self.learn_win.t_int_s if self.learn_win else 0.0

    @property
    def learn_u_first(self) -> float | None:
        """Return the first power value in the window (for consistency check)."""
        return self.learn_win.u_first if self.learn_win else None

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
    def in_deadtime_window(self) -> bool:
        """Check if we are currently inside the estimated dead time window."""
        now = time.monotonic()
        
        # Check Heat Deadtime
        if self.dt_est.deadtime_heat_reliable and self._t_heat_episode_start is not None and self.dt_est.deadtime_heat_s is not None:
            dt_s = self.dt_est.deadtime_heat_s
            t_start = self._t_heat_episode_start
            # Robustness: ensure both are real numbers (not MagicMock)
            if isinstance(dt_s, (int, float)) and isinstance(t_start, (int, float)):
                 elapsed = now - t_start
                 if elapsed < dt_s:
                     return True

        # Check Cool Deadtime
        if self.dt_est.deadtime_cool_reliable and self._t_cool_episode_start is not None and self.dt_est.deadtime_cool_s is not None:
            dt_s = self.dt_est.deadtime_cool_s
            t_start = self._t_cool_episode_start
            # Robustness: ensure both are real numbers (not MagicMock)
            if isinstance(dt_s, (int, float)) and isinstance(t_start, (int, float)):
                 elapsed = now - t_start
                 if elapsed < dt_s:
                     return True

        return False

    # Phase 2: Helper for Near-Band Auto-Calculation
    def _update_near_band_auto(self, hvac_mode: VThermHvacMode, current_temp: float, ext_temp: Optional[float]) -> None:
        """
        Calculate Near-Band thresholds based on Dead Time and Model Slopes (Phase 2).
        
        Delegates to DeadbandManager component.
        """
        # Delegate to DeadbandManager component
        self.deadband_mgr._update_near_band_auto(
            hvac_mode=hvac_mode,
            current_temp=current_temp,
            ext_temp=ext_temp,
            dt_est=self.dt_est,
            estimator=self.est,
            cycle_min=self.cycle_min,
        )
        # Sync state from component
        self._near_band_below_deg = self.deadband_mgr.near_band_below_deg
        self._near_band_above_deg = self.deadband_mgr.near_band_above_deg
        self._near_band_source = self.deadband_mgr.near_band_source

    # ------------------------------
    # Main control law
    # ------------------------------

    ########################################################################
    #                                                                      #
    #                      CONTROL LAW                                     #
    #                                                                      #
    ########################################################################

    def _calculate_forced_calibration(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode: VThermHvacMode,
    ) -> None:
        """Execute the forced calibration state machine.
        
        Delegates to CalibrationManager component.
        """
        # Delegate to CalibrationManager component
        result = self.calibration_mgr.calculate(
            target_temp=target_temp,
            current_temp=current_temp,
            hvac_mode=hvac_mode,
            dt_est=self.dt_est,
            max_on_percent=self._max_on_percent,
        )
        
        # Sync state from component
        self._calibration_state = result.phase
        if result.phase == SmartPICalibrationPhase.IDLE and result.message == "completed":
            self._last_calibration_time = self.calibration_mgr.last_calibration_time
            self._calibration_start_time = None
        
        # Apply result
        if result.on_percent is not None:
            self._on_percent = result.on_percent
        else:
            self._on_percent = 0.0

        # Update diagnostics
        self._last_i_mode = "CALIB"
        self._last_sat = "NO_SAT"
        self._last_u_ff = 0.0
        self._last_u_pi = self._on_percent
        self._last_u_cmd = self._on_percent
        self._last_u_limited = self._on_percent
        self._last_u_applied = self._on_percent
        self.u_prev = self._on_percent  # Important for learning/estimator

    def update_realized_power(
        self,
        u_applied: float | None = None,
        dt_min: float = 0.0,
        forced_by_timing: bool = False,
        realized_percent: float | None = None,
        **kwargs
    ) -> None:
        """
        Adjust integral term based on REALIZED power (Energy Awareness).
        Called by handler if actual heater output differed from command.
        """
        # Resolve argument name differences for compatibility with various tests
        val = realized_percent if realized_percent is not None else u_applied
        if val is None:
             # Handle rare cases where it might be called with positional arg only
             return

        # 1. Update states
        self._last_u_applied = val
        self._last_forced_by_timing = forced_by_timing
        self.u_prev = val

        # 2. Skip if no timing info or in deadband
        if dt_min <= 0 or self._in_deadband or abs(self.Ki) < 1e-6:
            return

        # 3. Tracking Anti-Windup Logic
        # If forced by timing, we skip tracking to avoid artificial integral drift
        if forced_by_timing:
            self._last_aw_du = 0.0
            return

        # Calculate tracking reference (u_aw_ref)
        u_aw_ref = self._last_u_limited
        max_on = self._max_on_percent if self._max_on_percent is not None else 1.0
        
        # If command was saturated, the "true" unconstrained command is the reference
        if self._last_u_cmd > max_on - 0.001 and self._last_u_limited >= max_on - 0.001:
            u_aw_ref = self._last_u_cmd
        elif self._last_u_cmd < 0.001 and self._last_u_limited <= 0.001:
            u_aw_ref = self._last_u_cmd
            
        du = val - u_aw_ref
        self._last_aw_du = du

        # Energy Awareness: Adjust integral if applied power differed from reference
        if abs(du) > 0.001:
            i_max = self.ctl.config.i_max
            old_i = self.integral
            self.integral = clamp(self.integral + du, -i_max, i_max)
            _LOGGER.debug(
                "%s - Realized adjustment: du=%.3f -> integral %.2f -> %.2f",
                self._name, du, old_i, self.integral
            )

    def _convert_monotonic_to_wall_ts(self, monotonic_ts: float | None) -> float | None:
        """Convert a monotonic timestamp to wall clock time for persistence.
        
        Args:
            monotonic_ts: Monotonic timestamp or None
            
        Returns:
            Wall clock timestamp (time.time()) or None if already None or expired
        """
        if monotonic_ts is None:
            return None
        remaining = monotonic_ts - time.monotonic()
        if remaining > 0:
            return time.time() + remaining
        return None

    def _convert_wall_to_monotonic_ts(self, wall_ts: float | None) -> float | None:
        """Convert a wall clock timestamp to monotonic timestamp.
        
        Args:
            wall_ts: Wall clock timestamp (time.time()) or None
            
        Returns:
            Monotonic timestamp or None if already None or expired
        """
        if wall_ts is None:
            return None
        delay = wall_ts - time.time()
        if delay > 0:
            return time.monotonic() + delay
        return None

    def save_state(self) -> dict:
        """Save algorithm state for persistence."""
        state = {
            "version": 2,
            "on_percent": self._on_percent,
            "last_target_temp": self._last_target_temp,
            "last_calibration_time": self._last_calibration_time,
            "cycles_since_reset": self._cycles_since_reset,
            "learning_start_date": self._learning_start_date.isoformat() if self._learning_start_date else None,
            "in_deadband": self._in_deadband,
            "in_near_band": self._in_near_band,
            # Convert monotonic timestamp to wall clock time for persistence
            "learning_resume_ts": self._convert_monotonic_to_wall_ts(self._learning_resume_ts),
            # Backward compatibility for boost state at top level
            "setpoint_boost_active": self._setpoint_boost_active,
            "prev_setpoint_for_boost": self._prev_setpoint_for_boost,
            "est_state": self.est.save_state(),
            "dt_est_state": self.dt_est.save_state(),
            "gov_state": self.gov.save_state(),
            "ctl_state": self.ctl.save_state(),
            "sp_mgr_state": self.sp_mgr.save_state() if hasattr(self.sp_mgr, "save_state") else {},
            # New component states
            "lw_state": self.learn_win.save_state() if hasattr(self.learn_win, "save_state") else {},
            "db_state": self.deadband_mgr.save_state() if hasattr(self.deadband_mgr, "save_state") else {},
            "cal_state": self.calibration_mgr.save_state() if hasattr(self.calibration_mgr, "save_state") else {},
            "gs_state": self.gain_scheduler.save_state() if hasattr(self.gain_scheduler, "save_state") else {}
        }
        return state

    @staticmethod
    def _migrate_old_state_format(state: dict) -> dict:
        """
        Migrate old flat-key state format to new nested format.
        
        This function converts the legacy flat-key format (where all state
        was stored at the top level) to the new nested format (where each
        component's state is stored in a separate sub-dict).
        
        Args:
            state: State dict in either old or new format
            
        Returns:
            State dict in new nested format
        """
        # Already in new format
        if "est_state" in state:
            # Still need to handle top-level in_deadband/in_near_band for
            # backward compatibility with states saved before component migration
            result = dict(state)
            if "in_deadband" in state or "in_near_band" in state:
                db_state = dict(state.get("db_state", {}))
                if "in_deadband" in state:
                    db_state["in_deadband"] = state["in_deadband"]
                if "in_near_band" in state:
                    db_state["in_near_band"] = state["in_near_band"]
                result["db_state"] = db_state
            return result
        
        # Migrate old flat-key format to new nested format
        return {
            "version": 2,
            "on_percent": state.get("on_percent", 0.0),
            "last_target_temp": state.get("last_target_temp"),
            "cycles_since_reset": state.get("cycles_since_reset", 0),
            "learning_start_date": state.get("learning_start_date"),
            "learning_resume_ts": state.get("learning_resume_ts"),
            "accumulated_dt": state.get("accumulated_dt", 0.0),
            "in_deadband": state.get("in_deadband", False),
            "in_near_band": state.get("in_near_band", False),
            "setpoint_boost_active": state.get("setpoint_boost_active", False),
            "prev_setpoint_for_boost": state.get("prev_setpoint_for_boost"),
            # Component states
            "est_state": {
                "a": state.get("a"),
                "b": state.get("b"),
                "learn_ok_count": state.get("learn_ok_count", 0),
                "learn_ok_count_a": state.get("learn_ok_count_a", 0),
                "learn_ok_count_b": state.get("learn_ok_count_b", 0),
                "learn_skip_count": state.get("learn_skip_count", 0),
                "a_meas_hist": state.get("a_meas_hist", []),
                "b_meas_hist": state.get("b_meas_hist", []),
                "b_hat_hist": state.get("b_hat_hist", []),
            },
            "dt_est_state": {
                "deadtime_heat_s": state.get("deadtime_heat_s"),
                "deadtime_cool_s": state.get("deadtime_cool_s"),
                "deadtime_heat_reliable": state.get("deadtime_heat_reliable", state.get("deadtime_reliable", False)),
                "deadtime_cool_reliable": state.get("deadtime_cool_reliable", False),
                "history_heat": state.get("deadtime_samples", []),
                "history_cool": state.get("deadtime_samples_cool", []),
            },
            "gov_state": {
                "governance_regime": state.get("governance_regime"),
                "freeze_reason_thermal": state.get("freeze_reason_thermal"),
                "freeze_reason_gains": state.get("freeze_reason_gains"),
                "governance_decision_thermal": state.get("governance_decision_thermal"),
                "governance_decision_gains": state.get("governance_decision_gains"),
            },
            "ctl_state": {
                "integral": state.get("integral"),
                "u_prev": state.get("u_prev"),
                "hysteresis_thermal_guard": state.get("hysteresis_thermal_guard"),
            },
            "sp_mgr_state": {
                "filtered_setpoint": state.get("filtered_setpoint"),
                "last_raw_setpoint": state.get("last_raw_setpoint"),
                "initial_temp_for_filter": state.get("initial_temp_for_filter"),
                "setpoint_boost_active": state.get("setpoint_boost_active", False),
                "prev_setpoint_for_boost": state.get("prev_setpoint_for_boost"),
            },
            # Learning window state is intentionally DISCARDED on reboot
            # to ensure a fresh start after interruption
            "lw_state": {
                "learn_win_active": False,
                "learn_win_start_ts": None,
                "learn_T_int_start": None,
                "learn_T_ext_start": None,
                "learn_u_int": 0.0,
                "learn_t_int_s": None,
                "learn_u_first": None,
                "learning_start_date": state.get("learning_start_date"),
                "learning_resume_ts": state.get("learning_resume_ts"),
            },
            "db_state": {
                "in_deadband": state.get("in_deadband", False),
                "in_near_band": state.get("in_near_band", False),
                "near_band_below_deg_auto": state.get("near_band_below_deg_auto"),
                "near_band_above_deg_auto": state.get("near_band_above_deg_auto"),
            },
            "cal_state": {
                "last_calibration_time": state.get("last_calibration_time"),
                "calibration_state": state.get("calibration_state"),
                "calibration_start_time": state.get("calibration_start_time"),
                "force_calibration_requested": state.get("force_calibration_requested", False),
                "calibration_retry_count": state.get("calibration_retry_count", 0),
            },
            "gs_state": {
                "kp": state.get("kp"),
                "ki": state.get("ki"),
                "kp_source": state.get("kp_source"),
                "ki_source": state.get("ki_source"),
            },
            # Legacy key for learning resume conversion
            "skip_learning_cycles_left": state.get("skip_learning_cycles_left", 0),
        }

    def load_state(self, state: dict) -> None:
        """Restore algorithm state with backward compatibility for old flat-key format."""
        if not state:
            return
        
        # Migrate old format to new format (handles both old and new formats)
        migrated = self._migrate_old_state_format(state)
        
        # Log if migration occurred
        if "est_state" not in state:
            _LOGGER.info("%s - Migrated old flat-key state format to new nested format", self._name)
        
        # Extract component states
        est_state = migrated.get("est_state", {})
        dt_est_state = migrated.get("dt_est_state", {})
        gov_state = migrated.get("gov_state", {})
        ctl_state = migrated.get("ctl_state", {})
        sp_state = migrated.get("sp_mgr_state", {})
        lw_state = migrated.get("lw_state", {})
        db_state = migrated.get("db_state", {})
        cal_state = migrated.get("cal_state", {})
        gs_state = migrated.get("gs_state", {})
        
        # Top-level state
        on_percent = migrated.get("on_percent", 0.0)
        last_target_temp = migrated.get("last_target_temp")
        cycles_since_reset = migrated.get("cycles_since_reset", 0)
        learning_start_date = migrated.get("learning_start_date")
        learning_resume_ts = migrated.get("learning_resume_ts")
        accumulated_dt = migrated.get("accumulated_dt", 0.0)
        legacy_skip = migrated.get("skip_learning_cycles_left", 0)
        
        # Load top-level state
        self._on_percent = float(on_percent or 0.0)
        self._last_target_temp = last_target_temp
        self._cycles_since_reset = int(cycles_since_reset or 0)
        if accumulated_dt is not None:
            self._accumulated_dt = float(accumulated_dt)
        
        # Learning start date
        if learning_start_date:
            try:
                self._learning_start_date = datetime.fromisoformat(learning_start_date)
            except (ValueError, TypeError):
                self._learning_start_date = None
        
        # Learning resume timestamp - handle both wall clock and legacy skip
        if learning_resume_ts is not None:
            self._learning_resume_ts = self._convert_wall_to_monotonic_ts(learning_resume_ts)
        elif legacy_skip and int(legacy_skip) > 0:
            # Legacy key conversion
            duration_min = float(legacy_skip) * max(self._cycle_min, 15.0)
            self._learning_resume_ts = time.monotonic() + (duration_min * 60.0)
        
        # Load component states
        self.est.load_state(est_state)
        self.dt_est.load_state(dt_est_state)
        self.gov.load_state(gov_state)
        self.ctl.load_state(ctl_state)
        
        if hasattr(self.sp_mgr, "load_state"):
            self.sp_mgr.load_state(sp_state)
        
        if hasattr(self.learn_win, "load_state"):
            self.learn_win.load_state(lw_state)
        
        if hasattr(self.deadband_mgr, "load_state"):
            self.deadband_mgr.load_state(db_state)
        
        if hasattr(self.calibration_mgr, "load_state"):
            self.calibration_mgr.load_state(cal_state)
        
        if hasattr(self.gain_scheduler, "load_state"):
            self.gain_scheduler.load_state(gs_state)
        
        # Sync local caches from components for diagnostics
        self._in_deadband = bool(db_state.get("in_deadband", False))
        self._in_near_band = bool(db_state.get("in_near_band", False))
        self._last_calibration_time = cal_state.get("last_calibration_time")
        
        _LOGGER.debug(
            "%s - SmartPI state loaded: a=%.6f, b=%.6f, learns=%d",
            self._name, self.est.a, self.est.b, self.est.learn_ok_count
        )

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
        """
        now = time.monotonic()
        
        # --- 1. Validation ---
        if target_temp is None or current_temp is None:
            _LOGGER.warning("%s - Missing target or current temp, force 0", self._name)
            self._on_percent = 0.0
            return

        if hvac_mode == VThermHvacMode_OFF:
            self.ctl.reset()
            self._on_percent = 0.0
            self._last_u_applied = 0.0
            self._in_deadband = False
            self._in_near_band = False
            self._output_initialized = True
            self._last_calculate_time = None
            return

        # Handle explicit force off (shedding, windows)
        if power_shedding:
             self._on_percent = 0.0
             self._last_u_applied = 0.0
             self.u_prev = 0.0
             # We update regime to PERTURBED but SKIP PID calculation
             self.gov.on_cycle_start()
             self.gov.update_regime(GovernanceRegime.PERTURBED)
             decision, reason = self.gov.decide_update("thermal")
             # Integral is frozen by skip
             self._last_i_mode = f"I:FREEZE({reason.value})"
             self._output_initialized = True
             return

        # Determine dt (time since last calculate)
        dt_min = 0.0
        is_first_run = False
        if self._last_calculate_time is None:
             is_first_run = True
        else:
             dt_min = (now - self._last_calculate_time) / 60.0
        self._last_calculate_time = now

        # Resume from OFF/Shedding/Startup (Only on first run after OFF)
        if is_first_run:
            if self._startup_grace_period:
                # startup/reboot -> NO learning pause
                self._startup_grace_period = False
                self._learning_resume_ts = None
                _LOGGER.debug("%s - Startup/Reboot: No learning pause applied", self._name)
            else:
                # Resume from window/OFF -> Pause learning to let system stabilize
                self._learning_resume_ts = now + (LEARNING_PAUSE_RESUME_MIN * 60.0)
                _LOGGER.debug("%s - Resume from OFF: Learning paused for %d min", self._name, LEARNING_PAUSE_RESUME_MIN)
        
        # Cap dt to avoid huge jumps after pause
        if dt_min > (self._cycle_min * 10):
            dt_min = self._cycle_min

        # --- 2. Setpoint Management ---
        # Filter setpoint
        self._last_raw_setpoint = target_temp
        target_temp_filt = self.sp_mgr.filter_setpoint(
            target_temp, 
            current_temp, 
            hvac_mode, 
            dt_min, 
            advance_ema=True
        )
        self._filtered_setpoint = target_temp_filt
        
        # Check for setpoint change boost
        error = target_temp_filt - current_temp
        if hvac_mode == VThermHvacMode_COOL: error = -error
            
        # Determine setpoint change for this cycle (for rate limit bypass)
        setpoint_changed = (self._prev_setpoint_for_boost is None or 
                            abs(self._prev_setpoint_for_boost - target_temp) > 0.01)
            
        # Call real-time learning (heartbeat)
        if dt_min > 0:
            self.update_learning(
                dt_min=dt_min,
                current_temp=current_temp,
                ext_temp=ext_current_temp,
                u_active=self.u_prev,
                setpoint_changed=setpoint_changed
            )

        self._setpoint_boost_active = self.sp_mgr.update_boost_state(target_temp, error, hvac_mode)

        # FIX 3: Calibration state machine trigger
        # Sync state from CalibrationManager component
        self._calibration_state = self.calibration_mgr.state
        # _force_calibration_requested is now a property delegating to calibration_mgr
        self._calibration_retry_count = self.calibration_mgr.retry_count
        self._last_calibration_time = self.calibration_mgr.last_calibration_time
        self._calibration_start_time = self.calibration_mgr.calibration_start_time
        
        # We enter calibration if:
        # 1. Manual request (force_calibration_requested)
        # 2. OR: Periodic calibration (72h) AND we are in STABLE mode
        # 3. OR: Missing deadtime (unreliable deadtime_heat_reliable or deadtime_cool_reliable)
        # 4. AND: We are not already calibrating
        now_wall = time.time()
        periodic_due = (self._last_calibration_time is not None and
                        (now_wall - self._last_calibration_time) > (FORCE_CALIBRATION_INTERVAL_HOURS * 3600))
        
        # Backward compatibility: accept both governance regime EXCITED_STABLE (new)
        # and phase STABLE (legacy) for periodic calibration trigger.
        # The governance regime may not be updated yet when this check runs,
        # so we fall back to checking the phase property.
        can_start_periodic = (periodic_due and
            (self.gov.regime == GovernanceRegime.EXCITED_STABLE or
             self.phase == SmartPIPhase.STABLE))
        
        # Check for missing deadtime (triggers calibration if either heat or cool is unreliable)
        deadtime_ok = self.dt_est.deadtime_heat_reliable and self.dt_est.deadtime_cool_reliable
        can_start_missing_deadtime = (not deadtime_ok and
            self._calibration_retry_count < CALIBRATION_RETRY_MAX and
            (self.gov.regime == GovernanceRegime.EXCITED_STABLE or
             self.phase == SmartPIPhase.STABLE))
        
        if (self._force_calibration_requested or can_start_periodic or can_start_missing_deadtime) and self._calibration_state == SmartPICalibrationPhase.IDLE:
             is_manual = self._force_calibration_requested
             reason = "manual" if is_manual else ("periodic" if periodic_due else "missing_deadtime")
             _LOGGER.info("%s - Starting forced calibration (reason=%s)",
                          self._name, reason)
             self._calibration_state = SmartPICalibrationPhase.COOL_DOWN
             self._calibration_start_time = now
             # Sync to CalibrationManager component
             self.calibration_mgr._calibration_state = SmartPICalibrationPhase.COOL_DOWN
             self.calibration_mgr._calibration_start_time = now
             self.calibration_mgr._force_calibration_requested = False
             # Increment retry count for auto-triggered calibrations (not manual)
             if is_manual:
                 self._calibration_retry_count = 0  # Reset for manual requests
                 self.calibration_mgr._calibration_retry_count = 0
             else:
                 self._calibration_retry_count += 1
                 self.calibration_mgr._calibration_retry_count += 1

        # Check for calibration timeout
        if (self._calibration_state != SmartPICalibrationPhase.IDLE
            and self._calibration_start_time is not None):
            elapsed = (now - self._calibration_start_time) / 60.0  # in minutes
            if elapsed > CALIBRATION_TIMEOUT_MIN:
                _LOGGER.warning("%s - Calibration timeout after %.1f minutes",
                               self._name, elapsed)
                self._calibration_state = SmartPICalibrationPhase.IDLE
                self._calibration_start_time = None
                # Sync to CalibrationManager component
                self.calibration_mgr._calibration_state = SmartPICalibrationPhase.IDLE
                self.calibration_mgr._calibration_start_time = None

        # If calibrating, execute state machine and EXIT calculate early
        if self._calibration_state != SmartPICalibrationPhase.IDLE:
             self._calculate_forced_calibration(target_temp, current_temp, hvac_mode)
             self._output_initialized = True
             self._last_i_mode = "calibration"
             # Reset last target temp to avoid setpoint change detection on exit
             self._last_target_temp = target_temp
             return

        # 2DOF: Setpoint Weighting
        # Keep this logic here as it feeds into the controller
        if self._setpoint_boost_active or not self._tau_reliable:
            e_p = error
        else:
            e_p = self.setpoint_weight_b * error
            
        self._last_error = error
        self._last_error_p = e_p
        
        # --- 3. Deadband & Nearband State ---
        # Delegate to DeadbandManager component
        tau_info = self.est.tau_reliability()
        self._tau_reliable = tau_info.reliable
        
        db_result = self.deadband_mgr.update(
            error=error,
            hvac_mode=hvac_mode,
            tau_reliable=self._tau_reliable,
            dt_est=self.dt_est,
            estimator=self.est,
            current_temp=current_temp,
            ext_temp=ext_current_temp,
            cycle_min=self.cycle_min,
            deadband_c=self.deadband_c,
        )
        
        # Sync state from component
        in_deadband_now = db_result.in_deadband
        in_near_band_now = db_result.in_near_band
        was_in_deadband = self._in_deadband
        self._in_deadband = in_deadband_now
        self._in_near_band = in_near_band_now
        self._near_band_below_deg = self.deadband_mgr.near_band_below_deg
        self._near_band_above_deg = self.deadband_mgr.near_band_above_deg
        self._near_band_source = self.deadband_mgr.near_band_source

        # Bumpless transfer on deadband exit
        # We manually call controller's bumpless (since we manage deadband state here)
        if was_in_deadband and not in_deadband_now and not setpoint_changed:
            # Re-init integral so output doesn't jump
            # I = (u_prev - u_ff - Kp*ep) / Ki
            # We need u_ff and Kp/Ki current values...
            # We haven't calculated them yet for THIS cycle.
            # We should use previous cycle's values? Or estimate?
            # Original code did this IN THE MIDDLE of calculation.
            # Ideally we recalculate gains/FF first.
            pass # Defer bumpless until after Gain/FF calc
        
        # --- 4. Governance ---
        regime = self.gov.determine_regime(
            self.phase,
            ext_current_temp,
            integrator_hold,
            power_shedding,
            self._output_initialized,
            self._on_percent,
            self._in_deadband,
            self._in_near_band
        )
        self.gov.update_regime(regime)

        gov_decision_g, gov_reason_g = self.gov.decide_update('gains')
        self.gov.decide_update('thermal', self._learning_resume_ts, now) # Update diagnostics
        
        if gov_decision_g in (GovernanceDecision.HARD_FREEZE, GovernanceDecision.FREEZE):
            integrator_hold = True

        # --- 5. Hysteresis Phase ---
        if self.phase == SmartPIPhase.HYSTERESIS:
             out = self.ctl.calculate_hysteresis(
                 target_temp_filt, 
                 current_temp, 
                 hvac_mode,
                 HYST_UPPER_C, 
                 HYST_LOWER_C
             )
             if out is not None:
                 self._on_percent = out
             
             # Updates for Dead Time and Episode status
             self._update_deadtime_episode_status(self._on_percent, hvac_mode, now)
             self.dt_est.update(
                now=now,
                tin=current_temp,
                sp=target_temp_filt,
                u_applied=self._on_percent,
                max_on_percent=self._max_on_percent if self._max_on_percent is not None else 1.0,
                is_hysteresis=True
             )
             return

        # --- 6. Gain Scheduling & FF ---
        # Delegate gain calculation to GainScheduler component
        gain_result = self.gain_scheduler.calculate(
            tau_reliable=self._tau_reliable,
            tau_min=tau_info.tau_min,
            estimator=self.est,
            dt_est=self.dt_est,
            in_near_band=self._in_near_band,
            kp_near_factor=self.kp_near_factor,
            ki_near_factor=self.ki_near_factor,
            governance_decision=gov_decision_g,
        )
        
        # Sync gains from GainResult
        self.Kp = gain_result.kp
        self.Ki = gain_result.ki
        self._kp = gain_result.kp
        self._ki = gain_result.ki
        self._kp_source = gain_result.kp_source
        
        # 6b. Feed Forward
        u_ff = 0.0
        if ext_current_temp is not None:
             if self.est.learn_ok_count_a >= 10 and self._tau_reliable:
                 k_ff = clamp(self.est.b / max(self.est.a, 1e-6), 0.0, 3.0)
                 u_ff = clamp(k_ff * (target_temp_filt - ext_current_temp), 0.0, 1.0)
        
        if hvac_mode == VThermHvacMode_COOL: u_ff = 0.0
        
        # FF Warmup
        learn_scale = clamp(self.est.learn_ok_count / float(self.ff_warmup_ok_count), 0.0, 1.0)
        time_scale = clamp(self._cycles_since_reset / float(self.ff_warmup_cycles), 0.0, 1.0)
        reliable_cap = 1.0 if self._tau_reliable else self.ff_scale_unreliable_max
        u_ff *= clamp(reliable_cap * learn_scale * time_scale, 0.0, 1.0)
        
        # ------------------------------------------------------------------
        # FF gating above setpoint (overshoot protection)
        # If temperature is above setpoint + near_band_above,
        # disable positive feedforward to avoid heating in overshoot.
        # ------------------------------------------------------------------
        if error < -self._near_band_above_deg:
            u_ff = 0.0
            _LOGGER.debug("%s - FF disabled (above setpoint + near band)", self._name)
        
        # --- 7. Bumpless Transfer Application ---
        # Now that we have Kp, Ki, u_ff, we can do the bumpless adjustment if needed
        if self._in_deadband and not in_deadband_now and not setpoint_changed:
             # Calculate required integral to maintain u_prev
             # u_prev = u_ff + Kp*e_p + Ki*I
             # Ki*I = u_prev - u_ff - Kp*e_p
             # I = ...
             if self.Ki > KI_MIN:
                 req_i_val = (self.u_prev - u_ff - self.Kp * e_p) / self.Ki
                 # We simply set the integral in controller (or adjust)
                 # SmartPIController doesn't let us SET integral directly from public API nicely?
                 # It has `integral` attribute.
                 current_i = self.ctl.integral
                 d_i = req_i_val - current_i
                 self.ctl.bumpless_transfer(d_i, self.Ki)
                 _LOGGER.debug("%s - Bumpless transfer applied", self._name)

        # --- Thermal Guard Logic ---
        # Detect steep setpoint decrease in HEAT mode -> Prevent integral windup (negative)
        if hvac_mode == VThermHvacMode_HEAT:
            # Check if setpoint decreased significantly
            if self._last_target_temp is not None and target_temp < self._last_target_temp - 0.01:
                 # If temp is high, activate guard
                 if current_temp > target_temp + DEADBAND_ABOVE_C:
                     self._hysteresis_thermal_guard = True
            
            # Maintenance/Exit
            if self._hysteresis_thermal_guard:
                # Exit when close enough to target
                 if current_temp <= target_temp + DEADBAND_BELOW_C:
                     self._hysteresis_thermal_guard = False

        # --- 8. PID Compute ---
        u_cmd = self.ctl.compute_pwm(
            error,
            e_p,
            self.Kp,
            self.Ki,
            u_ff,
            dt_min,
            self._cycle_min,
            self._in_deadband,
            integrator_hold,
            hvac_mode,
            current_temp,
            target_temp_filt,
            self._hysteresis_thermal_guard,
            self._tau_reliable,
            self.est.learn_ok_count_a
        )

        # --- 9. Soft Constraints ---
        # Rate Limit
        rate_limit = SETPOINT_BOOST_RATE if self._setpoint_boost_active else MAX_STEP_PER_MINUTE
        # Bypass rate limit on first run, setpoint change, or when dt_min is 0
        if setpoint_changed or not self._output_initialized or dt_min <= 0.0:
             u_limited = u_cmd
        else:
             max_step = rate_limit * dt_min
             u_limited = clamp(u_cmd, self.u_prev - max_step, self.u_prev + max_step)
             
        if self._max_on_percent is not None and u_limited > self._max_on_percent:
             u_limited = self._max_on_percent
             
        self._last_u_limited = u_limited
        self._on_percent = u_limited
        
        # --- 10. Timing Constraints & Anti-Windup Tracking ---
        # CycleManager update_timing_constraints handles min on/off delays
        # and updates self._last_forced_by_timing
        u_final = self.update_timing_constraints(self.u_prev, u_limited)
        self._on_percent = u_final
        self._last_u_applied = u_final # For diagnostics
        
        self.ctl.update_anti_windup(
            u_limited,
            u_final,
            dt_min,
            self.Ki,
            self.Kp,
            e_p,
            integrator_hold,
            self._in_deadband,
            self._max_on_percent,
            current_temp,
            target_temp_filt,
            self._hysteresis_thermal_guard
        )
        
        self._output_initialized = True
        
        # Diagnose Sync
        self.integral = self.ctl.integral
        self._last_u_pi = self.ctl.u_pi
        self._last_u_ff = self.ctl.u_ff
        self._last_u_cmd = self.ctl.u_cmd
        self._last_aw_du = self.ctl.last_aw_du
        self._last_i_mode = self.ctl.last_i_mode
        self._last_sat = self.ctl.last_sat
        
        # Dead Time Update
        self._last_target_temp = target_temp
        self._update_deadtime_episode_status(self._on_percent, hvac_mode, now)
        self.dt_est.update(
            now=now,
            tin=current_temp,
            sp=target_temp_filt,
            u_applied=self._on_percent,
            max_on_percent=self._max_on_percent if self._max_on_percent is not None else 1.0,
            is_hysteresis=False
        )
    def _update_deadtime_episode_status(self, u_applied: float, hvac_mode: VThermHvacMode, now: float) -> None:
        """
        Update the start timestamps for heating/cooling episodes.
        Used for in_deadtime_window property.
        """
        # Determine if active based on u > 0 and mode
        is_heating = (u_applied > 0.01) and (hvac_mode == VThermHvacMode_HEAT)
        is_cooling = (u_applied > 0.01) and (hvac_mode == VThermHvacMode_COOL)
        
        # Heat Episode Logic
        if is_heating:
            if self._t_heat_episode_start is None:
                self._t_heat_episode_start = now
                _LOGGER.debug("%s - DeadTime: Heating episode started at %s", self._name, now)
        else:
            if self._t_heat_episode_start is not None:
                _LOGGER.debug("%s - DeadTime: Heating episode stopped", self._name)
            self._t_heat_episode_start = None
            
        # Cool Episode Logic
        if is_cooling:
             if self._t_cool_episode_start is None:
                self._t_cool_episode_start = now
                _LOGGER.debug("%s - DeadTime: Cooling episode started at %s", self._name, now)
        else:
             if self._t_cool_episode_start is not None:
                _LOGGER.debug("%s - DeadTime: Cooling episode stopped", self._name)
             self._t_cool_episode_start = None



    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information (suitable for attributes/UI)."""
        return build_diagnostics(self)
