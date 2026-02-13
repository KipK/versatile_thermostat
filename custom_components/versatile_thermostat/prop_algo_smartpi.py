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
import time
from datetime import datetime
from typing import Any, Dict, Optional

from .vtherm_hvac_mode import (
    VThermHvacMode,
    VThermHvacMode_COOL,
    VThermHvacMode_HEAT,
    VThermHvacMode_OFF,
)
from .cycle_manager import CycleManager
from homeassistant.core import HomeAssistant

from .smartpi.const import (
    SmartPIPhase as SmartPIPhase,
    GovernanceRegime as GovernanceRegime,
    GovernanceDecision as GovernanceDecision,
    SmartPICalibrationPhase as SmartPICalibrationPhase,
    KP_SAFE as KP_SAFE,
    KI_SAFE as KI_SAFE,
    KP_MAX as KP_MAX,
    KI_MIN as KI_MIN,
    KI_MAX as KI_MAX,
    KP_MIN as KP_MIN,
    MAX_STEP_PER_MINUTE as MAX_STEP_PER_MINUTE,
    SETPOINT_BOOST_RATE as SETPOINT_BOOST_RATE,
    SETPOINT_BOOST_THRESHOLD as SETPOINT_BOOST_THRESHOLD,
    SETPOINT_BOOST_ERROR_MIN as SETPOINT_BOOST_ERROR_MIN,
    SKIP_CYCLES_AFTER_RESUME as SKIP_CYCLES_AFTER_RESUME,
    LEARNING_PAUSE_RESUME_MIN as LEARNING_PAUSE_RESUME_MIN,
    EPISODE_MIN_DURATION_ON_S as EPISODE_MIN_DURATION_ON_S,
    EPISODE_MIN_DURATION_OFF_S as EPISODE_MIN_DURATION_OFF_S,
    SMARTPI_RECALC_INTERVAL_SEC as SMARTPI_RECALC_INTERVAL_SEC,
    AW_TRACK_TAU_S as AW_TRACK_TAU_S,
    AW_TRACK_MAX_DELTA_I as AW_TRACK_MAX_DELTA_I,
    HYST_UPPER_C as HYST_UPPER_C,
    HYST_LOWER_C as HYST_LOWER_C,
    DEFAULT_DEADBAND_C as DEFAULT_DEADBAND_C,
    DEADBAND_BELOW_C as DEADBAND_BELOW_C,
    DEADBAND_ABOVE_C as DEADBAND_ABOVE_C,
    DEADBAND_HYSTERESIS as DEADBAND_HYSTERESIS,
    AB_HISTORY_SIZE as AB_HISTORY_SIZE,
    AB_MIN_SAMPLES as AB_MIN_SAMPLES,
    DEFAULT_NEAR_BAND_DEG as DEFAULT_NEAR_BAND_DEG,
    DEFAULT_KP_NEAR_FACTOR as DEFAULT_KP_NEAR_FACTOR,
    DEFAULT_KI_NEAR_FACTOR as DEFAULT_KI_NEAR_FACTOR,
    FreezeReason as FreezeReason,
    FORCE_CALIBRATION_INTERVAL_HOURS as FORCE_CALIBRATION_INTERVAL_HOURS,
    CALIBRATION_RETRY_MAX as CALIBRATION_RETRY_MAX,
    CALIBRATION_TIMEOUT_MIN as CALIBRATION_TIMEOUT_MIN,
    clamp as clamp,
)
from .smartpi.learning import DeadTimeEstimator, ABEstimator
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

        # Outputs (duty-cycle only, timing calculated by handler)
        self._on_percent: float = 0.0

        # Diagnostics / status
        self._last_error: float = 0.0
        self._last_error_p: float = 0.0
        self._last_u_ff: float = 0.0
        self._last_u_pi: float = 0.0
        self._tau_reliable: bool = False
        self._sign_flip_active: bool = False

        # Track last time calculate() was executed for dt-based integration
        self._last_calculate_time: Optional[float] = None
        # Accumulated time for cycle counting (used for FF warm-up)
        self._accumulated_dt: float = 0.0

        # Timestamp for robust learning dt calculation
        self._learn_last_ts: float | None = None

        # Track last target temp for learning invalidation
        self._last_target_temp = None

        # Learning start timestamp
        self._learning_start_date: Optional[datetime] = datetime.now()

        # Helper to distinguish Startup (Init) from Resume (OFF->ON)
        # We want to pause learning on Resume, but NOT on Startup/Reboot
        self._startup_grace_period: bool = True

        # Tracking anti-windup diagnostics
        self._last_u_cmd: float = 0.0       # command after [0,1] clamp
        self._last_u_limited: float = 0.0   # after rate-limit and max_on_percent
        self._last_u_applied: float = 0.0   # after timing constraints
        self._last_aw_du: float = 0.0       # tracking delta for diagnostics
        self._last_forced_by_timing: bool = False  # True when timing forced 0%/100%
        self._output_initialized: bool = False # True once calculate() runs successfully

        # --- Dead Time (L) Support ---
        self.dt_est = DeadTimeEstimator()
        self._heat_request_prev: bool = False
        self._t_heat_episode_start: float | None = None
        self._t_cool_episode_start: float | None = None
        self._deadtime_skip_count_a: int = 0
        self._deadtime_skip_count_b: int = 0

        # Feature flag for integral freeze during deadtime (Default OFF)
        self.feature_integral_freeze: bool = False

        # --- Near-Band Auto-Tuning (Phase 2) ---
        # Near-band state is now managed by DeadbandManager component

        # --- Forced Calibration State ---
        # Calibration state is now managed by CalibrationManager component

        # --- Safety-First Governance (Delegated to self.gov) ---
        if saved_state:
            self.load_state(saved_state)

        # Legacy attributes for tests
        self._prev_kp = self.Kp
        self._prev_ki = self.Ki

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
        if self.ctl:
            self.ctl.reset()
        if self.sp_mgr:
            self.sp_mgr.reset()
        if self.gov:
            self.gov.reset()

        self._last_error = 0.0
        self._last_error_p = 0.0
        self._on_percent = 0.0
        self._output_initialized = False
        self._last_u_ff = 0.0
        self._last_u_pi = 0.0
        self._last_u_cmd = 0.0
        self._last_u_limited = 0.0
        self._last_u_applied = 0.0
        self._last_aw_du = 0.0
        self._e_filt = None
        self._cycles_since_reset = 0
        self._accumulated_dt = 0.0
        self._prev_kp = KP_SAFE
        self._prev_ki = KI_SAFE

        # Reset learning states
        self._last_calculate_time = None
        self._learn_last_ts = None
        self._last_target_temp = None
        self._learning_start_date = datetime.now()

        # Learning window state is managed by learn_win component

        # Reset Dead Time Estimator
        self.dt_est.reset()
        self._heat_request_prev = False
        self._t_heat_episode_start = None
        self._deadtime_skip_count_a = 0
        self._deadtime_skip_count_b = 0

        # Reset Phase 2 - near-band state is managed by DeadbandManager component

        # Reset Calibration - calibration state is managed by CalibrationManager component

        # Governance is reset above (self.gov.reset())

        # Reset new component managers (Phase 2.5 refactoring)
        if self.learn_win:
            self.learn_win.reset_all()
        if self.deadband_mgr:
            self.deadband_mgr.reset()
        if self.calibration_mgr:
            self.calibration_mgr.reset()
        if self.gain_scheduler:
            self.gain_scheduler.reset()

        _LOGGER.info("%s - SmartPI learning and history reset", self._name)

    # ------------------------------
    # Property Mappings (Component Redirection)
    # ------------------------------

    @property
    def calibration_state(self) -> SmartPICalibrationPhase:
        return self.calibration_mgr.state

    @property
    def _last_calibration_time(self) -> float | None:
        return self.calibration_mgr.last_calibration_time

    @_last_calibration_time.setter
    def _last_calibration_time(self, value: float | None):
        self.calibration_mgr._last_calibration_time = value

    @property
    def _calibration_retry_count(self) -> int:
        return self.calibration_mgr.retry_count

    @_calibration_retry_count.setter
    def _calibration_retry_count(self, value: int):
        self.calibration_mgr._calibration_retry_count = value

    @property
    def _calibration_start_time(self) -> float | None:
        return self.calibration_mgr.calibration_start_time

    @_calibration_start_time.setter
    def _calibration_start_time(self, value: float | None):
        self.calibration_mgr._calibration_start_time = value

    @property
    def _learning_resume_ts(self) -> float | None:
        return self.learn_win.learning_resume_ts

    @_learning_resume_ts.setter
    def _learning_resume_ts(self, value: float | None):
        self.learn_win.set_learning_resume_ts(value)


    @property
    def _near_band_below_deg(self) -> float:
        return self.deadband_mgr.near_band_below_deg

    @property
    def _near_band_above_deg(self) -> float:
        return self.deadband_mgr.near_band_above_deg

    @property
    def _near_band_source(self) -> str:
        return self.deadband_mgr.near_band_source

    @property
    def Kp(self) -> float:
        return self.gain_scheduler.kp

    @Kp.setter
    def Kp(self, value: float):
        self.gain_scheduler._kp = value

    @property
    def Ki(self) -> float:
        return self.gain_scheduler.ki

    @Ki.setter
    def Ki(self, value: float):
        self.gain_scheduler._ki = value

    @property
    def _kp(self) -> float:
        return self.gain_scheduler.kp

    @_kp.setter
    def _kp(self, value: float):
        self.gain_scheduler._kp = value

    @property
    def _ki(self) -> float:
        return self.gain_scheduler.ki

    @_ki.setter
    def _ki(self, value: float):
        self.gain_scheduler._ki = value

    @property
    def _kp_source(self) -> str:
        return self.gain_scheduler.kp_source

    @_kp_source.setter
    def _kp_source(self, value: str):
        self.gain_scheduler._kp_source = value

    @property
    def _hysteresis_thermal_guard(self) -> bool:
        return self.ctl.hysteresis_thermal_guard

    @_hysteresis_thermal_guard.setter
    def _hysteresis_thermal_guard(self, value: bool):
        self.ctl.hysteresis_thermal_guard = value

    @property
    def _hysteresis_state(self) -> str:
        return self.ctl.hysteresis_state

    @property
    def _last_i_mode(self) -> str:
        return self.ctl.last_i_mode

    @_last_i_mode.setter
    def _last_i_mode(self, value: str):
        self.ctl.last_i_mode = value

    @property
    def _last_sat(self) -> str:
        # Compatibility with legacy code that expects this attribute
        return self.ctl.last_sat

    @_last_sat.setter
    def _last_sat(self, value: str):
        self.ctl.last_sat = value

    @property
    def _in_deadband(self) -> bool:
        return self.deadband_mgr.in_deadband

    @_in_deadband.setter
    def _in_deadband(self, value: bool):
        self.deadband_mgr._in_deadband = value

    @property
    def _in_near_band(self) -> bool:
        return self.deadband_mgr.in_near_band

    @_in_near_band.setter
    def _in_near_band(self, value: bool):
        self.deadband_mgr._in_near_band = value


    @property
    def _current_governance_regime(self) -> str:
        return self.gov.regime.value if hasattr(self.gov.regime, 'value') else str(self.gov.regime)

    @_current_governance_regime.setter
    def _current_governance_regime(self, value: str):
        self.gov._current_regime = GovernanceRegime(value) if isinstance(value, str) else value

    @property
    def phase(self) -> str:
        if self.calibration_mgr.state != SmartPICalibrationPhase.IDLE:
            return SmartPIPhase.CALIBRATION
        if len(self.est.a_meas_hist) < AB_HISTORY_SIZE or len(self.est.b_meas_hist) < AB_HISTORY_SIZE:
            return SmartPIPhase.HYSTERESIS
        return SmartPIPhase.STABLE

    @property
    def meas_count_a(self) -> int:
        return len(self.est.a_meas_hist)

    @property
    def meas_count_b(self) -> int:
        return len(self.est.b_meas_hist)

    @property
    def _cycle_regimes(self):
        return self.gov._cycle_regimes

    # --- Setpoint Manager Redirects ---
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
    def _force_calibration_requested(self) -> bool:
        return self.calibration_mgr.calibration_requested

    @_force_calibration_requested.setter
    def _force_calibration_requested(self, value: bool):
        self.calibration_mgr.calibration_requested = value

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
        self.learn_win.set_learning_resume_ts(time.monotonic() + (duration_min * 60.0))

        # Also reset the learning timestamp to avoid using stale dt
        self._learn_last_ts = None

        # Compute wall-clock time for logging/diagnostics
        try:
            # Just for logging
            resume_dt_log = datetime.now().timestamp() + (duration_min * 60.0)
            resume_dt_iso = datetime.fromtimestamp(resume_dt_log).isoformat()
        except (ValueError, TypeError, OverflowError) as e:
            _LOGGER.warning("Could not format resume time for logging: %s", e)
            resume_dt_iso = "unknown"

        _LOGGER.info("%s - SmartPI notified of resume after interruption, skipping learning until (approx) %s", self._name, resume_dt_iso)

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
        # self.learn_win.learning_resume_ts is the source of truth
        pass
    async def on_cycle_started(self, on_time_sec: float, off_time_sec: float, on_percent: float, hvac_mode: str) -> None:
        """Called when a cycle starts."""
        await super().on_cycle_started(on_time_sec, off_time_sec, on_percent, hvac_mode)
        self._setpoint_changed_in_cycle = False
        # Update internal on_percent to match applied value
        self._on_percent = on_percent
        # Reset governance regime tracking for new cycle
        self.gov.on_cycle_start()

    async def on_cycle_completed(self, new_params: dict, prev_params: dict | None) -> bool:
        """Handle end of cycle (learning). Return False to extend window."""
        await super().on_cycle_completed(new_params, prev_params)

        if prev_params is None:
            # First cycle or check-in, nothing to learn yet
            return True

        # 1. Retrieve Context
        # Note: on_cycle_completed is now mainly used for cycle counting loops.
        # Learning accumulation is done via update_learning() in calculate().
        # MOVED TO update_learning() called by calculate() heartbeat.
        # This method now only handles cycle counting/stats if needed.

        # Cycle accepted -> Count it
        self._cycles_since_reset += 1

        # NOTE: Slope collection for Near-Band is also moved to update_learning
        # or calculate() if needed.

        return True

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
    def u_ff(self) -> float:
        """Last feed-forward value."""
        return self._last_u_ff

    @property
    def u_pi(self) -> float:
        return self._last_u_pi

    @property
    def kp_reel(self) -> float:
        """Return the actual Kp used for calculation (after near-band adjustment)."""
        return self.Kp
    @property
    def ki_reel(self) -> float:
        """Return the actual Ki used for calculation (after near-band adjustment)."""
        return self.Ki

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
    def last_decision_thermal(self) -> str:
        return self.gov.last_decision_thermal.value

    @last_decision_thermal.setter
    def last_decision_thermal(self, value: str | GovernanceDecision):
        if isinstance(value, str):
            try:
                self.gov.last_decision_thermal = GovernanceDecision(value)
            except ValueError:
                pass
        else:
            self.gov.last_decision_thermal = value


    @property
    def freeze_reason_thermal(self) -> str:
        return self.gov.last_freeze_reason_thermal.value

    @property
    def last_decision_gains(self) -> str:
        return self.gov.last_decision_gains.value

    @last_decision_gains.setter
    def last_decision_gains(self, value: str | GovernanceDecision):
        if isinstance(value, str):
            try:
                self.gov.last_decision_gains = GovernanceDecision(value)
            except ValueError:
                pass
        else:
            self.gov.last_decision_gains = value

    @property
    def freeze_reason_gains(self) -> str:
        return self.gov.last_freeze_reason_gains.value


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
    def last_sat(self) -> str:
        return self.ctl.last_sat

    @last_sat.setter
    def last_sat(self, value: str):
        self.ctl.last_sat = value

    @property
    def in_deadband(self) -> bool:
        return self.deadband_mgr.in_deadband

    @in_deadband.setter
    def in_deadband(self, value: bool):
        self.deadband_mgr._in_deadband = value

    @property
    def in_near_band(self) -> bool:
        return self.deadband_mgr.in_near_band

    @in_near_band.setter
    def in_near_band(self, value: bool):
        self.deadband_mgr._in_near_band = value

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
        # Near-band state is now managed by DeadbandManager component

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

        # Calibration state is now fully managed by CalibrationManager

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
            "last_calibration_time": self.calibration_mgr.last_calibration_time,
            "cycles_since_reset": self._cycles_since_reset,
            "learning_start_date": self._learning_start_date.isoformat() if self._learning_start_date else None,
            # Convert monotonic timestamp to wall clock time for persistence
            "learning_resume_ts": self._convert_monotonic_to_wall_ts(self._learning_resume_ts),
            "est_state": self.est.save_state(),
            "dt_est_state": self.dt_est.save_state(),
            "gov_state": self.gov.save_state(),
            "ctl_state": self.ctl.save_state(),
            "sp_mgr_state": self.sp_mgr.save_state() if hasattr(self.sp_mgr, "save_state") else {},
            # Component states
            "lw_state": self.learn_win.save_state() if hasattr(self.learn_win, "save_state") else {},
            "db_state": self.deadband_mgr.save_state() if hasattr(self.deadband_mgr, "save_state") else {},
            "cal_state": self.calibration_mgr.save_state() if hasattr(self.calibration_mgr, "save_state") else {},
            "gs_state": self.gain_scheduler.save_state() if hasattr(self.gain_scheduler, "save_state") else {}
        }
        return state

    def _migrate_old_state_format(self, state: dict) -> dict:
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
            "on_percent": float(state.get("on_percent") or self._on_percent),
            "last_target_temp": state.get("last_target_temp", self._last_target_temp),
            "cycles_since_reset": int(state.get("cycles_since_reset") or self._cycles_since_reset),
            "accumulated_dt": float(state.get("accumulated_dt") or self._accumulated_dt),
            "est_state": {
                k: v for k, v in {
                    "a": state.get("a"),
                    "b": state.get("b"),
                    "learn_ok_count": state.get("learn_ok_count"),
                    "learn_ok_count_a": state.get("learn_ok_count_a"),
                    "learn_ok_count_b": state.get("learn_ok_count_b"),
                    "learn_skip_count": state.get("learn_skip_count"),
                    "a_meas_hist": state.get("a_meas_hist"),
                    "b_meas_hist": state.get("b_meas_hist"),
                    "a_hat_hist": state.get("a_hat_hist"),
                    "b_hat_hist": state.get("b_hat_hist"),
                }.items() if v is not None
            },
            "dt_est_state": {
                k: v for k, v in {
                    "deadtime_heat_s": state.get("deadtime_heat_s"),
                    "deadtime_cool_s": state.get("deadtime_cool_s"),
                    "deadtime_heat_reliable": state.get("deadtime_heat_reliable"),
                    "deadtime_cool_reliable": state.get("deadtime_cool_reliable"),
                    "history_heat": state.get("history_heat"),
                    "history_cool": state.get("history_cool"),
                }.items() if v is not None
            },
            "gov_state": {
                k: v for k, v in {
                    "regime": state.get("regime"),
                    "cycle_regime": state.get("cycle_regime"),
                    "cycle_regimes": state.get("cycle_regimes"),
                }.items() if v is not None
            },
            "ctl_state": {
                k: v for k, v in {
                    "integral": state.get("integral"),
                    "u_prev": state.get("u_prev"),
                    "hysteresis_thermal_guard": state.get("hysteresis_thermal_guard"),
                }.items() if v is not None
            },
            "sp_mgr_state": {
                k: v for k, v in {
                    "last_raw_setpoint": state.get("last_raw_setpoint"),
                    "filtered_setpoint": state.get("filtered_setpoint"),
                    "setpoint_boost_active": state.get("setpoint_boost_active"),
                }.items() if v is not None
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
                "learning_resume_ts": state.get("learning_resume_ts"),
                "learning_start_date": state.get("learning_start_date"),
            },
            "db_state": {
                k: v for k, v in {
                    "in_deadband": state.get("in_deadband"),
                    "in_near_band": state.get("in_near_band"),
                    "near_band_below_deg": state.get("near_band_below_deg"),
                    "near_band_above_deg": state.get("near_band_above_deg"),
                    "near_band_source": state.get("near_band_source"),
                }.items() if v is not None
            },
            "cal_state": {
                k: v for k, v in {
                    "calibration_state": state.get("calibration_state"),
                    "calibration_start_time": state.get("calibration_start_time"),
                    "calibration_retry_count": state.get("calibration_retry_count"),
                    "last_calibration_time": state.get("last_calibration_time"),
                }.items() if v is not None
            },
            "gs_state": {
                k: v for k, v in {
                    "kp": state.get("Kp", state.get("kp")),
                    "ki": state.get("Ki", state.get("ki")),
                    "kp_source": state.get("kp_source"),
                    "ki_source": state.get("ki_source"),
                }.items() if v is not None
            },
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

        # Load component states
        self.est.load_state(migrated.get("est_state", {}))
        self.dt_est.load_state(migrated.get("dt_est_state", {}))
        self.gov.load_state(migrated.get("gov_state", {}))
        self.ctl.load_state(migrated.get("ctl_state", {}))
        self.sp_mgr.load_state(migrated.get("sp_mgr_state", {}))
        self.learn_win.load_state(migrated.get("lw_state", {}))
        self.deadband_mgr.load_state(migrated.get("db_state", {}))
        self.calibration_mgr.load_state(migrated.get("cal_state", {}))
        self.gain_scheduler.load_state(migrated.get("gs_state", {}))

    def _validate_and_handle_off(
        self,
        target_temp: float | None,
        current_temp: float | None,
        hvac_mode: VThermHvacMode,
        power_shedding: bool,
    ) -> bool:
        """Input validation and OFF/Shedding handling.

        Returns:
            True if calculation should STOP (OFF or invalid).
        """
        if target_temp is None or current_temp is None:
            _LOGGER.warning("%s - Missing target or current temp, force 0", self._name)
            self._on_percent = 0.0
            return True

        if hvac_mode == VThermHvacMode_OFF:
            self.ctl.reset()
            self._on_percent = 0.0
            self._last_u_applied = 0.0
            self.deadband_mgr._in_deadband = False
            self.deadband_mgr._in_near_band = False
            self._output_initialized = True
            self._last_calculate_time = None
            return True

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
            return True

        return False

    def _update_time_tracking(self, now: float) -> tuple[float, bool]:
        """Update dt_min and handles first-run logic.

        Returns:
            Tuple of (dt_min, is_first_run).
        """
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
                self.learn_win.set_learning_resume_ts(None)
                _LOGGER.debug("%s - Startup/Reboot: No learning pause applied", self._name)
            else:
                # Resume from window/OFF -> Pause learning to let system stabilize
                self.learn_win.set_learning_resume_ts(now + (LEARNING_PAUSE_RESUME_MIN * 60.0))
                _LOGGER.debug("%s - Resume from OFF: Learning paused for %d min", self._name, LEARNING_PAUSE_RESUME_MIN)

        # Cap dt to avoid huge jumps after pause
        if dt_min > (self._cycle_min * 10):
            dt_min = self._cycle_min

        return dt_min, is_first_run

    def _manage_setpoint(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode: VThermHvacMode,
        dt_min: float
    ) -> tuple[float, bool, float, float | None]:
        """Setpoint filtering and boost logic.

        Returns:
            Tuple of (target_temp_filt, setpoint_changed, error, old_target_temp).
        """
        # Filter setpoint
        self._last_raw_setpoint = target_temp
        target_temp_filt = self.sp_mgr.filter_setpoint(target_temp, current_temp, hvac_mode, dt_min, advance_ema=True)
        self._filtered_setpoint = target_temp_filt

        setpoint_changed = False
        old_target_temp = self._last_target_temp  # Save before update
        if self._last_target_temp is not None:
            if abs(target_temp - self._last_target_temp) > 0.01:
                setpoint_changed = True
                _LOGGER.info(
                    "%s - Target change detected (%.2f -> %.2f), invalidating learning window",
                    self._name, self._last_target_temp, target_temp
                )
        self._last_target_temp = target_temp

        error = target_temp_filt - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            error = -error

        self._setpoint_boost_active = self.sp_mgr.update_boost_state(target_temp, error, hvac_mode)

        return target_temp_filt, setpoint_changed, error, old_target_temp

    def _update_control_context(
        self,
        error: float,
        hvac_mode: VThermHvacMode,
        current_temp: float,
        ext_current_temp: float | None,
    ) -> tuple[float, bool]:
        """Update tau reliability, error weighting, and deadband state.

        Returns:
            Tuple of (e_p, was_in_deadband).
        """
        tau_info = self.est.tau_reliability()
        self._tau_reliable = tau_info.reliable

        if self._setpoint_boost_active or not self._tau_reliable:
            e_p = error
        else:
            e_p = self.setpoint_weight_b * error

        self._last_error = error
        self._last_error_p = e_p

        # Deadband update
        was_in_deadband = self.deadband_mgr.in_deadband
        self.deadband_mgr.update(
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

        return e_p, was_in_deadband

    def _apply_gains_and_ff(
        self,
        gov_decision_g: GovernanceDecision,
        target_temp_filt: float,
        ext_current_temp: float | None,
        hvac_mode: VThermHvacMode,
        error: float,
    ) -> tuple[float, bool]:
        """Calculate gains and feedforward, and handles integrator hold.

        Returns:
            Tuple of (u_ff, integrator_hold).
        """
        # Delegate gain calculation to GainScheduler component
        tau_info = self.est.tau_reliability()
        self.gain_scheduler.calculate(
            tau_reliable=self._tau_reliable,
            tau_min=tau_info.tau_min,
            estimator=self.est,
            dt_est=self.dt_est,
            in_near_band=self.deadband_mgr.in_near_band,
            kp_near_factor=self.kp_near_factor,
            ki_near_factor=self.ki_near_factor,
            governance_decision=gov_decision_g,
        )

        # Gains updated within GainScheduler component

        # Feed Forward
        u_ff = 0.0
        if ext_current_temp is not None:
            if self.est.learn_ok_count_a >= 10 and self._tau_reliable:
                k_ff = clamp(self.est.b / max(self.est.a, 1e-6), 0.0, 3.0)
                u_ff = clamp(k_ff * (target_temp_filt - ext_current_temp), 0.0, 1.0)

        if hvac_mode == VThermHvacMode_COOL:
            u_ff = 0.0

        # FF Warmup
        learn_scale = clamp(self.est.learn_ok_count / float(self.ff_warmup_ok_count), 0.0, 1.0)
        time_scale = clamp(self._cycles_since_reset / float(self.ff_warmup_cycles), 0.0, 1.0)
        reliable_cap = 1.0 if self._tau_reliable else self.ff_scale_unreliable_max
        u_ff *= clamp(reliable_cap * learn_scale * time_scale, 0.0, 1.0)

        # FF gating above setpoint (overshoot protection)
        if error < 0:
            u_ff = 0.0
            _LOGGER.debug("%s - FF disabled (above setpoint)", self._name)

        integrator_hold = gov_decision_g in (GovernanceDecision.HARD_FREEZE, GovernanceDecision.FREEZE)

        return u_ff, integrator_hold

    def _apply_soft_constraints(
        self,
        u_cmd: float,
        dt_min: float,
        setpoint_changed: bool
    ) -> float:
        """Apply rate limiting and clamping to the output.

        Returns:
            The limited output.
        """
        # Rate Limit
        rate_limit = SETPOINT_BOOST_RATE if self._setpoint_boost_active else MAX_STEP_PER_MINUTE
        # Bypass rate limit on first run, setpoint change, or when dt_min is 0
        if setpoint_changed or not self._output_initialized or dt_min <= 0.0:
            u_limited = u_cmd
        else:
            max_step = rate_limit * dt_min
            u_limited = clamp(u_cmd, self.u_prev - max_step, self.u_prev + max_step)

        # SATURATION & FINAL OUTPUT
        self._on_percent = clamp(u_limited, 0.0, self._max_on_percent if self._max_on_percent is not None else 1.0)
        return self._on_percent

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None = None,
        hvac_mode: VThermHvacMode | None = None,
        slope: float | None = None,
        integrator_hold: bool = False,
        power_shedding: bool = False,
        *args,
        **kwargs,
    ) -> float:
        """
        Compute the next duty-cycle command.
        """
        # Compatibility handling for old signature: calculate(t, c, dt_min, now, hvac_mode)
        # In old calls: ext_current_temp=dt_min, hvac_mode=now, slope=hvac_mode
        if hvac_mode is not None and not isinstance(hvac_mode, VThermHvacMode) and isinstance(slope, VThermHvacMode):
            # old call detected
            hvac_mode = slope
            slope = None
        elif hvac_mode is None and len(args) > 0 and isinstance(args[0], VThermHvacMode):
            # another old call variant
            hvac_mode = args[0]

        now = time.monotonic()

        # --- 1. Validation & Handle OFF ---
        if self._validate_and_handle_off(target_temp, current_temp, hvac_mode, power_shedding):
            return

        # --- 2. Update Time Tracking ---
        dt_min, is_first_run = self._update_time_tracking(now)

        # --- 3. Setpoint Management ---
        target_temp_filt, setpoint_changed, error, old_target_temp = self._manage_setpoint(
            target_temp, current_temp, hvac_mode, dt_min
        )

        # Clear cycle regimes on setpoint change to prevent REGIME_TRANSITION freeze
        if setpoint_changed:
            self.gov.on_cycle_start()
            # Handle integral reset and thermal guard on setpoint changes
            new_error, new_error_p = self.ctl.handle_setpoint_change(
                target_temp, old_target_temp, current_temp, hvac_mode, self.Kp, self.Ki
            )
            if new_error != 0.0:
                self._last_error = new_error
                self._last_error_p = new_error_p

        # --- 4. Learning & Calibration ---
        # Heartbeat learning update
        if dt_min > 0:
            self.update_learning(
                dt_min=dt_min,
                current_temp=current_temp,
                ext_temp=ext_current_temp,
                u_active=self.u_prev,
                setpoint_changed=setpoint_changed
            )

        # Calibration state machine
        if self.calibration_mgr.is_calibrating and self.calibration_mgr.calibration_start_time is not None:
            elapsed = (now - self.calibration_mgr.calibration_start_time) / 60.0
            if elapsed > CALIBRATION_TIMEOUT_MIN:
                _LOGGER.warning("%s - Calibration timeout after %.1f minutes", self._name, elapsed)
                self.calibration_mgr.handle_timeout()

        deadtime_ok = self.dt_est.deadtime_heat_reliable and self.dt_est.deadtime_cool_reliable
        self.calibration_mgr.check_and_start(
            now=now,
            now_wall=time.time(),
            governance_regime=self.gov.regime,
            phase=self.phase,
            deadtime_reliable=deadtime_ok,
            force_calibration_interval_hours=FORCE_CALIBRATION_INTERVAL_HOURS,
            calibration_retry_max=CALIBRATION_RETRY_MAX,
        )

        if self.calibration_mgr.is_calibrating:
            self._calculate_forced_calibration(target_temp, current_temp, hvac_mode)
            self._output_initialized = True
            self._last_i_mode = "calibration"
            self._last_target_temp = target_temp
            return

        # --- 5. Hysteresis Phase ---
        if self.phase == SmartPIPhase.HYSTERESIS:
            out = self.ctl.calculate_hysteresis(target_temp_filt, current_temp, hvac_mode, HYST_UPPER_C, HYST_LOWER_C)
            if out is not None:
                self._on_percent = out

            self._update_deadtime_episode_status(self._on_percent, hvac_mode, now)
            self.dt_est.update(
                now=now,
                tin=current_temp,
                sp=target_temp_filt,
                u_applied=self._on_percent,
                max_on_percent=self._max_on_percent if self._max_on_percent is not None else 1.0,
                is_hysteresis=True,
            )
            return

        # --- 6. Control Context & Deadband ---
        e_p, was_in_deadband = self._update_control_context(
            error, hvac_mode, current_temp, ext_current_temp
        )
        in_deadband_now = self.deadband_mgr.in_deadband

        # --- 7. Governance Decision ---
        regime = self.gov.determine_regime(
            self.phase,
            ext_current_temp,
            integrator_hold,
            power_shedding,
            self._output_initialized,
            self._on_percent,
            self.deadband_mgr.in_deadband,
            self.deadband_mgr.in_near_band
        )
        self.gov.update_regime(regime)
        gov_decision_g, gov_reason_g = self.gov.decide_update('gains')
        gov_decision_t, gov_reason_t = self.gov.decide_update('thermal', self.learn_win.learning_resume_ts, now)

        # --- 8. Gains & FF ---
        u_ff, gov_hold = self._apply_gains_and_ff(
            gov_decision_g, target_temp_filt, ext_current_temp, hvac_mode, error
        )
        # Apply explicit hold (parameter) or governance hold
        integrator_hold = integrator_hold or gov_hold

        # --- 9. Bumpless Transfer ---
        if was_in_deadband and not in_deadband_now and not setpoint_changed:
            if self.Ki > KI_MIN:
                req_i_val = (self.u_prev - u_ff - self.Kp * e_p) / self.Ki
                current_i = self.ctl.integral
                self.ctl.bumpless_transfer(req_i_val - current_i, self.Ki)
                _LOGGER.debug("%s - Bumpless transfer applied", self._name)

        # --- 10. Thermal Guard ---
        if hvac_mode == VThermHvacMode_HEAT:
            if self._last_target_temp is not None and target_temp < self._last_target_temp - 0.01:
                if current_temp > target_temp + DEADBAND_ABOVE_C:
                    self._hysteresis_thermal_guard = True
            if self._hysteresis_thermal_guard:
                if current_temp <= target_temp + DEADBAND_BELOW_C:
                    self._hysteresis_thermal_guard = False

        # --- 11. PID Compute ---
        u_cmd = self.ctl.compute_pwm(
            error,
            e_p,
            self.Kp,
            self.Ki,
            u_ff,
            dt_min,
            self._cycle_min,
            in_deadband_now,
            integrator_hold,
            hvac_mode,
            current_temp,
            target_temp_filt,
            self._hysteresis_thermal_guard,
            self._tau_reliable,
            self.est.learn_ok_count_a
        )

        # --- 12. Soft Constraints ---
        u_limited = self._apply_soft_constraints(u_cmd, dt_min, setpoint_changed)
        self._last_u_limited = u_limited

        # --- 13. Timing Constraints & Anti-Windup Tracking ---
        u_final = self.update_timing_constraints(self.u_prev, u_limited)
        self._on_percent = u_final
        self._last_u_applied = u_final

        self.ctl.update_anti_windup(
            u_limited,
            u_final,
            dt_min,
            self.Ki,
            self.Kp,
            e_p,
            integrator_hold,
            in_deadband_now,
            self._max_on_percent,
            current_temp,
            target_temp_filt,
            self._hysteresis_thermal_guard
        )

        # --- 14. Update State & Diagnostics ---
        self._output_initialized = True
        # Note: self.integral and self.u_prev are already updated in ctl and managed via properties
        self._last_u_pi = self.ctl.u_pi
        self._last_u_ff = self.ctl.u_ff
        self._last_u_cmd = self.ctl.u_cmd
        self._last_aw_du = self.ctl.last_aw_du
        # self._last_i_mode and self._last_sat are now properties delegating to self.ctl

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
        self.u_prev = self._on_percent
        self._cycles_since_reset += 1

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
