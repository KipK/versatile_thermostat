"""
Learning Window Manager for Smart-PI.

Manages multi-cycle learning window state and accumulation for the a/b estimator.
"""
from __future__ import annotations

import logging
from datetime import datetime
from typing import TYPE_CHECKING, Optional

from .const import (
    DELTA_MIN,
    DT_MAX_MIN,
    EPISODE_MIN_DURATION_OFF_S,
    EPISODE_MIN_DURATION_ON_S,
    MIN_ABS_DT,
    U_OFF_MAX,
    U_ON_MIN,
    clamp,
)

if TYPE_CHECKING:
    from .learning import ABEstimator, DeadTimeEstimator
    from .governance import SmartPIGovernance

_LOGGER = logging.getLogger(__name__)


class LearningWindowManager:
    """
    Manages multi-cycle learning window state and accumulation.
    
    The learning window accumulates temperature and power data over multiple
    cycles to provide robust estimates for the a/b thermal model parameters.
    """

    def __init__(self, name: str):
        """Initialize the learning window manager.
        
        Args:
            name: Entity name for logging purposes.
        """
        self._name = name
        
        # Learning window state (multi-cycle learning)
        self._active: bool = False
        self._start_ts: float | None = None
        self._T_int_start: float = 0.0
        self._T_ext_start: float = 0.0
        self._u_int: float = 0.0
        self._t_int_s: float = 0.0
        self._u_first: float | None = None
        
        # Learning start timestamp
        self._learning_start_date: Optional[datetime] = datetime.now()
        
        # Skip learning cycles after resume from interruption
        self._learning_resume_ts: Optional[float] = None

    # --------------------------------------------------------------------------
    # Properties for diagnostic access
    # --------------------------------------------------------------------------
    
    @property
    def active(self) -> bool:
        """Return True if a learning window is currently active."""
        return self._active

    @property
    def start_ts(self) -> float | None:
        """Return the monotonic timestamp when the current window started."""
        return self._start_ts

    @property
    def T_int_start(self) -> float:
        """Return the indoor temperature at window start."""
        return self._T_int_start

    @property
    def T_ext_start(self) -> float:
        """Return the outdoor temperature at window start."""
        return self._T_ext_start

    @property
    def u_int(self) -> float:
        """Return the accumulated power integral (u * dt)."""
        return self._u_int

    @property
    def t_int_s(self) -> float:
        """Return the accumulated time integral in seconds."""
        return self._t_int_s

    @property
    def u_first(self) -> float | None:
        """Return the first power value in the window (for consistency check)."""
        return self._u_first

    @property
    def learning_start_date(self) -> Optional[datetime]:
        """Return the wall-clock date when learning started."""
        return self._learning_start_date

    @property
    def learning_resume_ts(self) -> Optional[float]:
        """Return the monotonic timestamp for learning resume cooldown."""
        return self._learning_resume_ts

    # --------------------------------------------------------------------------
    # State management methods
    # --------------------------------------------------------------------------

    def reset(self) -> None:
        """Reset the multi-cycle learning window state."""
        self._active = False
        self._start_ts = None
        self._u_int = 0.0
        self._t_int_s = 0.0
        self._u_first = None

    def reset_all(self) -> None:
        """Reset all learning window state including timestamps."""
        self.reset()
        self._T_int_start = 0.0
        self._T_ext_start = 0.0
        self._learning_start_date = datetime.now()
        self._learning_resume_ts = None

    def set_learning_resume_ts(self, ts: Optional[float]) -> None:
        """Set the learning resume timestamp for cooldown after interruption.
        
        Args:
            ts: Monotonic timestamp until which learning should be paused.
        """
        self._learning_resume_ts = ts

    # --------------------------------------------------------------------------
    # Persistence methods
    # --------------------------------------------------------------------------

    def load_state(self, state: dict) -> None:
        """Load state from persistence dict.
        
        Args:
            state: Dictionary containing persisted state.
        """
        if not state:
            return

        self._active = state.get("learn_win_active", False)
        self._start_ts = state.get("learn_win_start_ts")
        self._T_int_start = state.get("learn_T_int_start", 0.0)
        self._T_ext_start = state.get("learn_T_ext_start", 0.0)
        self._u_int = state.get("learn_u_int", 0.0)
        self._t_int_s = state.get("learn_t_int_s", 0.0)
        self._u_first = state.get("learn_u_first")
        
        # Parse learning start date from ISO string
        start_date_str = state.get("learning_start_date")
        if start_date_str:
            try:
                self._learning_start_date = datetime.fromisoformat(start_date_str)
            except (ValueError, TypeError):
                self._learning_start_date = datetime.now()
        else:
            self._learning_start_date = datetime.now()
            
        self._learning_resume_ts = state.get("learning_resume_ts")

    def save_state(self) -> dict:
        """Save state to persistence dict.
        
        Returns:
            Dictionary containing the current state.
        """
        return {
            "learn_win_active": self._active,
            "learn_win_start_ts": self._start_ts,
            "learn_T_int_start": self._T_int_start,
            "learn_T_ext_start": self._T_ext_start,
            "learn_u_int": self._u_int,
            "learn_t_int_s": self._t_int_s,
            "learn_u_first": self._u_first,
            "learning_start_date": (
                self._learning_start_date.isoformat() 
                if self._learning_start_date else None
            ),
            "learning_resume_ts": self._learning_resume_ts,
        }

    # --------------------------------------------------------------------------
    # Learning window update method
    # --------------------------------------------------------------------------

    def update(
        self,
        dt_min: float,
        current_temp: float,
        ext_temp: float,
        u_active: float,
        setpoint_changed: bool,
        estimator: "ABEstimator",
        dt_est: "DeadTimeEstimator",
        governance: "SmartPIGovernance",
        learning_resume_ts: float | None,
        now: float,
        in_deadband: bool,
        in_near_band: bool,
        t_heat_episode_start: float | None,
        t_cool_episode_start: float | None,
        deadtime_skip_count_a: int = 0,
        deadtime_skip_count_b: int = 0,
        is_calibrating: bool = False,
    ) -> tuple[int, int]:
        """
        Update learning window and submit to estimator if conditions met.
        
        Args:
            dt_min: Elapsed time in minutes since last update.
            current_temp: Current indoor temperature.
            ext_temp: Current outdoor temperature.
            u_active: Power applied during this interval (0..1).
            setpoint_changed: True if setpoint changed during this interval.
            estimator: The ABEstimator instance for learning submission.
            dt_est: The DeadTimeEstimator instance for deadtime checks.
            governance: The SmartPIGovernance instance for regime decisions.
            learning_resume_ts: Timestamp for learning resume cooldown.
            now: Current monotonic time.
            in_deadband: Whether the system is in deadband.
            in_near_band: Whether the system is in near-band.
            t_heat_episode_start: Monotonic timestamp of heating episode start.
            t_cool_episode_start: Monotonic timestamp of cooling episode start.
            deadtime_skip_count_a: Counter for heating deadtime skips (returned).
            deadtime_skip_count_b: Counter for cooling deadtime skips (returned).
            
        Returns:
            Tuple of (deadtime_skip_count_a, deadtime_skip_count_b) for tracking.
        """
        # Import here to avoid circular imports at module level
        from .governance import GovernanceDecision
        from .learning import ABEstimator
        
        if dt_min <= 0:
            return deadtime_skip_count_a, deadtime_skip_count_b

        dt_s = dt_min * 60.0

        # Update learning resume timestamp if provided
        if learning_resume_ts is not None:
            self._learning_resume_ts = learning_resume_ts

        # --- Setpoint change aborts learning ---
        if setpoint_changed:
            _LOGGER.debug(
                "%s - update_learning: aborting due to setpoint change", 
                self._name
            )
            estimator.learn_skip_count += 1
            estimator.learn_last_reason = "skip: setpoint change"
            self.reset()
            return deadtime_skip_count_a, deadtime_skip_count_b

        # --- Governance gate (thermal domain: a/b learning) ---
        # During calibration, bypass governance to allow A/B learning
        # (the system traverses deadband/nearband at 100%/0%, governance
        # restrictions are not relevant).
        if not is_calibrating:
            gov_decision, gov_reason = governance.decide_update(
                'thermal', self._learning_resume_ts, now
            )
            if gov_decision in (GovernanceDecision.HARD_FREEZE, GovernanceDecision.FREEZE):
                estimator.learn_skip_count += 1
                estimator.learn_last_reason = f"skip: governance ({gov_reason.value})"
                if self._active:
                    self.reset()
                return deadtime_skip_count_a, deadtime_skip_count_b

        # --- Interruption / Resume Check ---
        if self._learning_resume_ts:
            if now < self._learning_resume_ts:
                estimator.learn_skip_count += 1
                estimator.learn_last_reason = "skip: resume cool-down"
                self.reset()
                return deadtime_skip_count_a, deadtime_skip_count_b
            else:
                self._learning_resume_ts = None

        # --- Validation: external temperature required ---
        if ext_temp is None:
            estimator.learn_skip_count += 1
            estimator.learn_last_reason = "skip: no external temp"
            self.reset()
            return deadtime_skip_count_a, deadtime_skip_count_b

        # --- Dead Time Gating ---
        # If in deadband or near-band (Stable PI), ignore deadtime skipping
        # because we are "safely landed" and small fluctuations should be 
        # treated as normal learning/skipping rather than blocking.
        ignore_deadtime_skip = in_deadband or in_near_band

        # Heating deadtime check
        if (
            not ignore_deadtime_skip 
            and dt_est.deadtime_heat_reliable 
            and t_heat_episode_start is not None 
            and dt_est.deadtime_heat_s is not None
        ):
            elapsed_episode = now - t_heat_episode_start
            if elapsed_episode < dt_est.deadtime_heat_s:
                estimator.learn_skip_count += 1
                estimator.learn_last_reason = "skip: deadtime window"
                deadtime_skip_count_a += 1
                if self._active:
                    self.reset()
                return deadtime_skip_count_a, deadtime_skip_count_b

        # Cooling deadtime check
        if (
            not ignore_deadtime_skip 
            and dt_est.deadtime_cool_reliable 
            and t_cool_episode_start is not None 
            and dt_est.deadtime_cool_s is not None
        ):
            elapsed_episode = now - t_cool_episode_start
            if elapsed_episode < dt_est.deadtime_cool_s:
                estimator.learn_skip_count += 1
                estimator.learn_last_reason = "skip: deadtime window (cool)"
                deadtime_skip_count_b += 1
                if self._active:
                    self.reset()
                return deadtime_skip_count_a, deadtime_skip_count_b

        # --- Learning Window Accumulation ---
        early_submit = False
        if not self._active:
            # Before starting window, check if backdated start would be in deadtime
            proposed_start_ts = now - dt_s

            # Check heating deadtime overlap
            if (
                not ignore_deadtime_skip 
                and dt_est.deadtime_heat_reliable 
                and t_heat_episode_start is not None 
                and dt_est.deadtime_heat_s is not None
            ):
                deadtime_end_ts = t_heat_episode_start + dt_est.deadtime_heat_s
                if proposed_start_ts < deadtime_end_ts:
                    estimator.learn_skip_count += 1
                    estimator.learn_last_reason = "skip: window would start in deadtime"
                    return deadtime_skip_count_a, deadtime_skip_count_b

            # Check cooling deadtime overlap
            if (
                not ignore_deadtime_skip 
                and dt_est.deadtime_cool_reliable 
                and t_cool_episode_start is not None 
                and dt_est.deadtime_cool_s is not None
            ):
                deadtime_end_ts = t_cool_episode_start + dt_est.deadtime_cool_s
                if proposed_start_ts < deadtime_end_ts:
                    estimator.learn_skip_count += 1
                    estimator.learn_last_reason = "skip: window would start in deadtime (cool)"
                    return deadtime_skip_count_a, deadtime_skip_count_b

            # OK to start window
            self._active = True
            self._start_ts = proposed_start_ts
            self._T_int_start = current_temp
            self._T_ext_start = ext_temp
            self._u_int = 0.0
            self._t_int_s = 0.0
            self._u_first = u_active
            estimator.learn_last_reason = "learn: window start"
        else:
            # Check power consistency
            if (
                self._u_first is not None
                and abs(u_active - self._u_first) > 1e-3
            ):
                # Power transition detected (e.g. hysteresis phase boundary).
                # Attempt early submission with accumulated data instead of
                # discarding the window.
                dT_early = current_temp - self._T_int_start
                abs_dT_early = abs(dT_early)
                delta_T_early = abs(self._T_int_start - self._T_ext_start)
                if abs_dT_early >= MIN_ABS_DT and delta_T_early >= DELTA_MIN:
                    _LOGGER.debug(
                        "%s - power transition: early submit (%.0fs, dT=%.3f)",
                        self._name, self._t_int_s, dT_early,
                    )
                    early_submit = True
                else:
                    estimator.learn_skip_count += 1
                    estimator.learn_last_reason = "skip: power instability"
                    self.reset()
                    return deadtime_skip_count_a, deadtime_skip_count_b
            else:
                early_submit = False

        if early_submit:
            # Use already-accumulated data (do not add the transition tick)
            window_dt_min = self._t_int_s / 60.0
            dT = current_temp - self._T_int_start
            abs_dT = abs(dT)
            delta_T = self._T_int_start - self._T_ext_start
        else:
            # Accumulate
            self._u_int += clamp(u_active, 0.0, 1.0) * dt_s
            self._t_int_s += dt_s

            # Current Window Stats
            window_dt_min = self._t_int_s / 60.0

            dT = current_temp - self._T_int_start
            abs_dT = abs(dT)
            delta_T = self._T_int_start - self._T_ext_start

            # Calculate preliminary u_eff for duration check
            if self._t_int_s > 0.0:
                u_eff_pre = self._u_int / self._t_int_s
            else:
                u_eff_pre = 0.0

            # Determine min duration based on power state
            if u_eff_pre > U_ON_MIN:
                min_dur_s = EPISODE_MIN_DURATION_ON_S
            elif u_eff_pre < U_OFF_MAX:
                min_dur_s = EPISODE_MIN_DURATION_OFF_S
            else:
                min_dur_s = EPISODE_MIN_DURATION_ON_S

            # --- Extension Checks ---
            if abs(delta_T) < DELTA_MIN:
                self.reset()
                estimator.learn_last_reason = "skip: delta too small"
                return deadtime_skip_count_a, deadtime_skip_count_b

            # Extend if duration not met or dT too small (and not timed out)
            duration_ok = self._t_int_s >= min_dur_s
            amplitude_ok = abs_dT >= MIN_ABS_DT

            if (not duration_ok or not amplitude_ok) and window_dt_min < DT_MAX_MIN:
                reason = []
                if not duration_ok:
                    reason.append(f"dur {self._t_int_s:.0f}/{min_dur_s}s")
                if not amplitude_ok:
                    reason.append(f"dT {abs_dT:.2f}/{MIN_ABS_DT}")
                estimator.learn_last_reason = f"skip: extending ({', '.join(reason)})"
                return deadtime_skip_count_a, deadtime_skip_count_b  # Extend window

            # Timeout Logic
            if window_dt_min >= DT_MAX_MIN:
                if not amplitude_ok:
                    self.reset()
                    estimator.learn_last_reason = "skip: window timeout (dT too small)"
                    return deadtime_skip_count_a, deadtime_skip_count_b
                # If amplitude OK but duration short (shouldn't happen), proceed

        if self._t_int_s <= 0.0:
            self.reset()
            estimator.learn_last_reason = "skip: window duty invalid"
            return deadtime_skip_count_a, deadtime_skip_count_b

        # --- Learning Submission ---
        u_eff = self._u_int / self._t_int_s
        dT_dt = dT / window_dt_min

        if u_eff < U_OFF_MAX:
            # OFF Learning
            relevant_samples = [
                p for p in dt_est.tin_history 
                if p[0] >= self._start_ts
            ]

            slope_val, method, _ = ABEstimator.robust_dTdt_per_min(
                relevant_samples,
                trim_start_frac=0.10,
                trim_end_frac=0.10,
            )

            if slope_val is not None:
                final_slope = slope_val
                estimator.diag_dTdt_method = method
            else:
                estimator.learn_skip_count += 1
                estimator.learn_last_reason = f"skip: OFF slope not robust ({method})"
                self.reset()
                return deadtime_skip_count_a, deadtime_skip_count_b

            estimator.learn(
                dT_int_per_min=final_slope,
                u=0.0,
                t_int=self._T_int_start,
                t_ext=self._T_ext_start,
            )
        elif u_eff > U_ON_MIN:
            # ON phase
            relevant_samples = [
                p for p in dt_est.tin_history 
                if p[0] >= self._start_ts
            ]

            slope_val, method, _ = ABEstimator.robust_dTdt_per_min(relevant_samples)
            if slope_val is not None:
                final_slope = slope_val
                estimator.diag_dTdt_method = method
            else:
                final_slope = dT_dt
                estimator.diag_dTdt_method = "fallback_simple"

            estimator.learn(
                dT_int_per_min=final_slope,
                u=u_eff,
                t_int=self._T_int_start,
                t_ext=self._T_ext_start,
            )
        else:
            estimator.learn_last_reason = "skip: low excitation (u mid)"

        self.reset()
        return deadtime_skip_count_a, deadtime_skip_count_b
