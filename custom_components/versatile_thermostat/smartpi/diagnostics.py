"""
Smart-PI Diagnostics Module.

This module builds a diagnostics dict by reading internal state of the SmartPI
algorithm. As a "friend" module tightly coupled to SmartPI internals, some
protected member access is expected and intentional.
"""
# pylint: disable=protected-access
from __future__ import annotations

from typing import Any, Dict, TYPE_CHECKING
from .const import (
    SmartPIPhase,
    EPISODE_MIN_DURATION_ON_S,
    EPISODE_MIN_DURATION_OFF_S,
    U_ON_MIN,
)

if TYPE_CHECKING:
    from ..prop_algo_smartpi import SmartPI

def build_diagnostics(algo: SmartPI) -> Dict[str, Any]:
    """Return diagnostic information (suitable for attributes/UI)."""
    tau_info = algo.est.tau_reliability()

    return {
        # Phase / Mode
        "phase": algo.phase,
        "regulation_mode": "hysteresis" if algo.phase == SmartPIPhase.HYSTERESIS else "smartpi",
        "hysteresis_state": algo._hysteresis_state,
        # Model
        "a": round(algo.est.a, 6),
        "b": round(algo.est.b, 6),
        "tau_min": round(tau_info.tau_min, 1),
        "tau_reliable": tau_info.reliable,
        "learn_ok_count": int(algo.est.learn_ok_count),
        "learn_ok_count_a": int(algo.est.learn_ok_count_a),
        "learn_ok_count_b": int(algo.est.learn_ok_count_b),
        "learn_skip_count": int(algo.est.learn_skip_count),
        "learn_last_reason": str(algo.est.learn_last_reason),
        # A1/A2/A3 Diagnostics
        "diag_dTdt_method": algo.est.diag_dTdt_method,
        "diag_b_mad_over_med": round(algo.est.diag_b_mad_over_med, 3) if algo.est.diag_b_mad_over_med is not None else None,
        "diag_a_mad_over_med": round(algo.est.diag_a_mad_over_med, 3) if algo.est.diag_a_mad_over_med is not None else None,
        # Learning metadata
        "learning_start_dt": algo._learning_start_date,
        "learn_progress_percent": (
            round((algo.learn_t_int_s / (EPISODE_MIN_DURATION_ON_S if algo.learn_u_int / max(algo.learn_t_int_s, 1) > U_ON_MIN else EPISODE_MIN_DURATION_OFF_S)) * 100, 1)
            if algo.learn_win_active
            else 0
        ),
        "learn_u_avg": round(algo.learn_u_int / max(algo.learn_t_int_s, 1.0), 3) if algo.learn_win_active else None,
        "learn_time_remaining": (
            round(max(0, (EPISODE_MIN_DURATION_ON_S if algo.learn_u_int / max(algo.learn_t_int_s, 1) > U_ON_MIN else EPISODE_MIN_DURATION_OFF_S) - algo.learn_t_int_s), 0)
            if algo.learn_win_active
            else None
        ),
        # PI
        "Kp": round(algo.Kp, 6),
        "Ki": round(algo.Ki, 6),
        "integral_error": round(algo.integral, 6),
        "i_mode": algo.last_i_mode,
        "sat": algo.last_sat,
        # Errors
        "error": round(algo.error, 4),
        "error_p": round(algo.error_p, 4),
        "error_filtered": round(algo.error_filtered, 4) if algo.error_filtered != 0.0 or algo._e_filt is not None else None,
        # 2DOF/scheduling
        "setpoint_weight_b": round(algo.setpoint_weight_b, 3),
        "near_band_deg": round(algo.near_band_deg, 3),
        "kp_near_factor": round(algo.kp_near_factor, 3),
        "ki_near_factor": round(algo.ki_near_factor, 3),
        "sign_flip_leak": round(algo.sign_flip_leak, 3),
        "sign_flip_active": algo.sign_flip_active,
        # Output
        "u_ff": round(algo.u_ff, 6),
        "ff_raw": round(algo._last_ff_raw, 6),
        "ff_reason": algo._last_ff_reason,
        "ff_scale": round(algo._last_ff_scale, 6) if algo._last_ff_scale is not None else None,
        "ff_H_inertia_s": round(algo._last_ff_H_inertia_s, 1) if algo._last_ff_H_inertia_s is not None else None,
        "ff_d_inertia_deg": round(algo._last_ff_d_inertia_deg, 4) if algo._last_ff_d_inertia_deg is not None else None,
        "u_pi": round(algo.u_pi, 6),
        "ff_warmup_ok_count": int(algo.ff_warmup_ok_count),
        "ff_warmup_cycles": int(algo.ff_warmup_cycles),
        "ff_scale_unreliable_max": round(algo.ff_scale_unreliable_max, 3),
        "cycles_since_reset": int(algo.cycles_since_reset),
        "on_percent": round(algo.on_percent, 6),
        "cycle_min": round(algo.cycle_min, 3),
        # Setpoint filter
        "filtered_setpoint": None if algo.sp_mgr.filtered_setpoint is None else round(algo.sp_mgr.filtered_setpoint, 2),
        # Resume skip
        "learning_resume_ts": int(algo._learning_resume_ts) if algo._learning_resume_ts else None,
        # Anti-windup tracking diagnostics
        "u_cmd": round(algo.u_cmd, 6),
        "u_limited": round(algo.u_limited, 6),
        "u_applied": round(algo.u_applied, 6),
        "aw_du": round(algo.aw_du, 6),
        "forced_by_timing": algo.forced_by_timing,
        # Deadband state
        "in_deadband": algo.in_deadband,
        "in_near_band": algo.in_near_band,
        # Setpoint boost state
        "setpoint_boost_active": algo.sp_mgr.boost_active,
        "hysteresis_thermal_guard": algo._hysteresis_thermal_guard,
        # Dead Time (Smart-PI v2)
        "deadtime_heat_s": algo.dt_est.deadtime_heat_s,
        "deadtime_heat_reliable": algo.dt_est.deadtime_heat_reliable,
        "deadtime_cool_s": algo.dt_est.deadtime_cool_s,
        "deadtime_cool_reliable": algo.dt_est.deadtime_cool_reliable,
        "in_deadtime_window": algo.in_deadtime_window,
        "kp_source": algo._kp_source,
        "deadtime_skip_count_a": algo._deadtime_skip_count_a,
        "deadtime_skip_count_b": algo._deadtime_skip_count_b,
        "deadtime_state": algo.dt_est.state,
        "deadtime_last_power": algo.dt_est.last_power,
        "deadtime_heat_start_time": algo.dt_est.heat_start_time,
        "deadtime_cool_start_time": algo.dt_est.cool_start_time,
        # Near-Band Auto (Phase 2) - delegated to DeadbandManager
        "near_band_below_deg": algo.deadband_mgr.near_band_below_deg,
        "near_band_above_deg": algo.deadband_mgr.near_band_above_deg,
        "near_band_source": algo.deadband_mgr.near_band_source,
        # Safety-First Governance
        "governance_regime": algo.gov._current_regime.value,
        "governance_cycle_regimes": [r.value for r in algo.gov._cycle_regimes],
        # Governance diagnostics
        "last_freeze_reason_thermal": algo.gov.last_freeze_reason_thermal.value,
        "last_freeze_reason_gains": algo.gov.last_freeze_reason_gains.value,
        "last_decision_thermal": algo.gov.last_decision_thermal.value,
        "last_decision_gains": algo.gov.last_decision_gains.value,
        # Setpoint boost aliases
        "boost_active": algo.sp_mgr.boost_active,
    }
