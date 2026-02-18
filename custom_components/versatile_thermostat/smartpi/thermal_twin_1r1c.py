"""
Thermal Twin 1R1C — Digital twin based on (a, b) model + deadtime.
Spec v4.1 compliant.

Diagnostics-only: does NOT modify the control command.

Model: dT/dt = a·u(t-L) - b·(T - Text) + d(t)
  - a: heating efficacy (°C·min⁻¹ per unit of u)
  - b: loss coefficient (min⁻¹), tau = 1/b
  - L: dead time (seconds)
  - d(t): external perturbation (solar, occupants, ...), estimated online

Architecture v4 — two distinct predictions:
  - T_pred_pure (pure model, no d_hat): drives T_hat state and d_raw estimation
  - T_pred (corrected model, includes d_hat_ema): exposed diagnostics, RMSE, CUSUM

v4.1 fixes:
  - CUSUM reset after detection (no simultaneous gain+loss)
  - d_hat_ema physical clamp (|d_hat| ≤ d_hat_max)
  - d_hat slow relaxation toward 0 when u < u_min (avoids stale solar bias)
  - rmse_pure exposed (uncompensated model quality indicator)
  - u_buffer without maxlen (fixes infinite loop on deadtime increase)
  - Input validation on dt_s, gamma
  - Buffer normalization after load_state()

Discretisation: exact exponential (ZOH, unconditionally stable).
Observer: simplified Luenberger (post-prediction nudging on pure model).
"""

from __future__ import annotations

import logging
from collections import deque
from math import exp, isfinite, log, sqrt

_LOGGER = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _clamp(v: float, lo: float, hi: float) -> float:
    """Clamp v between lo and hi."""
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _resolve_text(
    text_meas: float | None,
    policy: str,
    last_text: float | None,
) -> float | None:
    """Resolve Text according to policy. Returns None if unavailable."""
    if text_meas is not None:
        return float(text_meas)
    if policy == "hold_last" and last_text is not None:
        return last_text
    return None


# ---------------------------------------------------------------------------
# Physical bounds for T_steady guard (§15.4.2)
# ---------------------------------------------------------------------------
T_STEADY_MIN = -50.0
T_STEADY_MAX = 80.0


# ---------------------------------------------------------------------------
# ThermalTwin1R1C
# ---------------------------------------------------------------------------


class ThermalTwin1R1C:
    """Digital twin 1R1C with dead-time buffer and Luenberger nudging.

    Spec v4.1 compliant — dual prediction architecture:
      - Pure model for state estimation (T_hat) and d_raw extraction
      - Corrected model (with d_hat_ema) for exposed diagnostics

    The u_buffer has NO maxlen — size is controlled exclusively by code
    in step() and _resize_buffer(). This prevents the infinite loop that
    would occur if maxlen blocked growth when deadtime increases.
    """

    def __init__(
        self,
        dt_s: int = 60,
        gamma: float = 0.0,
        text_policy: str = "hold_last",
        u_clip: tuple[float, float] = (0.0, 1.0),
        max_deadtime_s: int = 4 * 3600,
        # --- Advanced diagnostics (§15) ---
        rmse_window: int = 30,
        rmse_reliable_threshold: float = 0.5,
        cusum_delta: float = 0.15,
        cusum_threshold: float = 2.0,
        N_sat: int = 20,
        dT_sat_threshold: float = 0.005,
        eps_sat: float = 0.3,
        d_hat_alpha: float = 0.05,
        d_hat_u_min: float = 0.05,  # v4 §15.6.4: freeze d_hat when u < this
        d_hat_max: float = 0.1,     # v4.1 §15.6.5: clamp |d_hat_ema| (°C/min)
        d_hat_relax: float = 0.005, # v4.1: slow relaxation rate toward 0 when frozen
    ) -> None:
        # ---- Input validation ----
        if dt_s <= 0:
            raise ValueError(f"dt_s must be > 0, got {dt_s}")
        if not 0.0 <= gamma <= 1.0:
            raise ValueError(
                f"gamma must be in [0, 1] for observer stability, got {gamma}"
            )

        self.dt_s: int = int(dt_s)
        self.dt_min: float = self.dt_s / 60.0
        self.gamma: float = float(gamma)
        self.text_policy: str = text_policy
        self.u_min: float = float(u_clip[0])
        self.u_max: float = float(u_clip[1])
        self.max_deadtime_s: int = int(max_deadtime_s)

        # Advanced diagnostics config
        self.rmse_window: int = rmse_window
        self.rmse_reliable_threshold: float = rmse_reliable_threshold
        self.cusum_delta: float = cusum_delta
        self.cusum_threshold: float = cusum_threshold
        self.N_sat: int = N_sat
        self.dT_sat_threshold: float = dT_sat_threshold
        self.eps_sat: float = eps_sat
        self.d_hat_alpha: float = d_hat_alpha
        self.d_hat_u_min: float = d_hat_u_min
        self.d_hat_max: float = d_hat_max
        self.d_hat_relax: float = d_hat_relax

        # Core state — u_buffer has NO maxlen (critical: prevents infinite loop)
        self.T_hat: float | None = None
        self.T_pred: float | None = None
        self.last_text: float | None = None
        self.dead_steps: int = 0
        self.u_buffer: deque[float] = deque([0.0])

        # Advanced diagnostics state
        self.innovation_buffer: deque[float] = deque(maxlen=rmse_window)
        self.innovation_pure_buffer: deque[float] = deque(maxlen=rmse_window)
        self.cusum_pos: float = 0.0
        self.cusum_neg: float = 0.0
        self.sat_count: int = 0
        self.last_tin_meas: float | None = None
        self.d_hat_ema: float = 0.0

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(
        self,
        tin_init: float,
        text_init: float | None = None,
        u_init: float = 0.0,
    ) -> None:
        """Initialise or warm-restart the twin."""
        self.T_hat = float(tin_init)
        self.T_pred = None
        if text_init is not None:
            self.last_text = float(text_init)
        u_val = _clamp(float(u_init), self.u_min, self.u_max)
        n = self.dead_steps + 1
        # NO maxlen — size controlled by step() and _resize_buffer()
        self.u_buffer = deque([u_val] * n)
        # Reset advanced diagnostics state
        self.innovation_buffer.clear()
        self.innovation_pure_buffer.clear()
        self.cusum_pos = 0.0
        self.cusum_neg = 0.0
        self.sat_count = 0
        self.last_tin_meas = None
        self.d_hat_ema = 0.0

    # ------------------------------------------------------------------
    # Dead-time buffer management
    # ------------------------------------------------------------------

    def _resize_buffer(self, new_dead_steps: int) -> None:
        """Resize u_buffer to match new_dead_steps, preserving recent history."""
        new_n = new_dead_steps + 1
        old_n = len(self.u_buffer)

        if new_n > old_n:
            pad_val = self.u_buffer[0] if self.u_buffer else 0.0
            for _ in range(new_n - old_n):
                self.u_buffer.appendleft(pad_val)
        elif new_n < old_n:
            while len(self.u_buffer) > new_n:
                self.u_buffer.popleft()

        self.dead_steps = new_dead_steps

    def _normalize_buffer(self) -> None:
        """Ensure u_buffer invariant: len == dead_steps + 1.

        Called after load_state() to guarantee consistency.
        """
        n = self.dead_steps + 1
        while len(self.u_buffer) < n:
            pad = self.u_buffer[0] if self.u_buffer else 0.0
            self.u_buffer.appendleft(pad)
        while len(self.u_buffer) > n:
            self.u_buffer.popleft()

    # ------------------------------------------------------------------
    # Step (one tick of simulation) — v4.1 dual prediction architecture
    # ------------------------------------------------------------------

    def step(  # pylint: disable=too-many-locals,too-many-branches,too-many-statements
        self,
        tin_meas: float,
        text_meas: float | None,
        a: float,
        b: float,
        u_now: float,
        deadtime_s: float,
        sp: float | None = None,
    ) -> dict:
        """Simulate one time-step and return diagnostics dict."""
        if self.T_hat is None:
            return {"status": "not_initialized"}

        # ---- Text policy ----
        text_used = _resolve_text(text_meas, self.text_policy, self.last_text)
        if text_meas is not None:
            self.last_text = float(text_meas)

        if text_used is None:
            return {"status": "missing_text", "T_hat_prev": self.T_hat}

        # ---- Params validation ----
        if not isfinite(a) or not isfinite(b) or b <= 0 or a < 0:
            return {
                "status": "invalid_params",
                "T_hat_prev": self.T_hat,
                "Text_used": text_used,
            }

        # ---- Deadtime buffer sizing ----
        dead_steps = int(round(
            _clamp(deadtime_s, 0, self.max_deadtime_s) / self.dt_s
        ))
        if dead_steps != self.dead_steps:
            self._resize_buffer(dead_steps)

        # ---- Push u and get u_eff ----
        n = self.dead_steps + 1
        u = _clamp(float(u_now), self.u_min, self.u_max)
        self.u_buffer.append(u)
        while len(self.u_buffer) > n:
            self.u_buffer.popleft()
        while len(self.u_buffer) < n:
            self.u_buffer.appendleft(
                self.u_buffer[0] if self.u_buffer else 0.0
            )
        u_eff = self.u_buffer[0]

        # ---- Exact discretisation (ZOH, unconditionally stable) ----
        t_prev = self.T_hat
        alpha = exp(-b * self.dt_min)
        one_minus_alpha = 1.0 - alpha

        # ── PURE prediction (drives T_hat and d_raw) ──
        t_eq_raw = text_used + (a / b) * u_eff
        t_pred_pure = t_eq_raw + (t_prev - t_eq_raw) * alpha

        # ── Innovation on pure model ──
        innovation_pure = float(tin_meas) - t_pred_pure

        # ── d_hat estimation (§15.6) ──
        if u_eff >= self.d_hat_u_min and one_minus_alpha > 1e-12:
            # Active heating: estimate perturbation from innovation
            d_raw = b * innovation_pure / one_minus_alpha
            self.d_hat_ema = (
                self.d_hat_alpha * d_raw
                + (1.0 - self.d_hat_alpha) * self.d_hat_ema
            )
            # Physical clamp (v4.1 §15.6.5)
            self.d_hat_ema = _clamp(
                self.d_hat_ema, -self.d_hat_max, self.d_hat_max
            )
        else:
            # u too low: slow relaxation toward 0 instead of hard freeze (v4.1).
            # Avoids stale solar bias when heating OFF at night.
            # d_hat_relax=0.005 → 95% decay in ~600 steps (10h).
            self.d_hat_ema *= (1.0 - self.d_hat_relax)

        # ── CORRECTED prediction (diagnostics only) ──
        t_eq_corr = text_used + (a * u_eff + self.d_hat_ema) / b
        t_pred = t_eq_corr + (t_prev - t_eq_corr) * alpha

        # ── Innovation on corrected model (for RMSE, CUSUM) ──
        innovation = float(tin_meas) - t_pred

        # ---- Nudge (Luenberger, on PURE model — §4.2) ----
        t_next = t_pred_pure + self.gamma * innovation_pure

        # ---- Update state ----
        self.T_hat = t_next
        self.T_pred = t_pred

        # ---- Advanced diagnostics (§15) ----
        adv = self._compute_advanced_diagnostics(
            innovation=innovation,
            innovation_pure=innovation_pure,
            a=a, b=b,
            u_now=u,
            tin_meas=float(tin_meas),
            text_used=text_used,
            sp=sp,
        )

        return {
            "status": "ok",
            "T_hat_prev": t_prev,
            "T_hat_next": t_next,
            "T_pred": t_pred,
            "Tin_meas": float(tin_meas),
            "Text_used": text_used,
            "a": float(a),
            "b": float(b),
            "tau_min": 1.0 / float(b),
            "deadtime_s": float(deadtime_s),
            "dead_steps": int(self.dead_steps),
            "u_now": u,
            "u_eff": u_eff,
            "gamma": float(self.gamma),
            "innovation": innovation,
            "d_hat_ema": round(self.d_hat_ema, 6),
            **adv,
        }

    # ------------------------------------------------------------------
    # Advanced diagnostics (§15) — v4.1
    # ------------------------------------------------------------------

    def _compute_advanced_diagnostics(
        self,
        innovation: float,
        innovation_pure: float,
        a: float,
        b: float,
        u_now: float,
        tin_meas: float,
        text_used: float,
        sp: float | None = None,
    ) -> dict:
        """Compute advanced diagnostics from innovation signals."""

        # ---- §15.1 Sliding RMSE ----
        self.innovation_buffer.append(innovation)
        self.innovation_pure_buffer.append(innovation_pure)

        if len(self.innovation_buffer) >= 2:
            rmse = sqrt(
                sum(e ** 2 for e in self.innovation_buffer)
                / len(self.innovation_buffer)
            )
        else:
            rmse = None

        if len(self.innovation_pure_buffer) >= 2:
            rmse_pure = sqrt(
                sum(e ** 2 for e in self.innovation_pure_buffer)
                / len(self.innovation_pure_buffer)
            )
        else:
            rmse_pure = None

        model_reliable = rmse is not None and rmse < self.rmse_reliable_threshold

        # ---- §15.2 CUSUM with reset (v4.1) ----
        self.cusum_pos = max(0.0, self.cusum_pos + innovation - self.cusum_delta)
        self.cusum_neg = max(0.0, self.cusum_neg - innovation - self.cusum_delta)
        external_gain_detected = self.cusum_pos > self.cusum_threshold
        external_loss_detected = self.cusum_neg > self.cusum_threshold
        if external_gain_detected:
            self.cusum_pos = 0.0
        if external_loss_detected:
            self.cusum_neg = 0.0

        # ---- §15.3 Perturbation ----
        perturbation_dTdt = b * innovation

        # ---- §15.4 T_steady ----
        T_steady = text_used + (a * u_now + self.d_hat_ema) / b
        T_steady_valid = (T_STEADY_MIN <= T_steady <= T_STEADY_MAX)
        if not T_steady_valid:
            model_reliable = False

        setpoint_reachable = (
            (T_steady >= sp)
            if (sp is not None and T_steady_valid)
            else None
        )

        # ---- §15.5 Emitter saturation ----
        if u_now >= 0.95:
            self.sat_count += 1
        else:
            self.sat_count = 0

        dTin_dt = None
        if self.last_tin_meas is not None:
            dTin_dt = (tin_meas - self.last_tin_meas) / self.dt_min
        self.last_tin_meas = tin_meas

        emitter_saturated = (
            sp is not None
            and self.sat_count >= self.N_sat
            and dTin_dt is not None
            and abs(dTin_dt) < self.dT_sat_threshold
            and tin_meas < sp - self.eps_sat
        )

        return {
            "rmse_30": round(rmse, 4) if rmse is not None else None,
            "rmse_pure": round(rmse_pure, 4) if rmse_pure is not None else None,
            "model_reliable": model_reliable,
            "perturbation_dTdt": round(perturbation_dTdt, 6),
            "cusum_pos": round(self.cusum_pos, 4),
            "cusum_neg": round(self.cusum_neg, 4),
            "external_gain_detected": external_gain_detected,
            "external_loss_detected": external_loss_detected,
            "T_steady": round(T_steady, 2),
            "T_steady_valid": T_steady_valid,
            "setpoint_reachable": setpoint_reachable,
            "emitter_saturated": emitter_saturated,
        }

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_state(self) -> dict:
        """Save twin state for persistence across restarts."""
        return {
            "T_hat": self.T_hat,
            "T_pred": self.T_pred,
            "last_text": self.last_text,
            "dead_steps": self.dead_steps,
            "u_buffer": list(self.u_buffer),
            "cusum_pos": self.cusum_pos,
            "cusum_neg": self.cusum_neg,
            "innovation_buffer": list(self.innovation_buffer),
            "innovation_pure_buffer": list(self.innovation_pure_buffer),
            "sat_count": self.sat_count,
            "last_tin_meas": self.last_tin_meas,
            "d_hat_ema": self.d_hat_ema,
        }

    def load_state(self, state: dict) -> None:
        """Restore twin state from persisted data.

        Normalizes u_buffer after loading to guarantee len == dead_steps + 1.
        """
        if not state:
            return
        self.T_hat = state.get("T_hat")
        self.T_pred = state.get("T_pred")
        self.last_text = state.get("last_text")
        ds = state.get("dead_steps", 0)
        self.dead_steps = int(ds)
        buf = state.get("u_buffer", [])
        self.u_buffer = deque(buf) if buf else deque([0.0] * (self.dead_steps + 1))
        self._normalize_buffer()
        self.cusum_pos = state.get("cusum_pos", 0.0)
        self.cusum_neg = state.get("cusum_neg", 0.0)
        self.sat_count = state.get("sat_count", 0)
        self.last_tin_meas = state.get("last_tin_meas")
        self.d_hat_ema = state.get("d_hat_ema", 0.0)
        inno_buf = state.get("innovation_buffer", [])
        self.innovation_buffer = deque(inno_buf, maxlen=self.rmse_window)
        inno_pure_buf = state.get("innovation_pure_buffer", [])
        self.innovation_pure_buffer = deque(inno_pure_buf, maxlen=self.rmse_window)

    # ------------------------------------------------------------------
    # Convenience: update twin + compute ETA in one call
    # ------------------------------------------------------------------

    def update_with_eta(
        self,
        tin: float,
        text: float | None,
        target: float,
        on_percent: float,
        tau_reliable: bool,
        a: float,
        b: float,
        mode: str,
        deadtime_heat_s: float | None,
        deadtime_cool_s: float | None,
        deadtime_heat_reliable: bool,
        deadtime_cool_reliable: bool,
    ) -> dict:
        """Update thermal twin and compute ETA best-case (diagnostics-only)."""
        if self.T_hat is None:
            if not tau_reliable:
                return {"status": "not_reliable"}
            self.reset(tin, text, u_init=on_percent)

        deadtime_s = deadtime_heat_s or 0.0
        twin_result = self.step(
            tin_meas=tin,
            text_meas=text,
            a=a, b=b,
            u_now=on_percent,
            deadtime_s=deadtime_s,
            sp=target,
        )

        if tau_reliable:
            eta_result = eta_best_case(
                tin0=tin, text=text, target=target,
                a=a, b=b, mode=mode,
                deadtime_heat_s=deadtime_heat_s,
                deadtime_cool_s=deadtime_cool_s,
                deadtime_heat_ok=deadtime_heat_reliable,
                deadtime_cool_ok=deadtime_cool_reliable,
                last_text=self.last_text,
                d_hat_ema=self.d_hat_ema,
            )
        else:
            eta_result = {"eta_s": None, "reason": "not_reliable"}

        return {
            **twin_result,
            **{f"eta_{k}": v for k, v in eta_result.items()},
        }


# ---------------------------------------------------------------------------
# ETA best-case (standalone function)
# ---------------------------------------------------------------------------

EPS_DENOM = 1e-6
MAX_ETA_S = 48 * 3600


def eta_best_case(  # pylint: disable=too-many-arguments,too-many-return-statements
    tin0: float,
    text: float | None,
    target: float,
    a: float,
    b: float,
    mode: str,
    deadtime_heat_s: float | None,
    deadtime_cool_s: float | None,
    deadtime_heat_ok: bool,
    deadtime_cool_ok: bool,
    eps: float = 0.1,
    eps_denom: float = EPS_DENOM,
    max_eta_s: float = MAX_ETA_S,
    text_policy: str = "hold_last",
    last_text: float | None = None,
    d_hat_ema: float = 0.0,
) -> dict:
    """Compute best-case ETA to reach *target* from *tin0*."""
    if abs(tin0 - target) <= eps:
        return {"eta_s": 0.0, "reason": "already_reached"}

    text_used = _resolve_text(text, text_policy, last_text)
    if text_used is None:
        return {"eta_s": None, "reason": "missing_text"}

    if not isfinite(a) or not isfinite(b) or b <= 0 or a < 0:
        return {"eta_s": None, "reason": "invalid_params"}

    tau_min = 1.0 / b

    if mode == "heat":
        u = 1.0
        l_s = (deadtime_heat_s
               if (deadtime_heat_ok and deadtime_heat_s is not None)
               else 0.0)
    else:
        u = 0.0
        l_s = (deadtime_cool_s
               if (deadtime_cool_ok and deadtime_cool_s is not None)
               else 0.0)

    t_inf = text_used + (a * u + d_hat_ema) / b

    denom = tin0 - t_inf
    if abs(denom) < eps_denom:
        return {"eta_s": None, "reason": "unreachable", "T_inf": t_inf}

    rho = (target - t_inf) / denom
    if rho <= 0 or rho >= 1:
        return {"eta_s": None, "reason": "unreachable",
                "T_inf": t_inf, "rho": rho}

    t_reach_min = (l_s / 60.0) - tau_min * log(rho)
    eta_s = max(0.0, t_reach_min * 60.0)

    if eta_s > max_eta_s:
        return {"eta_s": eta_s, "reason": "too_far", "T_inf": t_inf,
                "tau_min": tau_min, "L_s": l_s, "u": u, "rho": rho}

    return {"eta_s": eta_s, "reason": "ok", "T_inf": t_inf,
            "tau_min": tau_min, "L_s": l_s, "u": u, "rho": rho}
