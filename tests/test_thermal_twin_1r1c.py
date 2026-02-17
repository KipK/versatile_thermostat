"""Unit tests for ThermalTwin1R1C and eta_best_case."""

import math

import pytest

from custom_components.versatile_thermostat.smartpi.thermal_twin_1r1c import (
    ThermalTwin1R1C,
    eta_best_case,
)


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------

def _make_twin(**kwargs) -> ThermalTwin1R1C:
    """Convenience factory with sensible defaults."""
    return ThermalTwin1R1C(**kwargs)


# -----------------------------------------------------------------------
# 10.1 Dead-time buffer
# -----------------------------------------------------------------------

def test_deadtime_buffer():
    """dt=60s, deadtime=180s => dead_steps=3, N=4.
    Inject u=1.0 for 1 step after reset with u_init=0.0.
    u_eff must remain 0.0 for 3 steps then become 1.0.
    """
    twin = _make_twin(dt_s=60, gamma=0.0)
    twin.set_deadtime(180)
    assert twin.dead_steps == 3

    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.0)

    # Buffer should be filled with 0.0 (dead_steps+1 = 4 entries)
    assert len(twin.u_buffer) == 4

    a, b, text = 0.01, 0.01, 5.0
    tin = 20.0

    # Step 1: inject u=1.0
    r = twin.step(tin_meas=tin, text_meas=text, a=a, b=b,
                  u_now=1.0, deadtime_s=180)
    assert r["u_eff"] == 0.0

    # Steps 2-3: keep injecting u=1.0, u_eff should still be 0.0
    for _ in range(2):
        r = twin.step(tin_meas=tin, text_meas=text, a=a, b=b,
                      u_now=1.0, deadtime_s=180)
        assert r["u_eff"] == 0.0

    # Step 4: u_eff should now be 1.0
    r = twin.step(tin_meas=tin, text_meas=text, a=a, b=b,
                  u_now=1.0, deadtime_s=180)
    assert r["u_eff"] == 1.0


# -----------------------------------------------------------------------
# 10.2 Convergence to steady-state
# -----------------------------------------------------------------------

def test_convergence():
    """Text=5, u=1.0, gamma=0, a=0.05, b=0.01.
    T_inf = 5 + 0.05/0.01 = 10. Starting from Tin=15, T_hat should
    converge monotonically downward to 10.
    """
    twin = _make_twin(dt_s=60, gamma=0.0)
    twin.reset(tin_init=15.0, text_init=5.0, u_init=1.0)
    twin.set_deadtime(0)

    a, b, text = 0.05, 0.01, 5.0
    t_inf = text + a / b  # 10.0

    # Analytical checking: feed EXACT physics to the twin.
    # If the twin is perfect, d_hat should stay ~0 and T_hat should follow.
    t_current_truth = 15.0
    dt_min = 60.0 / 60.0  # 1 min

    for _ in range(600):
        # 1. Update physical truth
        # Exact solution step: T_next = T_inf + (T_prev - T_inf) * exp(-b*dt)
        # Here u=1.0 is constant.
        t_current_truth = t_inf + (t_current_truth - t_inf) * math.exp(-b * dt_min)

        # 2. Feed truth to twin
        r = twin.step(tin_meas=t_current_truth, text_meas=text, a=a, b=b,
                      u_now=1.0, deadtime_s=0)
        t_hat = r["T_hat_next"]

        # 3. Check tracking
        # The twin's T_hat should match the truth very closely (innovation ~ 0)
        assert abs(t_hat - t_current_truth) < 1e-4

    # Final convergence check
    # Final convergence check
    assert abs(t_hat - t_inf) < 0.02


# -----------------------------------------------------------------------
# 10.3 Nudging keeps T_hat bounded
# -----------------------------------------------------------------------

def test_nudging():
    """gamma=0.1, sinusoidal measurement noise.
    T_hat should stay bounded (not diverge).
    """
    twin = _make_twin(dt_s=60, gamma=0.1)
    twin.set_deadtime(0)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.5)

    a, b, text = 0.01, 0.01, 5.0

    for step in range(50):
        tin_meas = 20.0 + 0.5 * math.sin(step / 5.0)
        r = twin.step(tin_meas=tin_meas, text_meas=text, a=a, b=b,
                      u_now=0.5, deadtime_s=0)
        t_hat = r["T_hat_next"]
        # T_hat should stay in a reasonable range around the measurement
        assert 0.0 < t_hat < 40.0, f"T_hat diverged at step {step}: {t_hat}"


# -----------------------------------------------------------------------
# 10.4 ETA heat reachable
# -----------------------------------------------------------------------

def test_eta_heat_reachable():
    """a=0.5, b=0.01, text=5 => T_inf(u=1)=55. target=20 < 55 => reachable."""
    result = eta_best_case(
        tin0=15, text=5, target=20, a=0.5, b=0.01, mode="heat",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "ok"
    assert result["eta_s"] > 0


# -----------------------------------------------------------------------
# 10.4 ETA heat unreachable
# -----------------------------------------------------------------------

def test_eta_heat_unreachable():
    """a=0.5, b=0.01, text=5 => T_inf=55. target=60 > 55 => unreachable."""
    result = eta_best_case(
        tin0=15, text=5, target=60, a=0.5, b=0.01, mode="heat",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "unreachable"


# -----------------------------------------------------------------------
# 10.4 ETA cool reachable
# -----------------------------------------------------------------------

def test_eta_cool_reachable():
    """tin0=25, text=5, target=15, mode=cool. u=0 => T_inf=5. 5<15<25 => ok."""
    result = eta_best_case(
        tin0=25, text=5, target=15, a=0.5, b=0.01, mode="cool",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "ok"
    assert result["eta_s"] > 0


# -----------------------------------------------------------------------
# 10.4 ETA cool unreachable
# -----------------------------------------------------------------------

def test_eta_cool_unreachable():
    """tin0=25, text=5, target=3, mode=cool. T_inf=5. target=3 < T_inf => unreachable."""
    result = eta_best_case(
        tin0=25, text=5, target=3, a=0.5, b=0.01, mode="cool",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "unreachable"


# -----------------------------------------------------------------------
# 10.4 ETA too_far
# -----------------------------------------------------------------------

def test_eta_too_far():
    """target very close to T_inf=55 => rho ~ 0 => huge eta => too_far."""
    # With T_inf=55 and tin0=15, rho=(target-55)/(15-55).
    # We need eta > 48h, so target must be extremely close to T_inf.
    result = eta_best_case(
        tin0=15, text=5, target=55.0 - 1e-12, a=0.5, b=0.01, mode="heat",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "too_far"


# -----------------------------------------------------------------------
# 10.5 Numerical stability (large b => small tau)
# -----------------------------------------------------------------------

def test_numerical_stability():
    """b=6.0 (tau=10s), dt=60s. Exact exponential must remain stable:
    no oscillation (no sign change in consecutive deltas).
    """
    twin = _make_twin(dt_s=60, gamma=0.0)
    twin.set_deadtime(0)
    twin.reset(tin_init=15.0, text_init=5.0, u_init=1.0)

    a, b, text = 0.5, 6.0, 5.0
    t_current_truth = 15.0
    dt_min = 60.0 / 60.0
    t_inf = text + a / b * 1.0
    prev_t = 15.0
    deltas = []

    for _ in range(100):
        t_current_truth = t_inf + (t_current_truth - t_inf) * math.exp(-b * dt_min)

        r = twin.step(tin_meas=t_current_truth, text_meas=text, a=a, b=b,
                      u_now=1.0, deadtime_s=0)
        t_hat = r["T_hat_next"]
        delta = t_hat - prev_t
        deltas.append(delta)
        prev_t = t_hat

    # No sign changes in consecutive deltas (no oscillation)
    for i in range(1, len(deltas)):
        # Allow zero delta (convergence), but not sign reversal
        if abs(deltas[i]) > 1e-15 and abs(deltas[i - 1]) > 1e-15:
            assert deltas[i] * deltas[i - 1] >= 0, (
                f"Oscillation at step {i}: delta[{i-1}]={deltas[i-1]}, delta[{i}]={deltas[i]}"
            )


# -----------------------------------------------------------------------
# 10.6 Realistic tau — comparison with analytical solution
# -----------------------------------------------------------------------

def test_realistic_tau():
    """b=0.00833, a=0.05, text=5, T0=15, u=1.
    T_inf = 5 + 0.05/0.00833 ~ 11.0. Run 600 steps (10h).
    Verify convergence and error vs analytical < 0.1%.
    """
    b = 0.00833
    a = 0.05
    text = 5.0
    tin0 = 15.0
    t_inf = text + a / b
    dt_s = 60
    dt_min = dt_s / 60.0

    twin = _make_twin(dt_s=dt_s, gamma=0.0)
    twin.set_deadtime(0)
    twin.reset(tin_init=tin0, text_init=text, u_init=1.0)


    for step in range(1, 601):
        # Analytical solution: T(t) = T_inf + (T0 - T_inf) * exp(-b * t)
        t_minutes = step * dt_min
        t_analytical = t_inf + (tin0 - t_inf) * math.exp(-b * t_minutes)

        # Feed the analytical truth as measurement
        r = twin.step(tin_meas=t_analytical, text_meas=text, a=a, b=b,
                      u_now=1.0, deadtime_s=0)
        t_hat = r["T_hat_next"]

        # Relative error < 0.1%
        if abs(t_analytical) > 1e-6:
            rel_err = abs(t_hat - t_analytical) / abs(t_analytical)
            assert rel_err < 0.001, (
                f"Step {step}: T_hat={t_hat:.6f}, T_anal={t_analytical:.6f}, "
                f"rel_err={rel_err:.6f}"
            )

        prev_t = t_hat

    # Final convergence check
    assert abs(prev_t - t_inf) < 0.1


# -----------------------------------------------------------------------
# 10.7 Warm reset u_init
# -----------------------------------------------------------------------

def test_warm_reset_u_init():
    """deadtime=180s (dead_steps=3). Reset with u_init=0.8.
    First step with u_now=0.8: u_eff should be 0.8 (not 0).
    """
    twin = _make_twin(dt_s=60, gamma=0.0)
    twin.set_deadtime(180)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.8)

    # Buffer should be filled with 0.8
    assert all(abs(v - 0.8) < 1e-12 for v in twin.u_buffer)

    r = twin.step(tin_meas=20.0, text_meas=5.0, a=0.05, b=0.01,
                  u_now=0.8, deadtime_s=180)
    assert abs(r["u_eff"] - 0.8) < 1e-12


# -----------------------------------------------------------------------
# 10.8 ETA denom zero
# -----------------------------------------------------------------------

def test_eta_denom_zero():
    """text=5, a=0.2, b=0.01 => T_inf=25.0. tin0=25.0.
    denom = tin0 - T_inf = 0 => unreachable.
    """
    result = eta_best_case(
        tin0=25.0, text=5.0, target=24.0, a=0.2, b=0.01, mode="heat",
        deadtime_heat_s=0, deadtime_cool_s=0,
        deadtime_heat_ok=True, deadtime_cool_ok=True,
    )
    assert result["reason"] == "unreachable"


# -----------------------------------------------------------------------
# 10.9 CUSUM gain detection
# -----------------------------------------------------------------------

def test_cusum_gain():
    """gamma=0. Inject tin_meas = T_pred + 0.3 for 20 steps after warmup.
    cusum_pos should cross threshold (default 2.0) and
    external_gain_detected should be True.
    """
    twin = _make_twin(dt_s=60, gamma=0.0, cusum_threshold=2.0, cusum_delta=0.15)
    twin.set_deadtime(0)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.5)

    a, b, text = 0.05, 0.01, 5.0

    # Warmup: a few steps with accurate measurements
    t_hat = 20.0
    for _ in range(3):
        r = twin.step(tin_meas=t_hat, text_meas=text, a=a, b=b,
                      u_now=0.5, deadtime_s=0)
        t_hat = r["T_hat_next"]

    # Now inject positive bias: tin_meas = T_pred + 0.3
    gain_detected = False
    for _ in range(20):
        r = twin.step(tin_meas=t_hat, text_meas=text, a=a, b=b,
                      u_now=0.5, deadtime_s=0)
        t_pred = r["T_pred"]
        # Next step measurement will be biased
        t_hat = t_pred + 0.3
        if r["external_gain_detected"]:
            gain_detected = True

    # After 20 biased steps, CUSUM should have triggered
    assert gain_detected, (
        f"CUSUM gain not detected. cusum_pos={r['cusum_pos']}"
    )
    # cusum_neg should remain near 0 (no loss signal)
    assert r["cusum_neg"] < 1.0


# -----------------------------------------------------------------------
# 10.10 RMSE sliding window
# -----------------------------------------------------------------------

def test_rmse_sliding():
    """gamma=0. 30 steps with tin_meas = T_pred + 0.5 => RMSE ~ 0.5.
    Then 30 steps with tin_meas = T_pred => RMSE drops to ~ 0.
    """
    twin = _make_twin(dt_s=60, gamma=0.0, rmse_window=30)
    twin.set_deadtime(0)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.5)

    a, b, text = 0.05, 0.01, 5.0

    # Phase 1: biased measurements (30 steps)
    t_hat = 20.0
    for _ in range(30):
        r = twin.step(tin_meas=t_hat, text_meas=text, a=a, b=b,
                      u_now=0.5, deadtime_s=0)
        t_pred = r["T_pred"]
        t_hat = t_pred + 0.5  # Bias

    rmse_biased = r["rmse_30"]
    assert rmse_biased is not None
    assert abs(rmse_biased - 0.5) < 0.15, f"Expected RMSE ~0.5, got {rmse_biased}"

    # Phase 2: accurate measurements (30 steps)
    for _ in range(30):
        r = twin.step(tin_meas=t_hat, text_meas=text, a=a, b=b,
                      u_now=0.5, deadtime_s=0)
        t_pred = r["T_pred"]
        t_hat = t_pred  # No bias

    rmse_accurate = r["rmse_30"]
    assert rmse_accurate is not None
    # After 30 accurate steps the sliding window should mostly contain
    # near-zero innovations, but transition effects can leave residuals.
    assert rmse_accurate < 0.20, f"Expected RMSE near 0, got {rmse_accurate}"
    # Also verify it dropped significantly from the biased phase
    assert rmse_accurate < rmse_biased, (
        f"RMSE did not drop: biased={rmse_biased}, accurate={rmse_accurate}"
    )


# -----------------------------------------------------------------------
# 10.11 Emitter saturation
# -----------------------------------------------------------------------

def test_emitter_saturation():
    """u=1.0, sp=25, tin stagnates at 20. After 25+ steps (N_sat=20),
    emitter_saturated should be True.
    """
    twin = _make_twin(dt_s=60, gamma=0.0, N_sat=20, dT_sat_threshold=0.005,
                      eps_sat=0.3)
    twin.set_deadtime(0)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=1.0)

    a, b, text = 0.05, 0.01, 5.0
    sp = 25.0
    tin = 20.0

    saturated = False
    for step in range(30):
        r = twin.step(tin_meas=tin, text_meas=text, a=a, b=b,
                      u_now=1.0, deadtime_s=0, sp=sp)
        if r.get("emitter_saturated"):
            saturated = True

    assert saturated, "Emitter saturation not detected after 30 steps"

    # Also check setpoint_reachable
    t_steady = text + (a / b) * 1.0  # 5 + 5 = 10
    assert r["setpoint_reachable"] is False  # 10 < 25


# -----------------------------------------------------------------------
# 10.12 T_pred None before first step
# -----------------------------------------------------------------------

def test_t_pred_none_before_step():
    """After reset(), T_pred should be None and innovation_buffer empty."""
    twin = _make_twin(dt_s=60, gamma=0.0)
    twin.reset(tin_init=20.0, text_init=5.0, u_init=0.0)

    assert twin.T_pred is None
    assert len(twin.innovation_buffer) == 0
