import pytest
from custom_components.versatile_thermostat.vtherm_hvac_mode import (
    VThermHvacMode_HEAT,
    VThermHvacMode_COOL,
)
from custom_components.versatile_thermostat.smartpi.controller import SmartPIController
from custom_components.versatile_thermostat.smartpi.const import KI_MIN, AW_TRACK_TAU_S


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_ctl(integral: float = 0.5, i_mode: str = "I:RUN") -> SmartPIController:
    """Return a pre-configured controller."""
    ctl = SmartPIController("test_aw")
    ctl.integral = integral
    ctl.last_i_mode = i_mode
    # Set u_ff / u_cmd / u_pi to neutral values (model output)
    ctl.u_ff = 0.0
    ctl.u_pi = 0.2
    ctl.u_cmd = 0.2
    return ctl


COMMON_KWARGS = dict(
    dt_min=5.0,
    ki=0.05,
    kp=0.5,
    error_p=0.3,        # kp * e_p = 0.15, ki * integral = 0.05 * 0.5 = 0.025
    integrator_hold=False,
    in_deadband=False,
    max_on_percent=None,
    current_temp=20.0,
    target_temp=21.0,   # Tin < SP → no thermal invariant active
    hysteresis_thermal_guard=False,
    hvac_mode=VThermHvacMode_HEAT,
)


# ---------------------------------------------------------------------------
# T1 — AW blocked in I:SKIP
# ---------------------------------------------------------------------------

def test_t1_aw_blocked_in_skip():
    """AW must not modify integral when last_i_mode is I:SKIP(SAT_LO)."""
    ctl = make_ctl(integral=0.5, i_mode="I:SKIP(SAT_LO)")
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.0, u_applied=0.0,
        current_temp=22.0, target_temp=21.0,  # Tin > SP
        **{k: v for k, v in COMMON_KWARGS.items()
           if k not in ("current_temp", "target_temp")}
    )
    assert ctl.integral == before, "Integral must not change when I:SKIP is active"


# ---------------------------------------------------------------------------
# T2 — AW blocked in I:HOLD
# ---------------------------------------------------------------------------

def test_t2_aw_blocked_in_hold():
    """AW must not modify integral when last_i_mode is I:HOLD."""
    ctl = make_ctl(integral=0.5, i_mode="I:HOLD")
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.05, u_applied=0.05,
        **COMMON_KWARGS
    )
    assert ctl.integral == before, "Integral must not change when I:HOLD is active"


# ---------------------------------------------------------------------------
# T3 — AW blocked in I:FREEZE (deadband guard via Priority 1)
# ---------------------------------------------------------------------------

def test_t3_aw_blocked_in_freeze():
    """AW must not modify integral when last_i_mode is I:FREEZE."""
    ctl = make_ctl(integral=0.5, i_mode="I:FREEZE(deadband)")
    before = ctl.integral
    # in_deadband=False so structural pre-condition does not catch it
    ctl.update_anti_windup(
        u_limited=0.05, u_applied=0.05,
        **COMMON_KWARGS
    )
    assert ctl.integral == before, "Integral must not change when I:FREEZE is active"


# ---------------------------------------------------------------------------
# T4 — AW blocked in I:GUARD
# ---------------------------------------------------------------------------

def test_t4_aw_blocked_in_guard():
    """AW must not modify integral when last_i_mode is I:GUARD."""
    ctl = make_ctl(integral=0.5, i_mode="I:GUARD(freeze)")
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.05, u_applied=0.05,
        **COMMON_KWARGS
    )
    assert ctl.integral == before, "Integral must not change when I:GUARD is active"


# ---------------------------------------------------------------------------
# T5 — AW blocked in I:CLAMP
# ---------------------------------------------------------------------------

def test_t5_aw_blocked_in_clamp():
    """AW must not modify integral when last_i_mode is I:CLAMP."""
    ctl = make_ctl(integral=0.5, i_mode="I:CLAMP(near_ovr)")
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.05, u_applied=0.05,
        **COMMON_KWARGS
    )
    assert ctl.integral == before, "Integral must not change when I:CLAMP is active"


# ---------------------------------------------------------------------------
# T6 — AW discharge-only in HEAT when Tin > SP (Priority 2)
# ---------------------------------------------------------------------------

def test_t6_aw_discharge_only_heat_overshoot():
    """When Tin > SP in HEAT mode, AW may only reduce (or keep) the integral."""
    ctl = make_ctl(integral=0.5, i_mode="I:RUN")
    # u_model = u_ff + kp*ep + ki*I = 0 + 0.5*0.3 + 0.05*0.5 = 0.175
    # u_aw_ref = u_applied = 0.3 → du = 0.3 - 0.175 = +0.125 > 0 → normally d_integral > 0
    # Priority 2 (Tin > SP) should clamp to 0 or negative
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.3,
        u_applied=0.3,
        current_temp=22.0,  # Tin > SP=21.0
        target_temp=21.0,
        dt_min=5.0,
        ki=0.05,
        kp=0.5,
        error_p=0.3,
        integrator_hold=False,
        in_deadband=False,
        max_on_percent=None,
        hysteresis_thermal_guard=False,
        hvac_mode=VThermHvacMode_HEAT,
    )
    assert ctl.integral <= before, "Integral must not increase when Tin > SP in HEAT"


# ---------------------------------------------------------------------------
# T7 — u_applied = 0 drives integral down (Modif 1)
# ---------------------------------------------------------------------------

def test_t7_u_applied_zero_reduces_integral():
    """With u_applied=0 and u_limited>0, back-calc du must be negative → integral decreases."""
    ctl = make_ctl(integral=0.5, i_mode="I:RUN")
    # u_model = 0 + 0.5*0.3 + 0.05*0.5 = 0.175
    # u_aw_ref = u_applied = 0.0 → du = 0.0 - 0.175 = -0.175 < 0 → d_integral < 0
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.15,     # previous "old" reference, now irrelevant
        u_applied=0.0,      # timing forces off
        current_temp=20.0,  # Tin < SP → no thermal invariant active
        target_temp=21.0,
        dt_min=5.0,
        ki=0.05,
        kp=0.5,
        error_p=0.3,
        integrator_hold=False,
        in_deadband=False,
        max_on_percent=None,
        hysteresis_thermal_guard=False,
        hvac_mode=VThermHvacMode_HEAT,
    )
    assert ctl.integral < before, "Integral must decrease when u_applied=0 and u_model>0"


# ---------------------------------------------------------------------------
# T8 — Nominal case: u_applied ≈ u_limited, I:RUN, Tin < SP → AW free
# ---------------------------------------------------------------------------

def test_t8_nominal_no_saturation():
    """In nominal conditions (no saturation, Tin < SP, I:RUN), AW applies freely."""
    ctl = make_ctl(integral=0.5, i_mode="I:RUN")
    # u_model = 0 + 0.5*0.3 + 0.05*0.5 = 0.175
    # u_applied ≈ u_limited = 0.20 → du = 0.20 - 0.175 = +0.025 → d_integral > 0 (small)
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.20,
        u_applied=0.20,
        current_temp=20.0,  # Tin < SP (21.0)
        target_temp=21.0,
        dt_min=5.0,
        ki=0.05,
        kp=0.5,
        error_p=0.3,
        integrator_hold=False,
        in_deadband=False,
        max_on_percent=None,
        hysteresis_thermal_guard=False,
        hvac_mode=VThermHvacMode_HEAT,
    )
    # du > 0 → d_integral > 0 → integral increased slightly
    assert ctl.integral > before, "In nominal conditions AW should apply freely"


# ---------------------------------------------------------------------------
# T9 — COOL symmetry: Tin < SP → discharge only (integral must not decrease)
# ---------------------------------------------------------------------------

def test_t9_cool_symmetry_discharge_only():
    """In COOL mode, when Tin < SP, AW may only increase (or keep) the integral."""
    ctl = make_ctl(integral=-0.5, i_mode="I:RUN")
    # Set u_ff and u_cmd to be consistent
    ctl.u_ff = 0.0
    ctl.u_cmd = 0.2
    # u_model = 0 + 0.5*0.3 + 0.05*(-0.5) = 0.15 - 0.025 = 0.125
    # u_applied = 0.3 → du = 0.3 - 0.125 = +0.175 → normally d_integral > 0
    # COOL + Tin < SP → Priority 2 → d_integral = max(0, d_integral) → OK (positive)
    before = ctl.integral
    ctl.update_anti_windup(
        u_limited=0.3,
        u_applied=0.3,
        current_temp=18.0,  # Tin < SP=20.0
        target_temp=20.0,
        dt_min=5.0,
        ki=0.05,
        kp=0.5,
        error_p=0.3,
        integrator_hold=False,
        in_deadband=False,
        max_on_percent=None,
        hysteresis_thermal_guard=False,
        hvac_mode=VThermHvacMode_COOL,
    )
    # In COOL with Tin < SP and du > 0, d_integral should remain >= 0 (no reduction)
    assert ctl.integral >= before, "Integral must not decrease when Tin < SP in COOL (discharge only upwards)"
