from __future__ import annotations

import logging
import math
from typing import Optional

from .const import (
    KP_SAFE,
    KI_SAFE,
    KP_MIN,
    KP_MAX,
    KI_MIN,
    KI_MAX,
    INTEGRAL_LEAK,
    INTEGRAL_DEADBAND_MICROLEAK,
    DEADBAND_PLUS_MIN_U,
    DEADBAND_PLUS_MAX_U,
    OVERSHOOT_I_CLAMP_EPS_C,
    AW_TRACK_TAU_S,
    AW_TRACK_MAX_DELTA_I,
    clamp
)
from ..vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL

_LOGGER = logging.getLogger(__name__)

class SmartPIController:
    """
    PID Controller for Smart-PI.
    
    Responsible for:
    1. PID Calculation (Proportional, Integral, FF).
    2. Integral management (Anti-windup, Leaks, Clamping).
    3. Hysteresis logic (Phase 1).
    4. Output saturation handling.
    """

    def __init__(self, name: str):
        self._name = name
        
        # PI State
        self.integral: float = 0.0
        self.u_prev: float = 0.0
        self.u_ff: float = 0.0
        self.u_pi: float = 0.0
        self.u_cmd: float = 0.0     # Requested (clamped 0-1)
        self.u_limited: float = 0.0 # After rate/max limit
        self.u_applied: float = 0.0 # Final realized
        
        # Diagnostics
        self.last_error: float = 0.0
        self.last_error_p: float = 0.0
        self.last_i_mode: str = "init"
        self.last_sat: str = "init"
        self.last_aw_du: float = 0.0
        
        # Hysteresis State
        self.hysteresis_state: str = "off"
        self.hysteresis_thermal_guard: bool = False

    @property
    def config(self):
        """Mock config for Energy Awareness tests."""
        from .const import KI_MIN
        class Config:
            i_max = 2.0 / KI_MIN
        return Config()
        
    def reset(self):
        self.integral = 0.0
        self.u_prev = 0.0
        self.u_ff = 0.0
        self.u_pi = 0.0
        self.last_error = 0.0
        self.last_error_p = 0.0
        self.hysteresis_state = "off"
        self.hysteresis_thermal_guard = False
        
        
    def bumpless_transfer(self, d_integral: float, ki: float) -> None:
        """Adjust integral for bumpless transfer."""
        self.integral += d_integral
        # Clamp against simplified safe limits if KI is valid
        if ki > KI_MIN:
            i_max = 2.0 / ki
            self.integral = clamp(self.integral, -i_max, i_max)

    def load_state(self, state: dict):
        if not state: return
        self.integral = float(state.get("integral") or 0.0)
        self.u_prev = float(state.get("u_prev") or 0.0)
        self.hysteresis_thermal_guard = bool(state.get("hysteresis_thermal_guard") or False)
        # Note: other internal diagnositcs not critical to restore
        
    def save_state(self) -> dict:
        return {
            "integral": self.integral,
            "u_prev": self.u_prev,
            "hysteresis_thermal_guard": self.hysteresis_thermal_guard
        }

    def calculate_hysteresis(
        self,
        target_temp: float,
        current_temp: float,
        hvac_mode: VThermHvacMode,
        hyst_upper: float,
        hyst_lower: float
    ) -> float:
        """Simple hysteresis control."""
        on_percent = 0.0
        
        if hvac_mode == VThermHvacMode_COOL:
             if current_temp <= target_temp - hyst_lower:
                on_percent = 0.0
                self.hysteresis_state = "off"
             elif current_temp >= target_temp + hyst_upper:
                on_percent = 1.0
                self.hysteresis_state = "on"
             else:
                self.hysteresis_state = "band"
                on_percent = None # No change
        else: # HEAT
            if current_temp >= target_temp + hyst_upper:
                on_percent = 0.0
                self.hysteresis_state = "off"
            elif current_temp <= target_temp - hyst_lower:
                on_percent = 1.0
                self.hysteresis_state = "on"
            else:
                self.hysteresis_state = "band"
                on_percent = None # No change
                
        # If in band, we need to know previous state. 
        # But this function is stateless regarding previous OUTPUT, only previous HYST STATE.
        # Actually in the main class it sets self._on_percent directly.
        # We will return None if "no change"
        return on_percent

    def compute_pwm(
        self,
        error: float,
        error_p: float,
        kp: float,
        ki: float,
        u_ff: float,
        dt_min: float,
        cycle_min: float,
        in_deadband: bool,
        integrator_hold: bool,
        hvac_mode: VThermHvacMode,
        current_temp: float,
        target_temp: float,
        hysteresis_thermal_guard: bool,
        is_tau_reliable: bool,
        learn_ok_count_a: int
    ) -> float:
        """
        Main PID Calculation logic.
        Updates self.integral, self.u_pi, self.u_cmd.
        Returns u_cmd (clamped 0-1).
        """
        self.last_error = error
        self.last_error_p = error_p
        
        i_max = 2.0 / max(ki, KI_MIN)
        u_pi = 0.0
        
        if in_deadband:
            # Deadband Logic
            self.last_i_mode = "I:FREEZE(deadband)"
            
            # Micro-leak
            cycle_ref = max(float(cycle_min), 1.0)
            leak_factor = INTEGRAL_DEADBAND_MICROLEAK ** (dt_min / cycle_ref)
            self.integral *= leak_factor
            self.integral = clamp(self.integral, -i_max, i_max)
            
            # DB+ Hold Power
            if (hvac_mode != VThermHvacMode_COOL and error > 0.0 and 
                is_tau_reliable and learn_ok_count_a >= 10):
                u_hold = clamp(DEADBAND_PLUS_MIN_U, 0.0, DEADBAND_PLUS_MAX_U)
                u_total = max(u_ff, u_hold)
                u_pi = u_total - u_ff
        else:
            if integrator_hold:
                u_pi = kp * error_p + ki * self.integral
                self.last_i_mode = "I:HOLD"
                
                # Overshoot bleeding
                if hvac_mode != VThermHvacMode_COOL and current_temp >= (target_temp - OVERSHOOT_I_CLAMP_EPS_C):
                     if self.integral > 0.0:
                        leak_eff = INTEGRAL_LEAK ** (dt_min / max(1e-9, float(cycle_min)))
                        self.integral *= leak_eff
                        self.last_i_mode = "I:BLEED(hold_ovr)"
            else:
                # Preview
                u_pi_pre = kp * error_p + ki * self.integral
                u_raw_pre = u_ff + u_pi_pre
                
                if u_raw_pre > 1.0: sat = "SAT_HI"
                elif u_raw_pre < 0.0: sat = "SAT_LO"
                else: sat = "NO_SAT"
                self.last_sat = sat
                
                # Conditional Integration
                if (sat == "SAT_HI" and error > 0) or (sat == "SAT_LO" and error < 0):
                    self.last_i_mode = f"I:SKIP({sat})"
                    u_pi = u_pi_pre
                else:
                    d_integral = error * dt_min
                    self.last_i_mode = "I:RUN"
                    
                    # Overshoot Clamping
                    if hvac_mode != VThermHvacMode_COOL and current_temp >= (target_temp - OVERSHOOT_I_CLAMP_EPS_C):
                        if d_integral > 0.0:
                            d_integral = 0.0
                            self.last_i_mode = "I:CLAMP(near_ovr)"
                            
                    # Thermal Guard
                    if hysteresis_thermal_guard:
                        if current_temp > target_temp:
                            if d_integral > 0:
                                d_integral = 0.0
                                self.last_i_mode = "I:GUARD(freeze)"
                            else:
                                self.last_i_mode = "I:GUARD(drop)"
                    
                    self.integral += d_integral
                    self.integral = clamp(self.integral, -i_max, i_max)
                    u_pi = kp * error_p + ki * self.integral

        self.u_pi = u_pi
        self.u_ff = u_ff
        
        u_raw = u_ff + u_pi
        self.u_cmd = clamp(u_raw, 0.0, 1.0)
        
        return self.u_cmd

    def update_anti_windup(
        self,
        u_limited: float,
        u_applied: float,
        dt_min: float,
        ki: float,
        kp: float,
        error_p: float,
        integrator_hold: bool,
        in_deadband: bool,
        max_on_percent: float | None,
        current_temp: float,
        target_temp: float,
        hysteresis_thermal_guard: bool
    ):
        """Back-calculation Anti-Windup."""
        self.u_applied = u_applied
        self.u_limited = u_limited
        
        if (not integrator_hold) and (ki > KI_MIN) and (not in_deadband) and (not str(self.last_i_mode).startswith("I:CLAMP")):
            u_model = self.u_ff + (kp * error_p + ki * self.integral)
            u_aw_ref = u_limited
            
            if max_on_percent is not None and self.u_cmd > max_on_percent + 1e-9:
                u_aw_ref = self.u_cmd
            
            # du = u_applied - u_aw_ref  <-- Logic in original was du = u_aw_ref - u_model ?
            # Let's check original... 
            # Original: du = u_aw_ref - u_model
            du = u_aw_ref - u_model
            self.last_aw_du = du
            
            dt_sec = dt_min * 60.0
            beta = clamp(dt_sec / max(AW_TRACK_TAU_S, dt_sec), 0.0, 1.0)
            
            d_integral = beta * (du / ki)
            max_di = AW_TRACK_MAX_DELTA_I * max(dt_min, 0.0)
            d_integral = clamp(d_integral, -max_di, max_di)
            
            if hysteresis_thermal_guard and (current_temp > target_temp):
                d_integral = min(0.0, d_integral)
                
            i_max = 2.0 / max(ki, KI_MIN)
            self.integral += d_integral
            self.integral = clamp(self.integral, -i_max, i_max)
        else:
            self.last_aw_du = 0.0
        
        # Prepare for next cycle
        self.u_prev = u_applied
