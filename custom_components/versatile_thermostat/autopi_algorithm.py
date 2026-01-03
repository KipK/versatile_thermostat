"""
AutoPI Algorithm - Simplified auto-adaptive PI controller for Versatile Thermostat.

This module implements a PI controller with:
- Online model learning (RLS): dT_int (°C/min) ≈ a * u - b * (T_int - T_ext)
- Feed-forward compensation for thermal losses
- Overshoot protection with FF scaling and integral unwinding
- Anti-windup protection

The output is a power command between 0 and 1 (0-100%), to be applied as duty-cycle.
"""
import logging
import time
from dataclasses import dataclass, field
from typing import Dict, Any, Optional

from .vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL, VThermHvacMode_OFF, VThermHvacMode_HEAT

_LOGGER = logging.getLogger(__name__)

# RLS parameters
A_INIT = 0.0005
B_INIT = 0.0010
A_MIN, A_MAX = 1e-6, 0.05
B_MIN, B_MAX = 1e-6, 0.05
RLS_LAMBDA = 0.995  # Forgetting factor

# PI tuning parameters
KP_MIN, KP_MAX = 0.2, 2.0
KI_MIN, KI_MAX = 0.001, 0.2

# Overshoot scaling: FF scales to 0 when error reaches this value
OVERSHOOT_SCALE_RANGE = 0.5  # °C


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value to the interval [lo, hi]."""
    return max(lo, min(hi, x))


@dataclass
class RLS:
    """
    Recursive Least Squares estimator for thermal model parameters.
    
    Model: dT_int = a * u - b * (T_int - T_ext)
    - a: heating efficiency (°C/min at 100% power)
    - b: thermal losses (1/min)
    """
    theta_a: float = A_INIT
    theta_b: float = B_INIT
    P11: float = 1000.0  # Covariance diagonal (uncertainty on a)
    P22: float = 1000.0  # Covariance diagonal (uncertainty on b)
    lam: float = RLS_LAMBDA

    def update(self, u: float, t_int: float, t_ext: float, dt_int: float) -> None:
        """
        Update model parameters from an observation.
        
        Args:
            u: Applied power command [0, 1]
            t_int: Indoor temperature at previous step (°C)
            t_ext: Outdoor temperature (°C)
            dt_int: Observed temperature change (°C/min)
        """
        # Regression vector: phi = [u, -(T_int - T_ext)]
        phi1 = float(u)
        phi2 = float(-(t_int - t_ext))

        # Prediction and innovation
        y_hat = self.theta_a * phi1 + self.theta_b * phi2
        err = dt_int - y_hat

        # RLS gain (diagonal approximation)
        denom = self.lam + self.P11 * phi1 * phi1 + self.P22 * phi2 * phi2
        if denom <= 1e-12:
            return

        k1 = (self.P11 * phi1) / denom
        k2 = (self.P22 * phi2) / denom

        # Parameter update
        self.theta_a += k1 * err
        self.theta_b += k2 * err

        # Apply bounds to prevent drift
        self.theta_a = clamp(self.theta_a, A_MIN, A_MAX)
        self.theta_b = clamp(self.theta_b, B_MIN, B_MAX)

        # Covariance update
        self.P11 = (self.P11 - k1 * phi1 * self.P11) / self.lam
        self.P22 = (self.P22 - k2 * phi2 * self.P22) / self.lam


class AutoPI:
    """
    Auto-adaptive PI controller with feed-forward compensation.
    
    Features:
    - Online model learning via RLS
    - Feed-forward based on outdoor temperature
    - Overshoot protection (FF scaling + integral unwinding)
    - Anti-windup protection
    """

    def __init__(
        self,
        cycle_min: int,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: float = None,
        deadband_c: float = 0.05,
        min_useful: float = 0.05,
        max_step_per_min: float = 0.10,
        aggressiveness: float = 0.5,
        saved_state: Optional[Dict[str, Any]] = None
    ):
        self._name = name
        self._cycle_min = cycle_min
        self._minimal_activation_delay = minimal_activation_delay
        self._minimal_deactivation_delay = minimal_deactivation_delay
        self._max_on_percent = max_on_percent

        # Tuning parameters
        self.deadband_c = deadband_c
        self.min_useful = min_useful
        self.max_step_per_min = max_step_per_min
        self.aggressiveness = aggressiveness

        # RLS model estimator
        self.rls = RLS()

        # PI state
        self.integral: float = 0.0
        self.u_prev: float = 0.0
        self._prev_error: Optional[float] = None
        
        # Current gains
        self.Kp: float = 0.8
        self.Ki: float = 0.05

        # Outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0
        
        # Temperature tracking for learning
        self._last_t_in: Optional[float] = None
        
        # Last calculated values for diagnostics
        self._last_u_ff: float = 0.0
        self._last_error: float = 0.0

        if saved_state:
            self.load_state(saved_state)

        _LOGGER.debug("%s - AutoPI initialized", self._name)

    @property
    def a(self) -> float:
        """Heating efficiency parameter."""
        return self.rls.theta_a

    @property
    def b(self) -> float:
        """Thermal losses parameter."""
        return self.rls.theta_b

    @property
    def on_percent(self) -> float:
        """Current power command [0, 1]."""
        return self._on_percent

    @property
    def calculated_on_percent(self) -> float:
        """Alias for on_percent."""
        return self._on_percent

    @property
    def on_time_sec(self) -> int:
        """ON time in seconds for duty-cycle."""
        return self._on_time_sec

    @property
    def off_time_sec(self) -> int:
        """OFF time in seconds for duty-cycle."""
        return self._off_time_sec

    @property
    def integral_error(self) -> float:
        """Current integral accumulator value."""
        return self.integral

    @property
    def prev_error(self) -> float:
        """Previous error value."""
        return self._prev_error if self._prev_error is not None else 0.0

    @property
    def u_ff(self) -> float:
        """Last feed-forward value."""
        return self._last_u_ff

    @property
    def deadtime_s(self) -> int:
        """Dead time (not used in simplified version, kept for API)."""
        return 0

    def reset_learning(self) -> None:
        """Reset all learned parameters to defaults."""
        self.rls = RLS()
        self.integral = 0.0
        self.u_prev = 0.0
        self._prev_error = None
        self.Kp = 0.8
        self.Ki = 0.05
        self._last_t_in = None
        _LOGGER.info("%s - AutoPI learning reset to defaults", self._name)

    def load_state(self, state: Dict[str, Any]) -> None:
        """Load persistent state."""
        if not state:
            return
        self.rls.theta_a = state.get('a', A_INIT)
        self.rls.theta_b = state.get('b', B_INIT)
        self.integral = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        self.Kp = state.get('Kp', 0.8)
        self.Ki = state.get('Ki', 0.05)
        _LOGGER.debug(
            "%s - AutoPI state loaded: a=%.6f, b=%.6f",
            self._name, self.rls.theta_a, self.rls.theta_b
        )

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence."""
        return {
            'a': self.rls.theta_a,
            'b': self.rls.theta_b,
            'integral': self.integral,
            'u_prev': self.u_prev,
            'Kp': self.Kp,
            'Ki': self.Ki,
        }

    def notify_setpoint_change(
        self, t_set_old: float, t_set_new: float, t_in: float, is_major: bool = False
    ) -> None:
        """
        Handle setpoint changes.
        
        Args:
            t_set_old: Previous setpoint
            t_set_new: New setpoint
            t_in: Current temperature
            is_major: If True, reset integral completely
        """
        if is_major:
            self.integral = 0.0
            _LOGGER.debug("%s - Major setpoint change, integral reset", self._name)
        else:
            # Bumpless transfer
            if self.Ki > 0:
                e_old = t_set_old - t_in
                e_new = t_set_new - t_in
                d_integral = (self.Kp / self.Ki) * (e_old - e_new)
                d_integral = clamp(d_integral, -10.0, 10.0)
                self.integral += d_integral
                _LOGGER.debug("%s - Bumpless transfer applied", self._name)

    def notify_actuator_state(self, is_on: bool, u_cycle: float = 0.0) -> None:
        """Deprecated: kept for API compatibility."""
        pass

    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
    ) -> None:
        """
        Update the thermal model with observed data.
        
        Called once per cycle with data from the previous cycle.
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            return

        # Temperature change over one cycle (assumed 1 minute for simplicity)
        dt_int = current_temp - previous_temp

        t_ext = ext_current_temp if ext_current_temp is not None else current_temp

        self.rls.update(
            u=previous_power,
            t_int=previous_temp,
            t_ext=t_ext,
            dt_int=dt_int
        )

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None,
        hvac_mode: VThermHvacMode,
    ) -> None:
        """
        Calculate the power command.
        
        Args:
            target_temp: Setpoint temperature (°C)
            current_temp: Current indoor temperature (°C)
            ext_current_temp: Current outdoor temperature (°C)
            slope: Temperature slope (unused, kept for API)
            hvac_mode: Current HVAC mode
        """
        if target_temp is None or current_temp is None:
            self._on_percent = 0
            self._calculate_times()
            return

        if hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0
            self.u_prev = 0
            self._calculate_times()
            return

        # Get model parameters
        a = self.rls.theta_a
        b = self.rls.theta_b

        # Adapt Kp/Ki based on thermal time constant
        tau_min = 1.0 / max(b, 1e-6)  # Time constant in minutes
        kp_base = clamp(0.3 + 0.5 * (tau_min / 100.0), KP_MIN, KP_MAX)
        ki_base = clamp(kp_base / max(tau_min, 5.0), KI_MIN, KI_MAX)

        # Apply aggressiveness scaling: lower = more aggressive (higher gains)
        # aggressiveness=0.5 → scale=1.0, aggressiveness=1.0 → scale=0.5
        gain_scale = 0.5 / max(self.aggressiveness, 0.1)
        self.Kp = clamp(kp_base * gain_scale, KP_MIN, KP_MAX)
        self.Ki = clamp(ki_base * gain_scale, KI_MIN, KI_MAX)

        # Calculate error
        e = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            e = -e
        self._last_error = e

        # Feed-forward with overshoot scaling
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        k_ff = clamp(b / max(a, 1e-6), 0.0, 5.0)
        u_ff_base = clamp(k_ff * (target_temp - t_ext), 0.0, 1.0)

        if e < 0:
            # In overshoot: scale down FF proportionally
            # FF = 0 when error <= -OVERSHOOT_SCALE_RANGE
            scale = clamp(1.0 + (e / OVERSHOOT_SCALE_RANGE), 0.0, 1.0)
            u_ff = u_ff_base * scale
        else:
            u_ff = u_ff_base
        self._last_u_ff = u_ff

        # PI control with deadband and overshoot unwinding
        if abs(e) < self.deadband_c:
            # In deadband: decay integral slowly
            self.integral *= 0.98
            u_pi = 0.0
        else:
            # Overshoot unwinding: reduce integral on sign change
            if self._prev_error is not None:
                if (self._prev_error > 0 and e < 0) or (self._prev_error < 0 and e > 0):
                    self.integral *= 0.5
                    _LOGGER.debug("%s - Overshoot unwinding: integral *= 0.5", self._name)
            
            # Continuous reduction while in overshoot
            if e < 0:
                self.integral *= 0.9

            self.integral += e
            u_pi = self.Kp * e + self.Ki * self.integral

        self._prev_error = e

        # Combine FF and PI
        u_raw = u_ff + u_pi

        # Saturation with anti-windup
        u = clamp(u_raw, 0.0, 1.0)
        if u != u_raw:
            # Reduce integral on saturation
            self.integral *= 0.9

        # Rate limiting
        max_step = self.max_step_per_min
        u = clamp(u, self.u_prev - max_step, self.u_prev + max_step)

        # Minimum useful power
        if 0.0 < u < self.min_useful:
            u = 0.0

        # Apply max_on_percent limit
        if self._max_on_percent is not None and u > self._max_on_percent:
            u = self._max_on_percent

        self.u_prev = u
        self._on_percent = u

        self._calculate_times()

    def _calculate_times(self) -> None:
        """Calculate ON/OFF times from power percentage."""
        self._on_time_sec = int(self._on_percent * self._cycle_min * 60)
        if self._on_time_sec < self._minimal_activation_delay:
            self._on_time_sec = 0
        self._off_time_sec = int(self._cycle_min * 60 - self._on_time_sec)
        if self._off_time_sec < self._minimal_deactivation_delay:
            self._on_time_sec = int(self._cycle_min * 60)
            self._off_time_sec = 0

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information for attributes."""
        return {
            "a": self.rls.theta_a,
            "b": self.rls.theta_b,
            "Kp": self.Kp,
            "Ki": self.Ki,
            "integral_error": self.integral,
            "error": self._last_error,
            "u_ff": self._last_u_ff,
            "on_percent": self._on_percent,
        }
