"""
AutoPI Algorithm - Simplified auto-adaptive PI controller for Versatile Thermostat.

This module implements a PI controller with:
- Online model learning (RLS): dT_int (°C/min) ≈ a * u - b * (T_int - T_ext)
- SIMC-based PI tuning with dead time estimation
- Feed-forward compensation for thermal losses
- Gain scheduling near setpoint for stability
- Overshoot protection with integral unwinding
- Anti-windup protection

The output is a power command between 0 and 1 (0-100%), to be applied as duty-cycle.
"""
import logging
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

# PI tuning bounds
KP_MIN, KP_MAX = 0.1, 2.0
KI_MIN, KI_MAX = 0.001, 0.15

# Gain scheduling: reduce gains when error is below this threshold
GAIN_SCHEDULE_THRESHOLD = 1.5  # °C



# Anti-windup: maximum allowed integral value
MAX_INTEGRAL = 50.0


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
    - SIMC-based PI tuning
    - Gain scheduling near setpoint
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
        
        # Current gains (will be recalculated)
        self.Kp: float = 0.5
        self.Ki: float = 0.02

        # Outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0
        
        # Diagnostics
        self._last_u_ff: float = 0.0
        self._last_error: float = 0.0
        self._effective_kp: float = 0.0
        self._effective_ki: float = 0.0

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

    def reset_learning(self) -> None:
        """Reset all learned parameters to defaults."""
        self.rls = RLS()
        self.integral = 0.0
        self.u_prev = 0.0
        self._prev_error = None
        self.Kp = 0.5
        self.Ki = 0.02
        _LOGGER.info("%s - AutoPI learning reset to defaults", self._name)

    def load_state(self, state: Dict[str, Any]) -> None:
        """Load persistent state."""
        if not state:
            return
        self.rls.theta_a = state.get('a', A_INIT)
        self.rls.theta_b = state.get('b', B_INIT)
        self.rls.P11 = state.get('P11', 1000.0)
        self.rls.P22 = state.get('P22', 1000.0)
        self.integral = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        self.Kp = state.get('Kp', 0.5)
        self.Ki = state.get('Ki', 0.02)
        _LOGGER.debug(
            "%s - AutoPI state loaded: a=%.6f, b=%.6f",
            self._name, self.rls.theta_a, self.rls.theta_b
        )

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence."""
        return {
            'a': self.rls.theta_a,
            'b': self.rls.theta_b,
            'P11': self.rls.P11,
            'P22': self.rls.P22,
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

    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
        cycle_dt: float = None,
    ) -> None:
        """
        Update the thermal model with observed data.
        
        Called once per cycle with data from the previous cycle.
        
        Args:
            current_temp: Current indoor temperature (°C)
            ext_current_temp: Current outdoor temperature (°C)
            previous_temp: Indoor temperature at start of cycle (°C)
            previous_power: Average power applied during cycle [0, 1]
            hvac_mode: Current HVAC mode
            cycle_dt: Cycle duration in minutes (defaults to self._cycle_min)
        """
        if previous_temp is None or previous_power is None or current_temp is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            return

        # Use provided cycle duration or default to configured cycle
        dt_minutes = cycle_dt if cycle_dt is not None else self._cycle_min
        if dt_minutes <= 0:
            return

        # Temperature change rate in °C/min
        dt_int = (current_temp - previous_temp) / dt_minutes

        t_ext = ext_current_temp if ext_current_temp is not None else current_temp

        self.rls.update(
            u=previous_power,
            t_int=previous_temp,
            t_ext=t_ext,
            dt_int=dt_int
        )

    def _calculate_simc_gains(self, tau_min: float, a: float) -> tuple:
        """
        Calculate PI gains using SIMC (Skogestad IMC) method.
        
        For a first-order plus dead-time system:
        - Kp = tau / (K * (tau_c + theta))
        - Ti = min(tau, 4 * (tau_c + theta))
        - Ki = Kp / Ti
        
        Args:
            tau_min: Time constant in minutes
            a: Process gain (heating efficiency)
            
        Returns:
            Tuple of (Kp, Ki)
        """
        # Estimate dead time as fraction of tau (typical for thermal systems)
        theta = min(tau_min * 0.25, 5.0)
        
        # Closed-loop time constant: larger = more robust, less aggressive
        # aggressiveness=0.5 → tau_c=theta (SIMC default for "tight" control)
        # aggressiveness=1.0 → tau_c=2*theta (more robust)
        tau_c = theta * (0.5 + self.aggressiveness)
        
        # SIMC tuning formulas
        # K is the process gain: for our model, it's 'a' (°C/min per unit power)
        K = max(a, 1e-6)
        
        kp = tau_min / (K * (tau_c + theta))
        ti = min(tau_min, 4.0 * (tau_c + theta))
        ki = kp / max(ti, 1.0)
        
        # Apply bounds
        kp = clamp(kp, KP_MIN, KP_MAX)
        ki = clamp(ki, KI_MIN, KI_MAX)
        
        return kp, ki

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

        # Thermal time constant (minutes)
        tau_min = 1.0 / max(b, 1e-6)

        # Calculate base gains using SIMC method
        self.Kp, self.Ki = self._calculate_simc_gains(tau_min, a)

        # Calculate error
        e = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            e = -e
        self._last_error = e

        # Gain scheduling: reduce gains when approaching setpoint
        # This prevents aggressive response when error is small
        if abs(e) < GAIN_SCHEDULE_THRESHOLD:
            # Smooth reduction: 50% to 100% of base gains
            schedule_factor = 0.5 + 0.5 * (abs(e) / GAIN_SCHEDULE_THRESHOLD)
        else:
            schedule_factor = 1.0
        
        effective_kp = self.Kp * schedule_factor
        effective_ki = self.Ki * schedule_factor
        self._effective_kp = effective_kp
        self._effective_ki = effective_ki

        # Model confidence for feedforward scaling
        confidence_a = max(0.0, min(100.0, 100.0 * (1.0 - self.rls.P11 / 1000.0)))
        confidence_b = max(0.0, min(100.0, 100.0 * (1.0 - self.rls.P22 / 1000.0)))
        model_confidence = (confidence_a + confidence_b) / 200.0  # 0 to 1

        # Feed-forward calculation
        t_ext = ext_current_temp if ext_current_temp is not None else current_temp
        k_ff = clamp(b / max(a, 1e-6), 0.0, 3.0)
        u_ff_base = clamp(k_ff * (target_temp - t_ext), 0.0, 1.0)

        # Limit feedforward during learning phase (when model is unreliable)
        # At 0% confidence: use 30% of FF, at 100%: use full FF
        ff_confidence_factor = 0.3 + 0.7 * model_confidence
        u_ff_base *= ff_confidence_factor

        u_ff = u_ff_base
        self._last_u_ff = u_ff

        # PI control with deadband and improved anti-windup
        if abs(e) < self.deadband_c:
            # In deadband: decay integral slowly
            self.integral *= 0.95
            u_pi = 0.0
        else:
            # Improved overshoot handling: proportional decay instead of multiplicative
            if e < 0:  # Overshoot (temp above setpoint)
                # Decay integral proportionally to overshoot magnitude
                decay_amount = 0.5 * abs(e) * self._cycle_min
                self.integral = max(0.0, self.integral - decay_amount)
            else:
                # Normal operation: accumulate error
                self.integral += e * self._cycle_min
            
            # Clamp integral to prevent excessive windup
            self.integral = clamp(self.integral, -MAX_INTEGRAL, MAX_INTEGRAL)
            
            u_pi = effective_kp * e + effective_ki * self.integral

        self._prev_error = e

        # Combine FF and PI
        u_raw = u_ff + u_pi

        # Saturation with anti-windup
        u = clamp(u_raw, 0.0, 1.0)
        if u != u_raw and u_raw > 1.0:
            # Output saturated high: reduce integral to prevent windup
            excess = u_raw - 1.0
            self.integral = max(0.0, self.integral - excess / max(effective_ki, 0.001))

        # Adaptive rate limiting: stricter near setpoint
        if abs(e) < 0.5:
            max_step = self.max_step_per_min * 0.3  # Much slower near setpoint
        elif abs(e) < 1.0:
            max_step = self.max_step_per_min * 0.5
        else:
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
        # Thermal time constant (minutes)
        tau_min = 1.0 / max(self.rls.theta_b, 1e-6)

        # Model confidence: 0-100% based on covariance reduction
        confidence_a = max(0.0, min(100.0, 100.0 * (1.0 - self.rls.P11 / 1000.0)))
        confidence_b = max(0.0, min(100.0, 100.0 * (1.0 - self.rls.P22 / 1000.0)))
        model_confidence = (confidence_a + confidence_b) / 2.0

        return {
            "a": self.rls.theta_a,
            "b": self.rls.theta_b,
            "tau_min": round(tau_min, 1),
            "confidence_a": round(confidence_a, 1),
            "confidence_b": round(confidence_b, 1),
            "model_confidence": round(model_confidence, 1),
            "Kp": round(self.Kp, 3),
            "Ki": round(self.Ki, 4),
            "effective_Kp": round(self._effective_kp, 3),
            "effective_Ki": round(self._effective_ki, 4),
            "integral_error": round(self.integral, 2),
            "error": round(self._last_error, 2),
            "u_ff": round(self._last_u_ff, 3),
            "on_percent": round(self._on_percent, 3),
        }
