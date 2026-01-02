
"""
This module implements the AutoPI algorithm (Recursive Least Squares + PI Controller)
for Versatile Thermostat.

Based on auto-adaptive PI regulation algorithm:
- Model learned online (RLS): dT_int (°C/min) ≈ a * u - b * (T_int - T_ext)
- Control: u = clamp(u_ff + u_PI, 0..1)
  - u_ff compensates for thermal losses (feed-forward based on T_ext)
  - u_PI corrects error (T_set - T_int) with PI + safeguards
"""
import logging
from typing import Dict, Any, Optional

from .vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL, VThermHvacMode_OFF, VThermHvacMode_HEAT
from .const import PROPORTIONAL_FUNCTION_AUTO_PI

_LOGGER = logging.getLogger(__name__)


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value to the interval [lo, hi]."""
    return max(lo, min(hi, x))


class RLS2:
    """
    RLS (Recursive Least Squares) 2-parameter estimator, simplified version (diagonal covariance).

    Thermal model (discrete, 1 minute step):
    ----------------------------------------
    dtint = T_int[k] - T_int[k-1]  (°C/min)
    dtint ≈ a * u  -  b * (T_int - T_ext)

    Parameters
    ----------
    theta_a : float
        Current estimate of 'a' (°C/min at 100%).
        Conservative initial value (e.g.: 0.0005).
    theta_b : float
        Current estimate of 'b' (1/min).
        Conservative initial value (e.g.: 0.0010).
    P11, P22 : float
        Diagonal elements of covariance matrix (initial uncertainty).
        Higher values = faster learning at start.
    lam : float
        Forgetting factor (0<lam<=1). Close to 1 => slow forgetting (learning over days).
    """

    def __init__(self, lmbda: float = 0.995):
        self.lmbda = lmbda  # Forgetting factor

        # Initialize parameters with conservative values (as per spec)
        self.theta_a: float = 0.0005  # °C/min at 100%
        self.theta_b: float = 0.0010  # 1/min

        # Covariance matrix P initialized with large values (high uncertainty)
        self.P11: float = 1000.0
        self.P22: float = 1000.0

    def update(self, u: float, t_int: float, t_ext: float, dt_int: float) -> None:
        """
        Update (a, b) from an observation.

        Parameters
        ----------
        u : float
            Command applied previously, normalized in [0, 1].
        t_int : float
            Indoor temperature (°C) at previous step (k-1).
        t_ext : float
            Outdoor temperature (°C) at current/previous step.
        dt_int : float
            Observed indoor temperature variation per minute: T_int[k] - T_int[k-1] (°C/min).
        """
        # Regression vector: phi = [u, -(Tint-Text)]
        phi1 = float(u)
        phi2 = float(-(t_int - t_ext))

        # Prediction and innovation
        y_hat = self.theta_a * phi1 + self.theta_b * phi2
        err = dt_int - y_hat

        # RLS gain (diagonal)
        denom = self.lmbda + self.P11 * phi1 * phi1 + self.P22 * phi2 * phi2
        if denom <= 1e-12:
            return

        k1 = (self.P11 * phi1) / denom
        k2 = (self.P22 * phi2) / denom

        # Update parameters
        self.theta_a += k1 * err
        self.theta_b += k2 * err

        # Physical bounds (anti-drift) - as per specification
        self.theta_a = clamp(self.theta_a, 1e-6, 0.05)  # °C/min at 100%
        self.theta_b = clamp(self.theta_b, 1e-6, 0.05)  # 1/min

        # Update covariances (diagonal)
        self.P11 = (self.P11 - k1 * phi1 * self.P11) / self.lmbda
        self.P22 = (self.P22 - k2 * phi2 * self.P22) / self.lmbda

    def reset(self):
        """Reset parameters to initial conservative values."""
        self.theta_a = 0.0005
        self.theta_b = 0.0010
        self.P11 = 1000.0
        self.P22 = 1000.0


class AutoPI:
    """
    Auto-adaptive PI controller + external feed-forward.

    Output
    ------
    - u = [0, 1] (to convert to 0..100%)
    - diagnostic dictionary: a, b, Kp, Ki, u_ff, e

    Main Parameters
    ---------------
    cycle_min : int
        Target duty-cycle window (e.g.: 10 min). Mainly for documentation.
    deadband_c : float
        Deadband around setpoint (°C). Within deadband, PI action is frozen.
    min_useful : float
        Minimum useful command (0..1). Below => 0.
    max_step_per_min : float
        Command variation limit per minute (anti-oscillation), as fraction (0.10 = 10%/min).
    """

    def __init__(
        self,
        cycle_min: int,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: float = None,
        # Tuning parameters
        deadband_c: float = 0.05,
        min_useful: float = 0.05,
        max_step_per_min: float = 0.10,
        aggressiveness: float = 0.5,
        # Persistence data
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
        self.aggressiveness = aggressiveness  # Lower = more aggressive (faster response)

        # RLS Estimator
        self.rls = RLS2()

        # PI State
        self.integral: float = 0.0
        self.u_prev: float = 0.0  # Previous control output
        self._prev_error: float = 0.0  # Previous error for diagnostics

        # Calculated outputs
        self._on_percent: float = 0.0
        self._on_time_sec: int = 0
        self._off_time_sec: int = 0

        # Current gains (updated by learning) - initial values from spec
        self.Kp: float = 0.8
        self.Ki: float = 0.05

        # Cached model parameters
        self.a: float = 0.0
        self.b: float = 0.0

        if saved_state:
            self.load_state(saved_state)


        _LOGGER.debug("%s - AutoPI initialized", self._name)

    def reset_learning(self):
        """Reset learning to default values."""
        self.rls.reset()
        self.integral = 0.0
        self.u_prev = 0.0
        self._prev_error = 0.0
        self.Kp = 0.8
        self.Ki = 0.05
        self.a = 0.0
        self.b = 0.0
        _LOGGER.info("%s - AutoPI learning reset to defaults", self._name)

    def load_state(self, state: Dict[str, Any]):
        """Load persistent state"""
        if not state:
            return
        self.rls.theta_a = state.get('theta_a', 0.0005)
        self.rls.theta_b = state.get('theta_b', 0.0010)
        self.rls.P11 = state.get('P11', 1000.0)
        self.rls.P22 = state.get('P22', 1000.0)
        self.integral = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        self.Kp = state.get('Kp', 0.8)
        self.Ki = state.get('Ki', 0.05)
        _LOGGER.debug("%s - AutoPI state loaded: a=%.4f, b=%.4f", self._name, self.rls.theta_a, self.rls.theta_b)

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence"""
        return {
            'theta_a': self.rls.theta_a,
            'theta_b': self.rls.theta_b,
            'P11': self.rls.P11,
            'P22': self.rls.P22,
            'integral': self.integral,
            'u_prev': self.u_prev,
            'Kp': self.Kp,
            'Ki': self.Ki,
        }

    def update_learning(
        self,
        current_temp: float,
        ext_current_temp: float,
        previous_temp: float,
        previous_power: float,
        hvac_mode: VThermHvacMode,
    ):
        """
        Update RLS model and PI gains.
        Should be called once per cycle, using data from the PREVIOUS cycle.

        The model expects temperature change in °C/min, so we normalize by cycle duration.
        """
        if previous_temp is None or previous_power is None:
            return

        if hvac_mode != VThermHvacMode_HEAT:
            # Only learn in HEAT mode for now
            return

        # Calculate temperature change per minute (normalize by cycle duration)
        dt_int_total = current_temp - previous_temp  # Total change over the cycle
        dt_int_per_min = dt_int_total / self._cycle_min  # Change per minute

        # Update RLS with data from previous cycle
        t_ext = ext_current_temp if ext_current_temp else current_temp
        self.rls.update(previous_power, previous_temp, t_ext, dt_int_per_min)

        # Update PI gains based on learned parameters
        self._update_gains()

    def _update_gains(self):
        """
        Calculate PI gains based on current RLS parameters.

        Uses a prudent heuristic approach:
        - tau ≈ 1/b (in minutes). If tau is large => slow system, moderate Kp, low Ki.
        - aggressiveness scales the gains: lower = more aggressive (faster response)
        """
        a = self.rls.theta_a
        b = self.rls.theta_b

        # Cache for diagnostics
        self.a = a
        self.b = b

        # Time constant in minutes (as per specification)
        tau_min = 1.0 / max(b, 1e-6)

        # Base Kp adaptation (prudent heuristic from spec)
        base_kp = clamp(0.3 + 0.5 * (tau_min / 100.0), 0.2, 2.0)

        # Scale by aggressiveness: lower aggressiveness = higher gains
        # aggressiveness=0.5 gives base gains, aggressiveness=1.0 gives half, aggressiveness=0.25 gives double
        aggr_factor = 1.0 / max(self.aggressiveness * 2.0, 0.2)
        self.Kp = clamp(base_kp * aggr_factor, 0.2, 4.0)

        # Ki adaptation (prudent heuristic from spec, also scaled)
        base_ki = clamp(base_kp / max(tau_min, 5.0), 0.001, 0.2)
        self.Ki = clamp(base_ki * aggr_factor, 0.001, 0.4)

        _LOGGER.debug(
            "%s - AutoPI Gains Updated: a=%.5f, b=%.5f, tau=%.1f min, Kp=%.2f, Ki=%.4f, aggr=%.2f",
            self._name, a, b, tau_min, self.Kp, self.Ki, self.aggressiveness
        )

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None,  # Not used in PI but kept for interface compatibility
        hvac_mode: VThermHvacMode,
    ):
        """
        Calculate the heating percentage (Control Loop).
        """
        if target_temp is None or current_temp is None:
            self._on_percent = 0
            return

        if hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0
            self.u_prev = 0
            return

        # Calculate error
        e = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            e = -e

        # Get model parameters
        a = self.rls.theta_a
        b = self.rls.theta_b

        # Feed-forward: external compensation (steady state: a*u ≈ b*(T_set - T_ext))
        k_ff = clamp(b / max(a, 1e-6), 0.0, 5.0)
        u_ff = 0.0
        if ext_current_temp is not None:
            u_ff = clamp(k_ff * (target_temp - ext_current_temp), 0.0, 1.0)

        # PI with deadband
        dt_min = 1.0  # Integration step is 1 minute per cycle
        if abs(e) < self.deadband_c:
            # Soft leakage of integral to avoid drift
            self.integral *= 0.98
            u_pi = 0.0
        else:
            self.integral += e * dt_min
            u_pi = self.Kp * e + self.Ki * self.integral

        u_raw = u_ff + u_pi

        # Saturation + anti-windup
        u = clamp(u_raw, 0.0, 1.0)
        if u != u_raw:
            # If saturated, reduce integral slightly
            self.integral *= 0.9

        # Rate limiting (ramp)
        u = clamp(u, self.u_prev - self.max_step_per_min, self.u_prev + self.max_step_per_min)

        # Minimum useful command
        if 0.0 < u < self.min_useful:
            u = 0.0

        self.u_prev = u
        self._on_percent = u
        self._prev_error = e  # Store for diagnostics

        # Calculate times
        self._calculate_times()

        _LOGGER.debug("%s - AutoPI Calculate: T=%.2f, Tgt=%.2f, Ext=%s", self._name, current_temp, target_temp, ext_current_temp)
        _LOGGER.debug("%s - Terms: e=%.2f, P=%.2f, I=%.2f, FF=%.2f -> u=%.2f", self._name, e, self.Kp*e, self.Ki*self.integral, u_ff, u)

    def _calculate_times(self):
        """Calculate on/off times from percent"""
        self._on_time_sec = int(self._on_percent * self._cycle_min * 60)

        # Apply min durations
        if self._on_time_sec < self._minimal_activation_delay:
            self._on_time_sec = 0

        self._off_time_sec = int(self._cycle_min * 60 - self._on_time_sec)

        if self._off_time_sec < self._minimal_deactivation_delay:
            self._on_time_sec = int(self._cycle_min * 60)
            self._off_time_sec = 0

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostic information for attributes"""
        return {
            "a": round(self.a, 6),
            "b": round(self.b, 6),
            "Kp": round(self.Kp, 4),
            "Ki": round(self.Ki, 4),
            "integral": round(self.integral, 4),
            "u_prev": round(self.u_prev, 4),
        }

    @property
    def on_percent(self):
        return self._on_percent

    @property
    def calculated_on_percent(self):
        """Required compatibility property"""
        return self._on_percent

    @property
    def on_time_sec(self):
        return self._on_time_sec

    @property
    def off_time_sec(self):
        return self._off_time_sec

    @property
    def integral_error(self):
        """Alias for backward compatibility"""
        return self.integral

    @property
    def prev_error(self):
        """Return the last error value for diagnostics"""
        return getattr(self, '_prev_error', 0.0)
