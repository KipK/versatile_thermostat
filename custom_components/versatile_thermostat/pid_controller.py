
"""
This module implements the AutoPI algorithm (Recursive Least Squares + PI Controller)
for Versatile Thermostat.
"""
import logging
import math
from datetime import datetime
from typing import Dict, Any, Optional

from .vtherm_hvac_mode import VThermHvacMode, VThermHvacMode_COOL, VThermHvacMode_OFF, VThermHvacMode_HEAT
from .const import PROPORTIONAL_FUNCTION_AUTO_PI

_LOGGER = logging.getLogger(__name__)

class RLS2:
    """
    Recursive Least Squares estimator for a 1st order thermal model:
    dT_int = a * power + b * (T_ext - T_int)

    Estimates parameters 'a' (heating/cooling efficiency) and 'b' (thermal loss).
    """

    def __init__(self, cycle_min: int, lmbda: float = 0.98):
        self.cycle_min = cycle_min
        self.dt = cycle_min * 60  # Cycle duration in seconds
        self.lmbda = lmbda  # Forgetting factor

        # Initialize parameters
        # theta = [a, b]^T
        self.theta_a = 0.01  # Initial guess for 'a'
        self.theta_b = 0.0001 # Initial guess for 'b'

        # Covariance matrix P initialized with large values (high uncertainty)
        self.p11 = 1000.0
        self.p12 = 0.0
        self.p21 = 0.0
        self.p22 = 1000.0

    def update(self, power: float, t_in: float, t_out: float, t_in_prev: float):
        """
        Update the RLS estimator with new measurements.
        """
        if t_in_prev is None:
            return

        # Model: T_in[k] - T_in[k-1] = a * u[k-1] - b * (T_in[k-1] - T_out[k-1])
        # y = dT
        # phi = [u, -(T_in - T_out)]

        y = t_in - t_in_prev
        phi1 = power
        phi2 = -(t_in_prev - t_out)

        # 1. Calculate gain vector K = (P * phi) / (lambda + phi.T * P * phi)
        # P * phi
        p_phi1 = self.p11 * phi1 + self.p12 * phi2
        p_phi2 = self.p21 * phi1 + self.p22 * phi2

        # denominator = lambda + phi.T * (P * phi)
        denom = self.lmbda + (phi1 * p_phi1 + phi2 * p_phi2)

        k1 = p_phi1 / denom
        k2 = p_phi2 / denom

        # 2. Update prediction error e = y - phi.T * theta
        prediction = self.theta_a * phi1 + self.theta_b * phi2
        error = y - prediction

        # 3. Update parameters theta = theta + K * e
        self.theta_a += k1 * error
        self.theta_b += k2 * error

        # Constrain parameters to physical reality
        # 'a' (efficiency) must be positive
        self.theta_a = max(1e-6, self.theta_a)
        # 'b' (losses) must be positive and stable (b*dt < 1 implies T_in doesn't drop instantly)
        self.theta_b = max(1e-6, self.theta_b)

        # 4. Update covariance matrix P = (P - K * phi.T * P) / lambda
        # term = K * (phi.T * P) -> term_ij = Ki * (phi1*P1j + phi2*P2j)
        # We already have (P * phi) which is (phi.T * P)^T since P is symmetric
        # So (phi.T * P) = [p_phi1, p_phi2]

        term11 = k1 * p_phi1
        term12 = k1 * p_phi2
        term21 = k2 * p_phi1
        term22 = k2 * p_phi2

        self.p11 = (self.p11 - term11) / self.lmbda
        self.p12 = (self.p12 - term12) / self.lmbda
        self.p21 = (self.p21 - term21) / self.lmbda
        self.p22 = (self.p22 - term22) / self.lmbda

    def get_time_constant(self) -> float:
        """Return estimated time constant (tau = 1/b) in seconds"""
        if self.theta_b > 1e-9:
            # theta_b is normalized per cycle (dt) in the update equation approx?
            # actually model: dT = a*u - b*(T-Text).
            # continuous: dT/dt = (a/dt)*u - (b/dt)*(T-text) ? No.
            # RLS estimates the discrete coefficients for the step 'dt'.
            # Discrete form: T[k] = T[k-1] + a_d * u - b_d * (T - Text)
            # Time constant tau: e^(-dt/tau) = 1 - b_d  => -dt/tau = ln(1-b_d) => tau = -dt / ln(1-b_d)
            # For small b_d, tau approx dt / b_d.
            if self.theta_b < 1.0:
                 return -self.dt / math.log(1.0 - self.theta_b)
            return self.dt / self.theta_b # Fallback
        return 3600.0 * 10 # Default large time constant if b is 0

    def get_static_gain(self) -> float:
         """Return static gain = a/b"""
         if self.theta_b > 1e-9:
             return self.theta_a / self.theta_b
         return 0.0

class AutoPI:
    """
    AutoPI Controller
    Uses RLS2 to estimate system parameters and adjusts PI gains automatically.
    """

    def __init__(
        self,
        cycle_min: int,
        minimal_activation_delay: int,
        minimal_deactivation_delay: int,
        name: str,
        max_on_percent: float = None,
        # Persistence data
        saved_state: Optional[Dict[str, Any]] = None
    ):
        self._name = name
        self._cycle_min = cycle_min
        self._minimal_activation_delay = minimal_activation_delay
        self._minimal_deactivation_delay = minimal_deactivation_delay
        self._max_on_percent = max_on_percent

        # RLS Estimator
        self.rls = RLS2(cycle_min)

        # PI State
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.u_prev = 0.0 # Previous control output

        # Calculated outputs
        self._on_percent = 0.0
        self._on_time_sec = 0
        self._off_time_sec = 0

        # Constants for PI tuning (Pole Placement / IMC)
        # Desired closed loop time constant (e.g., 1/3 of open loop or fixed)
        self.tau_c_factor = 0.5 # Aggressiveness. Lower = faster. 
        
        # Current gains (updated by learning)
        self.Kp = 0.0
        self.Ki = 0.0
        self.a = 0.0
        self.b = 0.0

        if saved_state:
            self.load_state(saved_state)
        
        # Initialize gains
        self._update_gains()

        _LOGGER.debug("%s - AutoPI initialized", self._name)

    def load_state(self, state: Dict[str, Any]):
        """Load persistent state"""
        if not state:
            return
        self.rls.theta_a = state.get('theta_a', 0.01)
        self.rls.theta_b = state.get('theta_b', 0.0001)
        self.rls.p11 = state.get('p11', 1000.0)
        self.rls.p12 = state.get('p12', 0.0)
        self.rls.p21 = state.get('p21', 0.0)
        self.rls.p22 = state.get('p22', 1000.0)
        self.integral_error = state.get('integral', 0.0)
        self.u_prev = state.get('u_prev', 0.0)
        _LOGGER.debug("%s - AutoPI state loaded: a=%.4f, b=%.4f", self._name, self.rls.theta_a, self.rls.theta_b)

    def save_state(self) -> Dict[str, Any]:
        """Return state for persistence"""
        return {
            'theta_a': self.rls.theta_a,
            'theta_b': self.rls.theta_b,
            'p11': self.rls.p11,
            'p12': self.rls.p12,
            'p21': self.rls.p21,
            'p22': self.rls.p22,
            'integral': self.integral_error,
            'u_prev': self.u_prev
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
        """
        if previous_temp is not None and previous_power is not None and hvac_mode == VThermHvacMode_HEAT:
            # We assume we only learn in HEAT mode for now
            self.rls.update(previous_power, current_temp, ext_current_temp if ext_current_temp else current_temp, previous_temp)
            self._update_gains()

    def _update_gains(self):
        """Calculate PI gains based on current RLS parameters"""
        # Ensure stability
        self.a = max(self.rls.theta_a, 1e-4) # gain
        self.b = max(self.rls.theta_b, 1e-6) # decay
        
        # Estimate Time Constant tau (seconds)
        # If b is small, tau approx dt / b
        tau = self.rls.dt / self.b
        
        # Desired closed loop time constant 
        tau_c = tau * self.tau_c_factor
        # Clamp tau_c to be at least the cycle time to avoid oscillation
        tau_c = max(tau_c, self.rls.dt)

        # System Static Gain K_sys = a/b
        k_sys = self.a / self.b
        
        # Calculate Gains for PI
        # Kp = (1/K) * (tau/tau_c)
        self.Kp = (1.0 / k_sys) * (tau / tau_c)
        self.Ki = self.Kp / tau # Integral time Ti = tau
        
        _LOGGER.debug("%s - AutoPI Gains Updated: a=%.5f, b=%.5f, Kp=%.2f, Ki=%.4f", self._name, self.a, self.b, self.Kp, self.Ki)

    def calculate(
        self,
        target_temp: float | None,
        current_temp: float | None,
        ext_current_temp: float | None,
        slope: float | None, # Not used in PI but kept for interface compatibility
        hvac_mode: VThermHvacMode,
    ):
        """
        Calculate the heating percentage (Control Loop).
        """
        if target_temp is None or current_temp is None:
            self._on_percent = 0
            return

        # 3. Calculate Error
        error = target_temp - current_temp
        if hvac_mode == VThermHvacMode_COOL:
            error = -error
        elif hvac_mode == VThermHvacMode_OFF:
            self._on_percent = 0
            self.u_prev = 0
            return

        # 4. Feedforward
        # u_ff = (T_target - Text) / K_sys = (T_target - Text) * (b/a)
        u_ff = 0
        if ext_current_temp is not None:
            delta_t_ext = target_temp - ext_current_temp
            if self.a > 0:
                u_ff = delta_t_ext * (self.b / self.a)
        
        # 5. PI Calculation
        
        # Integral update
        if self._can_integrate(hvac_mode, self.u_prev):
            self.integral_error += error * self.rls.dt
        
        # u = Kp*e + Ki*I + u_ff
        u = self.Kp * error + self.Ki * self.integral_error + u_ff
        
        # Saturation
        if u > 1.0:
            u = 1.0
        elif u < 0.0:
            u = 0.0
        
        # Update output
        self._on_percent = u
        self.u_prev = u
        self.prev_error = error
        
        # Calculate times
        self._calculate_times()
        
        _LOGGER.debug("%s - AutoPI Calculate: T=%s, Tgt=%s, Ext=%s", self._name, current_temp, target_temp, ext_current_temp)
        _LOGGER.debug("%s - Terms: P=%.2f, I=%.2f, FF=%.2f -> u=%.2f", self._name, self.Kp*error, self.Ki*self.integral_error, u_ff, u)

    def _can_integrate(self, hvac_mode, u_prev):
        """Logic to prevent integral windup"""
        # Stop integrating if saturated
        if u_prev >= 1.0 or u_prev <= 0.0:
             return False
        return True

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

    @property
    def on_percent(self):
        return self._on_percent

    @property
    def on_time_sec(self):
        return self._on_time_sec

    @property
    def off_time_sec(self):
        return self._off_time_sec
