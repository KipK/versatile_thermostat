from dataclasses import dataclass
from typing import Dict, Optional

@dataclass
class RoomConfig:
    """
    Configuration for a simulated room.
    
    alpha: Heat loss coefficient (1/h). Rate at which temp drops per degree of difference.
    beta: Heating efficiency (K/h). Rate at which temp rises at 100% power.
    """
    name: str
    alpha: float
    beta: float

    @property
    def time_constant(self) -> float:
        """Return time constant in hours."""
        return 1.0 / self.alpha if self.alpha > 0 else float('inf')

# Predefined configurations
ROOM_CONFIGS = {
    'well_insulated': RoomConfig(name='Well Insulated', alpha=0.1, beta=2.5),  # Tau=10h
    'normal': RoomConfig(name='Normal', alpha=0.35, beta=4.0),                 # Tau=2.8h
    'badly_insulated': RoomConfig(name='Badly Insulated', alpha=0.8, beta=7.0), # Tau=1.25h
}

class RoomModel:
    """
    Simulates the thermal evolution of a room.
    
    Equation: dT/dt = -alpha * (T_in - T_ext) + beta * Power
    """
    def __init__(self, config: RoomConfig, initial_temp: float = 20.0):
        self.config = config
        self.temp = initial_temp
        self.time_hours = 0.0

    def step(self, ext_temp: float, power_percent: float, dt_minutes: float) -> float:
        """
        Advance the simulation by dt_minutes.
        
        :param ext_temp: External temperature (C)
        :param power_percent: Heating power (0-100)
        :param dt_minutes: Time step in minutes
        :return: New room temperature
        """
        dt_hours = dt_minutes / 60.0
        
        # Power normalized 0-1
        power_normalized = power_percent / 100.0
        
        # Calculate derivative: dT/dt (K/h)
        # Loss component: -alpha * (T_in - T_ext)
        loss = -self.config.alpha * (self.temp - ext_temp)
        
        # Gain component: +beta * Power
        gain = self.config.beta * power_normalized
        
        derivative = loss + gain
        
        # Update temperature (Euler method)
        self.temp += derivative * dt_hours
        self.time_hours += dt_hours
        
        return self.temp