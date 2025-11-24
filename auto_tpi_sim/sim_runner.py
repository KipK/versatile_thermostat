import asyncio
import logging
import sys
import os
import random
import math
from datetime import datetime, timedelta
from unittest.mock import MagicMock, patch

# Add project root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, project_root)

# Import Mocks
from auto_tpi_sim.mocks import MockHomeAssistant
from auto_tpi_sim.models import ROOM_CONFIGS, RoomModel

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger("SimRunner")

# Import Target (after path insertion)
try:
    from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager
except ImportError as e:
    logger.error(f"Failed to import AutoTpiManager: {e}")
    sys.exit(1)

class SimulationRunner:
    def __init__(self, room_type='normal', days=10):
        self.room_config = ROOM_CONFIGS[room_type]
        self.room = RoomModel(self.room_config)
        self.days = days
        self.hass = MockHomeAssistant()
        # Cycle min 15 minutes to allow for enough data points
        self.cycle_min = 15
        self.auto_tpi = AutoTpiManager(self.hass, f"sim_{room_type}", cycle_min=self.cycle_min, tpi_threshold_low=0.1, tpi_threshold_high=0.1)
        
        self.current_time = datetime(2024, 1, 1, 0, 0, 0)
        self.sim_step_min = 2 # 2 minutes step -> ~7-8 points per cycle
        
        self.target_temp = 20.0
        self.current_power_setpoint = 0.0
        
    async def run(self):
        logger.info(f"Starting simulation for {self.room_config.name} over {self.days} days")
        
        # Start learning
        await self.auto_tpi.start_learning()
        await self.auto_tpi.on_thermostat_mode_changed("HEAT", self.target_temp)
        
        total_steps = int(self.days * 24 * 60 / self.sim_step_min)
        
        # Patch datetime used in auto_tpi_manager
        with patch('custom_components.versatile_thermostat.auto_tpi_manager.datetime') as mock_datetime:
            mock_datetime.now.side_effect = lambda: self.current_time
            mock_datetime.fromtimestamp = datetime.fromtimestamp
            
            # Initialize cycle tracking
            last_cycle_time = self.current_time
            
            for step in range(total_steps):
                # 1. Update Environment Time
                self.current_time += timedelta(minutes=self.sim_step_min)
                
                # External Temp: Seasonality + Day/Night + Random
                # Simplified: 5 deg avg, +/- 5 deg daily swing
                hour_of_year = (step * self.sim_step_min) / 60.0
                ext_temp = 5.0 + 5.0 * math.sin(hour_of_year * 2 * math.pi / 24.0) + random.uniform(-0.2, 0.2)
                
                # 2. Control Logic (Simple TPI/PWM simulation)
                # We update power setpoint only at start of cycles (simplified)
                time_in_cycle = (self.current_time - last_cycle_time).total_seconds() / 60.0
                
                if time_in_cycle >= self.cycle_min:
                    # End of cycle
                    await self.auto_tpi.on_cycle_elapsed()
                    last_cycle_time = self.current_time
                    
                    # New cycle calculation
                    # Calculate needed power (Feedforward + P)
                    # Ideal Power = alpha * (Target - Ext) / beta
                    ideal_power = (self.room_config.alpha * (self.target_temp - ext_temp)) / self.room_config.beta
                    
                    # Add Proportional error
                    error = self.target_temp - self.room.temp
                    p_term = 0.5 * error
                    
                    self.current_power_setpoint = ideal_power + p_term
                    
                    # Clamp
                    self.current_power_setpoint = max(0.0, min(1.0, self.current_power_setpoint))
                    
                    # Add some randomness to simulate real world imperfections
                    self.current_power_setpoint += random.uniform(-0.05, 0.05)
                    self.current_power_setpoint = max(0.0, min(1.0, self.current_power_setpoint))

                
                power_percent = self.current_power_setpoint * 100.0
                hvac_action = "heating" if power_percent > 0 else "idle"
                
                # 3. Physics Step
                self.room.step(ext_temp, power_percent, self.sim_step_min)
                
                # 4. Feed AutoTpiManager
                await self.auto_tpi.update(
                    room_temp=self.room.temp,
                    ext_temp=ext_temp,
                    power_percent=power_percent,
                    target_temp=self.target_temp,
                    hvac_action=hvac_action
                )
                
                # Logging progress
                if step % (24 * 60 // self.sim_step_min) == 0:
                    day = step // (24 * 60 // self.sim_step_min)
                    print(f"  Day {day}: T_in={self.room.temp:.2f}, T_ext={ext_temp:.2f}, Power={power_percent:.0f}%")

            # Final Calculation
            print("\n--- Simulation Finished. Calculating parameters... ---")
            result = await self.auto_tpi.calculate()
            return result

async def main():
    print("=== Auto TPI Simulation Runner ===")
    
    # Simulate for each room type
    for room_type in ['well_insulated', 'normal', 'badly_insulated']:
        print(f"\n########################################")
        print(f"Running simulation for: {room_type}")
        print(f"########################################")
        
        runner = SimulationRunner(room_type, days=5) # 5 days is enough for ~480 cycles (if 15min)
        result = await runner.run()
        
        config = ROOM_CONFIGS[room_type]
        
        if result:
            print(f"\n>>> Results for {config.name}:")
            print(f"Calculated Parameters: {result}")
            
            # Validation
            # Theoretical k_ext = alpha / beta
            theoretical_k_ext = config.alpha / config.beta
            calc_k_ext = result.get('k_ext')
            
            # Theoretical time constant = 1/alpha (seconds) -> hours
            theoretical_tau = (1.0 / config.alpha) if config.alpha > 0 else 0
            calc_tau = result.get('time_constant_hours')
            
            print(f"\nValidation:")
            print(f"  k_ext: Predicted={calc_k_ext}, Theoretical={theoretical_k_ext:.4f}")
            print(f"  tau (h): Predicted={calc_tau}, Theoretical={theoretical_tau:.2f}")
            
            # Simple check
            if calc_k_ext and abs(calc_k_ext - theoretical_k_ext) < 0.05:
                print("  [SUCCESS] k_ext is within reasonable range.")
            else:
                print("  [WARNING] k_ext deviation is high.")
                
        else:
            print(f"\n>>> FAILED: No result calculated for {room_type}.")

if __name__ == "__main__":
    asyncio.run(main())