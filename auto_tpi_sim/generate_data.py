import csv
import os
import random
import math
from datetime import datetime, timedelta
from auto_tpi_sim.models import ROOM_CONFIGS, RoomModel

# Configuration
OUTPUT_DIR = "auto_tpi_sim/data"
DAYS_TO_SIMULATE = 2
SIM_STEP_MIN = 5  # 5 minute intervals
CYCLE_MIN = 15    # 15 minute TPI cycle

def ensure_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

def generate_simulation_data(room_type: str, days: int):
    room_config = ROOM_CONFIGS[room_type]
    room = RoomModel(room_config)
    
    print(f"Generating data for {room_config.name} ({room_type})...")
    
    start_time = datetime(2024, 1, 1, 0, 0, 0)
    current_time = start_time
    total_steps = int(days * 24 * 60 / SIM_STEP_MIN)
    
    # Simulation state
    target_temp = 20.0
    last_cycle_time = start_time
    current_power_setpoint = 0.0
    
    data_points = []
    
    for step in range(total_steps):
        # 1. Update Environment Time
        current_time += timedelta(minutes=SIM_STEP_MIN)
        
        # External Temp: Seasonality + Day/Night + Random
        # Simplified: 5 deg avg, +/- 5 deg daily swing
        hour_of_year = (step * SIM_STEP_MIN) / 60.0
        ext_temp = 5.0 + 5.0 * math.sin(hour_of_year * 2 * math.pi / 24.0) + random.uniform(-0.2, 0.2)
        
        # 2. Control Logic (Simple TPI/PWM simulation for data generation)
        time_in_cycle = (current_time - last_cycle_time).total_seconds() / 60.0
        
        if time_in_cycle >= CYCLE_MIN:
            last_cycle_time = current_time
            
            # Ideal Power to maintain steady state
            ideal_power = (room_config.alpha * (target_temp - ext_temp)) / room_config.beta
            
            # P-Controller
            error = target_temp - room.temp
            p_term = 0.5 * error
            
            current_power_setpoint = ideal_power + p_term
            
            # Clamp and noise
            current_power_setpoint = max(0.0, min(1.0, current_power_setpoint))
            current_power_setpoint += random.uniform(-0.05, 0.05)
            current_power_setpoint = max(0.0, min(1.0, current_power_setpoint))

        power_percent = current_power_setpoint * 100.0
        
        # 3. Physics Step
        room.step(ext_temp, power_percent, SIM_STEP_MIN)
        
        # Store data point
        # Columns: timestamp, target_temp, temp_ext, power, temp_int
        data_points.append({
            "timestamp": current_time.isoformat(),
            "target_temp": round(target_temp, 2),
            "temp_ext": round(ext_temp, 2),
            "power": round(power_percent, 2),
            "temp_int": round(room.temp, 4)
        })

    # Write to CSV
    filename = os.path.join(OUTPUT_DIR, f"{room_type}.csv")
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'target_temp', 'temp_ext', 'power', 'temp_int']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in data_points:
            writer.writerow(row)
            
    print(f"Saved {len(data_points)} points to {filename}")

def main():
    ensure_directory(OUTPUT_DIR)
    
    room_types = ['well_insulated', 'normal', 'badly_insulated']
    for rt in room_types:
        generate_simulation_data(rt, days=DAYS_TO_SIMULATE)

if __name__ == "__main__":
    main()