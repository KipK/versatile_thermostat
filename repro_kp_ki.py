
from dataclasses import dataclass

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

@dataclass
class TauReliability:
    reliable: bool
    tau_min: float

# Constants from code
KP_MIN, KP_MAX = 0.10, 2.50
KI_MIN, KI_MAX = 0.001, 0.050
KP_SAFE = 0.55

# User values
tau_min = 1506.8
tau_reliable = True
aggressiveness = 0.5 # Assumed default
near_band_deg = 0.3
kp_near_factor = 0.7
ki_near_factor = 0.5
target_temp = 20.0
current_temp = 19.85 # Error = 0.15 (User said error is 0.15)
hvac_mode = "HEAT"

def calculate_gains():
    print("--- Simulating AutoPI Gain Calculation ---")
    
    # 1. Check tau reliability
    if tau_reliable:
        tau = tau_min
        # Heuristic: Kp scales with tau
        kp_calc = 0.35 + 0.9 * (tau / 200.0)
        print(f"Base kp_calc (reliable): {kp_calc}")
    else:
        kp_calc = KP_SAFE
        tau = 200.0 # Placeholder
        print(f"Base kp_calc (unreliable): {kp_calc}")

    # 2. Apply aggressiveness
    kp_calc *= (aggressiveness * 2)
    print(f"After aggressiveness ({aggressiveness}): {kp_calc}")

    # 3. Calculate error
    e = target_temp - current_temp
    print(f"Error: {e}")

    # 4. Near-band gain scheduling
    in_near_band = (near_band_deg > 0.0) and (abs(e) <= near_band_deg)
    print(f"In near band: {in_near_band}")
    
    if in_near_band:
        kp_calc *= kp_near_factor
        print(f"After near_band factor ({kp_near_factor}): {kp_calc}")
    
    # 5. Clamp Kp
    kp = clamp(kp_calc, KP_MIN, KP_MAX)
    print(f"Final Kp (clamped [{KP_MIN}, {KP_MAX}]): {kp}")
    if kp == KP_MAX:
        print("-> Kp is SATURATED at MAX")

    # 6. Calculate Ki
    ki_calc = kp / max(tau, 10.0)
    print(f"Base Ki (Kp/tau): {ki_calc}")
    
    # 7. Clamp Ki (first pass)
    ki = clamp(ki_calc, KI_MIN, KI_MAX)
    print(f"Ki after first clamp: {ki}")

    # 8. Apply ki_near_factor
    if in_near_band:
        ki *= ki_near_factor
        print(f"Ki after near_band factor ({ki_near_factor}): {ki}")
        ki = clamp(ki, KI_MIN, KI_MAX)
        print(f"Final Ki (clamped [{KI_MIN}, {KI_MAX}]): {ki}")
        if ki == KI_MIN:
            print("-> Ki is SATURATED at MIN")

calculate_gains()
