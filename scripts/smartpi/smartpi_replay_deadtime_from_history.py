
import json
import logging
import argparse
import sys
import os

# Add project root to python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from datetime import datetime
from custom_components.versatile_thermostat.prop_algo_smartpi import DeadTimeEstimator

# Configure logging to show debug output
logging.basicConfig(level=logging.DEBUG, format='%(name)s - %(levelname)s - %(message)s')
_LOGGER = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser(description="Replay history from a JSON file to reproduce deadtime estimation behavior.")
    parser.add_argument("history_file", help="Path to the history JSON file")
    args = parser.parse_args()

    histo_file = args.history_file

    if not os.path.exists(histo_file):
        print(f"Error: History file {histo_file} not found")
        sys.exit(1)

    print(f"Loading history from {histo_file}...")
    with open(histo_file, "r") as f:
        history_data = json.load(f)

    # Sort history by timestamp just in case
    # Convert timestamps to compare
    history_data.sort(key=lambda x: datetime.fromisoformat(x.get("timestamp") or "1970-01-01T00:00:00+00:00").timestamp() if x.get("timestamp") else 0)

    est = DeadTimeEstimator()
    results = []
    prev_cool_dt = None
    prev_heat_dt = None
    
    print(f"\nStarting replay of {len(history_data)} entries...")

    prev_ts = 0.0
    for i, entry in enumerate(history_data):
        ts_str = entry.get("timestamp")
        if not ts_str:
            continue
            
        try:
            dt = datetime.fromisoformat(ts_str)
            now = dt.timestamp()
        except ValueError:
            continue
            
        if now <= prev_ts:
            continue
        prev_ts = now

        attrs = entry.get("attributes", {})
        
        # Tin
        tin = attrs.get("current_temperature")
        if tin is None:
            continue
            
        # Setpoint
        # attributes -> current_state -> target_temperature
        current_state = attrs.get("current_state", {})
        sp = current_state.get("target_temperature")
        if sp is None:
             # Try other locations
             req_state = attrs.get("requested_state", {})
             sp = req_state.get("target_temperature")
        
        if sp is None:
            # Fallback to temperature attribute if it looks like SP (sometimes it is)
            # But in the sample, `temperature` was 18.0 which matched current_state.target_temperature
            sp = attrs.get("temperature")

        if sp is None:
            continue

        # U applied
        # In the provided JSON, `attributes -> specific_states -> smart_pi -> u_applied` 
        # or `attributes -> power_percent` / 100.0?
        # Let's check smart_pi first.
        specific = attrs.get("specific_states", {})
        smart_pi = specific.get("smart_pi", {})
        u_applied = smart_pi.get("u_applied")
        
        if u_applied is None:
            # fallback to power_percent
            pp = attrs.get("power_percent")
            if pp is not None:
                u_applied = float(pp) / 100.0
            else:
                u_applied = 0.0

        # Max on percent
        max_on_percent = 1.0
        
        # Check if in Hysteresis phase
        # attributes -> specific_states -> smart_pi -> phase
        phase = smart_pi.get("phase")
        is_hysteresis = (phase == "Hysteresis")

        # est.update is synchronous
        prev_state = est.state
        est.update(now=now, tin=tin, sp=sp, u_applied=u_applied, max_on_percent=max_on_percent, is_hysteresis=is_hysteresis)

        if est.state != prev_state:
             print(f"[{i}] State Change: {prev_state} -> {est.state} (Temp: {tin:.2f}, Power: {u_applied:.2f})")
             
        if est.state == "WAITING_HEAT_RESPONSE" and prev_state != "WAITING_HEAT_RESPONSE":
            print(f"    Heat Start Time: {est.heat_start_time} (Temp: {est.heat_start_temp})")
            
        if est.state == "WAITING_COOL_RESPONSE" and prev_state != "WAITING_COOL_RESPONSE":
            print(f"    Cool Start Time: {est.cool_start_time} (Peak Temp: {est.cool_peak_temp})")

        if est.deadtime_cool_s is not None and est.deadtime_cool_s != prev_cool_dt:
            print(f"[{i}] New Deadtime Cool: {est.deadtime_cool_s} (Reliable: {est.deadtime_cool_reliable})")
            results.append(f"Cool: {est.deadtime_cool_s:.1f}s (Rel: {est.deadtime_cool_reliable})")
            prev_cool_dt = est.deadtime_cool_s
            
        if est.deadtime_heat_s is not None and est.deadtime_heat_s != prev_heat_dt:
            print(f"[{i}] New Deadtime Heat: {est.deadtime_heat_s} (Reliable: {est.deadtime_reliable})")
            results.append(f"Heat: {est.deadtime_heat_s:.1f}s (Rel: {est.deadtime_reliable})")
            prev_heat_dt = est.deadtime_heat_s

    print("\n=== Final Results ===")
    for r in results:
        print(r)
    
    if est.deadtime_cool_s is None:
        print("INFO: Deadtime Cool was not detected (expected for heating-only history)")
    else:
        print(f"Final Deadtime Cool: {est.deadtime_cool_s}")

    if est.deadtime_heat_s is None:
        print("INFO: Deadtime Heat was not detected")
    else:
        print(f"Final Deadtime Heat: {est.deadtime_heat_s}")

if __name__ == "__main__":
    main()
