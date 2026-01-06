#!/usr/bin/env python3
"""
AutoPI Analysis Report Script

This script connects to a Home Assistant instance, retrieves climate entity history,
extracts AutoPI algorithm metrics, and generates analysis reports with graphs.

Usage:
    python autopi_report.py \
        --url http://homeassistant.local:8123 \
        --token YOUR_LONG_LIVED_ACCESS_TOKEN \
        --entity climate.your_thermostat \
        --days 7 \
        --output-dir ./reports \
        --verbose
"""

import argparse
import os
import sys
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Optional, Tuple

import requests

try:
    import matplotlib.pyplot as plt
    import matplotlib.dates as mdates
    from matplotlib.patches import Patch
    from matplotlib.backends.backend_pdf import PdfPages
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not installed. Graphs and PDF will not be generated.")
    print("Install with: pip install matplotlib")


# ------------------------------------------------------------------------------
# API Functions
# ------------------------------------------------------------------------------

def fetch_history(
    base_url: str,
    token: str,
    entity_id: str,
    days: int,
    verbose: bool = False
) -> List[List[Dict[str, Any]]]:
    """Fetch entity history from Home Assistant API."""
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
    }

    now = datetime.now(timezone.utc)
    start = now - timedelta(days=days)

    start_iso = start.strftime("%Y-%m-%dT%H:%M:%S")
    end_iso = now.strftime("%Y-%m-%dT%H:%M:%S")

    url = f"{base_url}/api/history/period/{start_iso}"
    params = {
        "filter_entity_id": entity_id,
        "end_time": end_iso,
        # IMPORTANT: Get all state changes, not just significant ones
        # This is needed to capture attribute-only changes
        "significant_changes_only": "false",
        "minimal_response": "false",
    }

    if verbose:
        print(f"[DEBUG] API call: {url}")
        print(f"[DEBUG] Entity: {entity_id}")
        print(f"[DEBUG] Params: {params}")
        print(f"[DEBUG] Period: {start.strftime('%Y-%m-%d %H:%M')} to {now.strftime('%Y-%m-%d %H:%M')}")

    try:
        r = requests.get(url, headers=headers, params=params, timeout=120)
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] API request failed: {e}")
        sys.exit(1)

    if r.status_code != 200:
        print(f"[ERROR] API Error: {r.status_code} - {r.text}")
        sys.exit(1)

    data = r.json()

    if verbose:
        if len(data) > 0 and len(data[0]) > 0:
            print(f"[DEBUG] Retrieved {len(data[0])} history states")
            first_ts = data[0][0].get("last_changed", "")
            last_ts = data[0][-1].get("last_changed", "")
            print(f"[DEBUG] First: {first_ts}")
            print(f"[DEBUG] Last:  {last_ts}")
            
            # Show sample of available attributes from first state with attributes
            for i, state in enumerate(data[0][:5]):
                attrs = state.get("attributes", {})
                if attrs:
                    print(f"[DEBUG] Sample state #{i} attributes keys: {list(attrs.keys())}")
                    if "auto_pi" in attrs:
                        print(f"[DEBUG]   -> HAS auto_pi attribute!")
                    if "vtherm_over_switch" in attrs:
                        vos = attrs.get("vtherm_over_switch", {})
                        if isinstance(vos, dict):
                            print(f"[DEBUG]   -> vtherm_over_switch.function: {vos.get('function')}")
                    break

    return data


# ------------------------------------------------------------------------------
# Data Extraction
# ------------------------------------------------------------------------------

def extract_autopi_data(
    history: List[List[Dict[str, Any]]],
    verbose: bool = False
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    Extract AutoPI metrics from climate entity history.
    
    Returns:
        Tuple of (time_series_data, summary_stats)
    """
    if not history or len(history) == 0 or len(history[0]) == 0:
        print("[ERROR] No history data received")
        return [], {}

    states = history[0]
    data_points = []
    autopi_count = 0
    no_autopi_count = 0

    for state in states:
        attrs = state.get("attributes", {}) or {}
        
        # Check if this is an AutoPI-enabled thermostat
        # AutoPI data is nested inside specific_states.auto_pi
        specific_states = attrs.get("specific_states", {}) or {}
        auto_pi = specific_states.get("auto_pi")
        
        # Parse timestamp
        ts_str = state.get("last_changed", "")
        try:
            ts = datetime.fromisoformat(ts_str.replace("Z", "+00:00"))
        except (ValueError, AttributeError):
            continue

        # Extract core temperature data
        current_temp = attrs.get("current_temperature")
        target_temp = attrs.get("temperature")
        hvac_mode = state.get("state")
        hvac_action = attrs.get("hvac_action")
        ema_temp = attrs.get("ema_temp") or specific_states.get("ema_temp")
        ext_temp = specific_states.get("ext_current_temperature")

        # If no auto_pi attribute, skip for AutoPI analysis but count
        if auto_pi is None:
            no_autopi_count += 1
            continue

        autopi_count += 1

        # Extract AutoPI metrics
        point = {
            "timestamp": ts,
            # Temperatures
            "current_temp": current_temp,
            "target_temp": target_temp,
            "ema_temp": ema_temp,
            "ext_temp": ext_temp,
            "hvac_mode": hvac_mode,
            "hvac_action": hvac_action,
            # Model parameters
            "a": auto_pi.get("a"),
            "b": auto_pi.get("b"),
            "tau_min": auto_pi.get("tau_min"),
            "tau_reliable": auto_pi.get("tau_reliable"),
            # Learning
            "learn_ok_count": auto_pi.get("learn_ok_count"),
            "learn_ok_count_a": auto_pi.get("learn_ok_count_a"),
            "learn_ok_count_b": auto_pi.get("learn_ok_count_b"),
            "learn_skip_count": auto_pi.get("learn_skip_count"),
            "learn_last_reason": auto_pi.get("learn_last_reason"),
            # Controller
            "Kp": auto_pi.get("Kp"),
            "Ki": auto_pi.get("Ki"),
            "integral_error": auto_pi.get("integral_error"),
            "error": auto_pi.get("error"),
            "error_p": auto_pi.get("error_p"),
            "error_filtered": auto_pi.get("error_filtered"),
            "i_mode": auto_pi.get("i_mode"),
            "sat": auto_pi.get("sat"),
            # Output
            "on_percent": auto_pi.get("on_percent"),
            "on_time_sec": auto_pi.get("on_time_sec"),
            "off_time_sec": auto_pi.get("off_time_sec"),
            "u_ff": auto_pi.get("u_ff"),
            "cycles_since_reset": auto_pi.get("cycles_since_reset"),
            "cycle_min": auto_pi.get("cycle_min"),
        }
        data_points.append(point)

    if verbose:
        print(f"[DEBUG] States with auto_pi: {autopi_count}")
        print(f"[DEBUG] States without auto_pi: {no_autopi_count}")

    if not data_points:
        print("[WARNING] No AutoPI data found in history.")
        print("Make sure the climate entity uses AutoPI (function: auto_pi)")
        return [], {}

    # Compute summary statistics
    summary = compute_summary_stats(data_points)

    return data_points, summary


def compute_summary_stats(data: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compute summary statistics from extracted data."""
    if not data:
        return {}

    # Get latest values
    latest = data[-1]
    first = data[0]

    # Extract numeric lists for averaging
    errors = [p["error"] for p in data if p["error"] is not None]
    on_percents = [p["on_percent"] for p in data if p["on_percent"] is not None]
    learn_ok_counts = [p["learn_ok_count"] for p in data if p["learn_ok_count"] is not None]
    learn_skip_counts = [p["learn_skip_count"] for p in data if p["learn_skip_count"] is not None]

    # Calculate error statistics
    abs_errors = [abs(e) for e in errors] if errors else []

    # Count i_mode occurrences
    i_mode_counts = {}
    for p in data:
        mode = p.get("i_mode")
        if mode:
            i_mode_counts[mode] = i_mode_counts.get(mode, 0) + 1

    # Count learn_last_reason occurrences
    reason_counts = {}
    for p in data:
        reason = p.get("learn_last_reason")
        if reason:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1

    # Time range
    time_range_hours = (data[-1]["timestamp"] - data[0]["timestamp"]).total_seconds() / 3600

    return {
        # Current model state
        "current_a": latest.get("a"),
        "current_b": latest.get("b"),
        "current_tau_min": latest.get("tau_min"),
        "current_tau_reliable": latest.get("tau_reliable"),
        "current_Kp": latest.get("Kp"),
        "current_Ki": latest.get("Ki"),
        # Learning progress
        "total_learn_ok": max(learn_ok_counts) if learn_ok_counts else 0,
        "total_learn_ok_a": latest.get("learn_ok_count_a", 0),
        "total_learn_ok_b": latest.get("learn_ok_count_b", 0),
        "total_learn_skip": max(learn_skip_counts) if learn_skip_counts else 0,
        "learn_rate": (max(learn_ok_counts) / (max(learn_ok_counts) + max(learn_skip_counts)) * 100
                      if learn_ok_counts and (max(learn_ok_counts) + max(learn_skip_counts)) > 0 else 0),
        # Temperature performance
        "mean_error": sum(errors) / len(errors) if errors else None,
        "mean_abs_error": sum(abs_errors) / len(abs_errors) if abs_errors else None,
        "max_error": max(errors) if errors else None,
        "min_error": min(errors) if errors else None,
        # Power stats
        "mean_on_percent": sum(on_percents) / len(on_percents) * 100 if on_percents else None,
        "max_on_percent": max(on_percents) * 100 if on_percents else None,
        # I-mode distribution
        "i_mode_distribution": i_mode_counts,
        # Learning reason distribution
        "reason_distribution": reason_counts,
        # Time info
        "time_range_hours": time_range_hours,
        "data_points": len(data),
        "first_timestamp": first["timestamp"],
        "last_timestamp": latest["timestamp"],
        "cycles_since_reset": latest.get("cycles_since_reset"),
    }


# ------------------------------------------------------------------------------
# Graph Generation
# ------------------------------------------------------------------------------

def generate_graphs(
    data: List[Dict[str, Any]],
    summary: Dict[str, Any],
    output_dir: str,
    entity_id: str,
    verbose: bool = False
) -> List[str]:
    """Generate analysis graphs and save as PNG files."""
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib not available, skipping graph generation")
        return []

    if not data:
        return []

    os.makedirs(output_dir, exist_ok=True)
    generated_files = []

    # Prepare time series
    timestamps = [p["timestamp"] for p in data]
    
    # Entity name for titles and file names
    entity_short = entity_id.split(".")[-1]  # e.g., "thermostat_chambre_verte"
    entity_name = entity_short.replace("_", " ").title()  # For display

    # Configure matplotlib style
    plt.style.use('seaborn-v0_8-whitegrid') if 'seaborn-v0_8-whitegrid' in plt.style.available else None
    plt.rcParams['figure.figsize'] = (14, 6)
    plt.rcParams['font.size'] = 10

    # --- Graph 1: Learning Progress (a, b, tau) ---
    fig1, axes1 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig1.suptitle(f"AutoPI Learning Progress - {entity_name}", fontsize=14, fontweight='bold')

    # Parameter 'a' (heating effectiveness)
    a_vals = [p["a"] for p in data]
    axes1[0].plot(timestamps, a_vals, 'b-', linewidth=1, label='a (°C/min at 100%)')
    axes1[0].set_ylabel('a (°C/min)')
    axes1[0].set_title('Heating Effectiveness (a)')
    axes1[0].legend(loc='upper right')
    axes1[0].grid(True, alpha=0.3)

    # Parameter 'b' (loss coefficient)
    b_vals = [p["b"] for p in data]
    axes1[1].plot(timestamps, b_vals, 'r-', linewidth=1, label='b (1/min)')
    axes1[1].set_ylabel('b (1/min)')
    axes1[1].set_title('Loss Coefficient (b)')
    axes1[1].legend(loc='upper right')
    axes1[1].grid(True, alpha=0.3)

    # Tau with reliability indicator
    tau_vals = [p["tau_min"] for p in data]
    reliable_vals = [p["tau_reliable"] for p in data]
    
    axes1[2].plot(timestamps, tau_vals, 'g-', linewidth=1, label='τ (minutes)')
    
    # Shade reliable regions
    reliable_start = None
    for i, (ts, rel) in enumerate(zip(timestamps, reliable_vals)):
        if rel and reliable_start is None:
            reliable_start = ts
        elif not rel and reliable_start is not None:
            axes1[2].axvspan(reliable_start, timestamps[i-1], alpha=0.2, color='green', label='_')
            reliable_start = None
    if reliable_start is not None:
        axes1[2].axvspan(reliable_start, timestamps[-1], alpha=0.2, color='green', label='_')

    axes1[2].set_ylabel('τ (min)')
    axes1[2].set_xlabel('Time')
    axes1[2].set_title('Thermal Time Constant (τ = 1/b) - Green = Reliable')
    axes1[2].legend(loc='upper right')
    axes1[2].grid(True, alpha=0.3)

    axes1[2].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
    plt.setp(axes1[2].xaxis.get_majorticklabels(), rotation=45, ha='right')

    plt.tight_layout()
    path1 = os.path.join(output_dir, f"{entity_short}_01_learning_progress.png")
    fig1.savefig(path1, dpi=150, bbox_inches='tight')
    plt.close(fig1)
    generated_files.append(path1)
    if verbose:
        print(f"[DEBUG] Saved: {path1}")

    # --- Graph 2: Temperature Tracking ---
    fig2, ax2 = plt.subplots(figsize=(14, 6))
    fig2.suptitle(f"Temperature Tracking - {entity_name}", fontsize=14, fontweight='bold')

    current_temps = [p["current_temp"] for p in data]
    target_temps = [p["target_temp"] for p in data]
    ext_temps = [p["ext_temp"] for p in data]
    on_percents = [p["on_percent"] if p["on_percent"] is not None else 0 for p in data]

    # Plot temperatures
    ax2.plot(timestamps, current_temps, 'b-', linewidth=1.5, label='Current T°')
    # Use step plot for target (setpoint stays constant, then jumps)
    ax2.step(timestamps, target_temps, 'r-', linewidth=1.5, where='post', label='Target T°')
    if any(t is not None for t in ext_temps):
        ax2.plot(timestamps, ext_temps, 'c-', linewidth=1, alpha=0.7, label='External T°')

    ax2.set_ylabel('Temperature (°C)')
    ax2.set_xlabel('Time')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    # Secondary axis for power
    ax2b = ax2.twinx()
    ax2b.fill_between(timestamps, 0, [p * 100 for p in on_percents], 
                       alpha=0.2, color='orange', label='Power %')
    ax2b.set_ylabel('Power (%)', color='orange')
    ax2b.set_ylim(0, 110)
    ax2b.tick_params(axis='y', labelcolor='orange')

    ax2.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
    plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45, ha='right')

    plt.tight_layout()
    path2 = os.path.join(output_dir, f"{entity_short}_02_temperature_tracking.png")
    fig2.savefig(path2, dpi=150, bbox_inches='tight')
    plt.close(fig2)
    generated_files.append(path2)
    if verbose:
        print(f"[DEBUG] Saved: {path2}")

    # --- Graph 3: PI Controller State ---
    fig3, axes3 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig3.suptitle(f"PI Controller State - {entity_name}", fontsize=14, fontweight='bold')

    # Gains - Kp on left axis, Ki on right axis (different scales)
    kp_vals = [p["Kp"] for p in data]
    ki_vals = [p["Ki"] for p in data]
    
    # Kp on primary (left) axis
    axes3[0].plot(timestamps, kp_vals, 'b-', linewidth=1, label='Kp')
    axes3[0].set_ylabel('Kp', color='blue')
    axes3[0].tick_params(axis='y', labelcolor='blue')
    axes3[0].set_title('Controller Gains (Kp, Ki)')
    axes3[0].grid(True, alpha=0.3)
    
    # Ki on secondary (right) axis with its own scale
    ax3_ki = axes3[0].twinx()
    ax3_ki.plot(timestamps, ki_vals, 'r-', linewidth=1, label='Ki')
    ax3_ki.set_ylabel('Ki', color='red')
    ax3_ki.tick_params(axis='y', labelcolor='red')
    
    # Combined legend
    lines1, labels1 = axes3[0].get_legend_handles_labels()
    lines2, labels2 = ax3_ki.get_legend_handles_labels()
    axes3[0].legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Error
    error_vals = [p["error"] for p in data]
    error_p_vals = [p["error_p"] for p in data]
    axes3[1].plot(timestamps, error_vals, 'b-', linewidth=1, label='Error (e)')
    axes3[1].plot(timestamps, error_p_vals, 'g-', linewidth=1, alpha=0.7, label='Error_p (2DOF)')
    axes3[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes3[1].set_ylabel('Error (°C)')
    axes3[1].set_title('Control Error')
    axes3[1].legend(loc='upper right')
    axes3[1].grid(True, alpha=0.3)

    # Integral
    integral_vals = [p["integral_error"] for p in data]
    axes3[2].plot(timestamps, integral_vals, 'purple', linewidth=1, label='Integral Error')
    axes3[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes3[2].set_ylabel('Integral')
    axes3[2].set_xlabel('Time')
    axes3[2].set_title('Integral Accumulator')
    axes3[2].legend(loc='upper right')
    axes3[2].grid(True, alpha=0.3)

    axes3[2].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
    plt.setp(axes3[2].xaxis.get_majorticklabels(), rotation=45, ha='right')

    plt.tight_layout()
    path3 = os.path.join(output_dir, f"{entity_short}_03_pi_controller.png")
    fig3.savefig(path3, dpi=150, bbox_inches='tight')
    plt.close(fig3)
    generated_files.append(path3)
    if verbose:
        print(f"[DEBUG] Saved: {path3}")

    # --- Graph 4: Power Output ---
    fig4, axes4 = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig4.suptitle(f"Power Output Analysis - {entity_name}", fontsize=14, fontweight='bold')

    # Power breakdown
    u_ff_vals = [p["u_ff"] if p["u_ff"] is not None else 0 for p in data]
    on_percent_vals = [p["on_percent"] if p["on_percent"] is not None else 0 for p in data]
    
    axes4[0].fill_between(timestamps, 0, [v * 100 for v in u_ff_vals], 
                           alpha=0.5, color='blue', label='Feed-forward (u_ff)')
    axes4[0].plot(timestamps, [v * 100 for v in on_percent_vals], 'r-', 
                   linewidth=1.5, label='Total Power (on_percent)')
    axes4[0].set_ylabel('Power (%)')
    axes4[0].set_title('Power Output Breakdown')
    axes4[0].legend(loc='upper right')
    axes4[0].set_ylim(0, 110)
    axes4[0].grid(True, alpha=0.3)

    # Cycles since reset
    cycles_vals = [p["cycles_since_reset"] for p in data]
    axes4[1].plot(timestamps, cycles_vals, 'g-', linewidth=1, label='Cycles since reset')
    axes4[1].set_ylabel('Cycles')
    axes4[1].set_xlabel('Time')
    axes4[1].set_title('Cycles Since Reset')
    axes4[1].legend(loc='upper right')
    axes4[1].grid(True, alpha=0.3)

    axes4[1].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
    plt.setp(axes4[1].xaxis.get_majorticklabels(), rotation=45, ha='right')

    plt.tight_layout()
    path4 = os.path.join(output_dir, f"{entity_short}_04_power_output.png")
    fig4.savefig(path4, dpi=150, bbox_inches='tight')
    plt.close(fig4)
    generated_files.append(path4)
    if verbose:
        print(f"[DEBUG] Saved: {path4}")

    # --- Graph 5: Learning Quality ---
    fig5, axes5 = plt.subplots(2, 1, figsize=(14, 8))
    fig5.suptitle(f"Learning Quality Analysis - {entity_name}", fontsize=14, fontweight='bold')

    # Learning count evolution
    learn_ok_vals = [p["learn_ok_count"] for p in data]
    learn_skip_vals = [p["learn_skip_count"] for p in data]
    axes5[0].plot(timestamps, learn_ok_vals, 'g-', linewidth=1.5, label='Learn OK count')
    axes5[0].plot(timestamps, learn_skip_vals, 'r-', linewidth=1, alpha=0.7, label='Learn SKIP count')
    axes5[0].set_ylabel('Count')
    axes5[0].set_title('Learning Progress (Cumulative)')
    axes5[0].legend(loc='upper left')
    axes5[0].grid(True, alpha=0.3)
    axes5[0].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
    plt.setp(axes5[0].xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Reason distribution (pie chart)
    reason_dist = summary.get("reason_distribution", {})
    if reason_dist:
        labels = list(reason_dist.keys())
        sizes = list(reason_dist.values())
        colors = ['#2ecc71' if 'ok' in l else '#e74c3c' if 'skip' in l else '#3498db' for l in labels]
        axes5[1].pie(sizes, labels=labels, autopct='%1.1f%%', colors=colors, startangle=90)
        axes5[1].set_title('Learning Reason Distribution')
    else:
        axes5[1].text(0.5, 0.5, 'No data', ha='center', va='center', fontsize=14)
        axes5[1].set_title('Learning Reason Distribution')

    plt.tight_layout()
    path5 = os.path.join(output_dir, f"{entity_short}_05_learning_quality.png")
    fig5.savefig(path5, dpi=150, bbox_inches='tight')
    plt.close(fig5)
    generated_files.append(path5)
    if verbose:
        print(f"[DEBUG] Saved: {path5}")

    return generated_files


# ------------------------------------------------------------------------------
# Text Report
# ------------------------------------------------------------------------------

def generate_text_report(
    summary: Dict[str, Any],
    entity_id: str,
    days: int,
    output_dir: str
) -> str:
    """Generate a text summary report."""
    os.makedirs(output_dir, exist_ok=True)
    
    # Entity name for file naming
    entity_short = entity_id.split(".")[-1]  # e.g., "thermostat_chambre_verte"

    lines = []
    lines.append("=" * 70)
    lines.append("  AUTOPI ANALYSIS REPORT")
    lines.append("=" * 70)
    lines.append("")
    lines.append(f"Entity:        {entity_id}")
    lines.append(f"Period:        {days} days")
    lines.append(f"Data points:   {summary.get('data_points', 'N/A')}")
    
    if summary.get('first_timestamp') and summary.get('last_timestamp'):
        lines.append(f"From:          {summary['first_timestamp'].strftime('%Y-%m-%d %H:%M')}")
        lines.append(f"To:            {summary['last_timestamp'].strftime('%Y-%m-%d %H:%M')}")
    
    lines.append(f"Duration:      {summary.get('time_range_hours', 0):.1f} hours")
    lines.append("")

    # Model state
    lines.append("-" * 70)
    lines.append("CURRENT MODEL STATE")
    lines.append("-" * 70)
    lines.append(f"  a (heating eff.)    : {summary.get('current_a', 'N/A'):.6f} °C/min" if summary.get('current_a') else "  a: N/A")
    lines.append(f"  b (loss coef.)      : {summary.get('current_b', 'N/A'):.6f} 1/min" if summary.get('current_b') else "  b: N/A")
    lines.append(f"  τ (time constant)   : {summary.get('current_tau_min', 'N/A'):.1f} min" if summary.get('current_tau_min') else "  τ: N/A")
    lines.append(f"  τ reliable          : {'Yes ✓' if summary.get('current_tau_reliable') else 'No ✗'}")
    lines.append(f"  Kp                  : {summary.get('current_Kp', 'N/A'):.4f}" if summary.get('current_Kp') else "  Kp: N/A")
    lines.append(f"  Ki                  : {summary.get('current_Ki', 'N/A'):.6f}" if summary.get('current_Ki') else "  Ki: N/A")
    lines.append(f"  Cycles since reset  : {summary.get('cycles_since_reset', 'N/A')}")
    lines.append("")

    # Learning progress
    lines.append("-" * 70)
    lines.append("LEARNING PROGRESS")
    lines.append("-" * 70)
    lines.append(f"  Learn OK count      : {summary.get('total_learn_ok', 0)}")
    lines.append(f"    - a learned (ON)  : {summary.get('total_learn_ok_a', 0)}")
    lines.append(f"    - b learned (OFF) : {summary.get('total_learn_ok_b', 0)}")
    lines.append(f"  Learn SKIP count    : {summary.get('total_learn_skip', 0)}")
    lines.append(f"  Success rate        : {summary.get('learn_rate', 0):.1f}%")
    lines.append("")
    
    # Learning reasons
    reason_dist = summary.get('reason_distribution', {})
    if reason_dist:
        lines.append("  Reason Distribution:")
        for reason, count in sorted(reason_dist.items(), key=lambda x: -x[1]):
            lines.append(f"    - {reason}: {count}")
    lines.append("")

    # Temperature performance
    lines.append("-" * 70)
    lines.append("TEMPERATURE PERFORMANCE")
    lines.append("-" * 70)
    if summary.get('mean_error') is not None:
        lines.append(f"  Mean error          : {summary['mean_error']:+.3f}°C")
    if summary.get('mean_abs_error') is not None:
        lines.append(f"  Mean |error|        : {summary['mean_abs_error']:.3f}°C")
    if summary.get('max_error') is not None:
        lines.append(f"  Max overheating     : {summary['max_error']:+.3f}°C")
    if summary.get('min_error') is not None:
        lines.append(f"  Max underheating    : {summary['min_error']:+.3f}°C")
    lines.append("")

    # Power stats
    lines.append("-" * 70)
    lines.append("POWER USAGE")
    lines.append("-" * 70)
    if summary.get('mean_on_percent') is not None:
        lines.append(f"  Mean power          : {summary['mean_on_percent']:.1f}%")
    if summary.get('max_on_percent') is not None:
        lines.append(f"  Max power           : {summary['max_on_percent']:.1f}%")
    lines.append("")

    # I-mode distribution
    i_mode_dist = summary.get('i_mode_distribution', {})
    if i_mode_dist:
        lines.append("-" * 70)
        lines.append("INTEGRATOR MODE DISTRIBUTION")
        lines.append("-" * 70)
        total = sum(i_mode_dist.values())
        for mode, count in sorted(i_mode_dist.items(), key=lambda x: -x[1]):
            pct = count / total * 100 if total > 0 else 0
            lines.append(f"  {mode:25s} : {count:5d} ({pct:5.1f}%)")
    lines.append("")

    lines.append("=" * 70)
    lines.append("")

    report_text = "\n".join(lines)
    
    # Save to file
    report_path = os.path.join(output_dir, f"{entity_short}_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(report_text)

    return report_text


# ------------------------------------------------------------------------------
# PDF Report Generation
# ------------------------------------------------------------------------------

def generate_pdf_report(
    data: List[Dict[str, Any]],
    summary: Dict[str, Any],
    report_text: str,
    output_dir: str,
    entity_id: str,
    verbose: bool = False
) -> Optional[str]:
    """Generate a PDF report with text summary and all graphs."""
    if not HAS_MATPLOTLIB:
        print("[WARNING] matplotlib not available, skipping PDF generation")
        return None
    
    if not data:
        return None
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Entity name for file naming
    entity_short = entity_id.split(".")[-1]  # e.g., "thermostat_chambre_verte"
    pdf_path = os.path.join(output_dir, f"{entity_short}_report.pdf")
    
    entity_name = entity_id.split(".")[-1].replace("_", " ").title()
    timestamps = [p["timestamp"] for p in data]
    
    with PdfPages(pdf_path) as pdf:
        # Page 1: Text Report
        fig_text = plt.figure(figsize=(11, 8.5))
        fig_text.suptitle(f"AutoPI Analysis Report - {entity_name}", fontsize=16, fontweight='bold', y=0.98)
        
        # Add text as a text box
        ax_text = fig_text.add_subplot(111)
        ax_text.axis('off')
        ax_text.text(0.02, 0.95, report_text, transform=ax_text.transAxes, 
                     fontsize=8, verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        pdf.savefig(fig_text, bbox_inches='tight')
        plt.close(fig_text)
        
        # Page 2: Learning Progress (a, b, tau)
        fig1, axes1 = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
        fig1.suptitle(f"AutoPI Learning Progress - {entity_name}", fontsize=14, fontweight='bold')
        
        a_vals = [p["a"] for p in data]
        axes1[0].plot(timestamps, a_vals, 'b-', linewidth=1, label='a (°C/min at 100%)')
        axes1[0].set_ylabel('a (°C/min)')
        axes1[0].set_title('Heating Effectiveness (a)')
        axes1[0].legend(loc='upper right')
        axes1[0].grid(True, alpha=0.3)

        b_vals = [p["b"] for p in data]
        axes1[1].plot(timestamps, b_vals, 'r-', linewidth=1, label='b (1/min)')
        axes1[1].set_ylabel('b (1/min)')
        axes1[1].set_title('Loss Coefficient (b)')
        axes1[1].legend(loc='upper right')
        axes1[1].grid(True, alpha=0.3)

        tau_vals = [p["tau_min"] for p in data]
        reliable_vals = [p["tau_reliable"] for p in data]
        axes1[2].plot(timestamps, tau_vals, 'g-', linewidth=1, label='τ (minutes)')
        
        reliable_start = None
        for i, (ts, rel) in enumerate(zip(timestamps, reliable_vals)):
            if rel and reliable_start is None:
                reliable_start = ts
            elif not rel and reliable_start is not None:
                axes1[2].axvspan(reliable_start, timestamps[i-1], alpha=0.2, color='green')
                reliable_start = None
        if reliable_start is not None:
            axes1[2].axvspan(reliable_start, timestamps[-1], alpha=0.2, color='green')
        
        axes1[2].set_ylabel('τ (min)')
        axes1[2].set_xlabel('Time')
        axes1[2].set_title('Thermal Time Constant (τ = 1/b) - Green = Reliable')
        axes1[2].legend(loc='upper right')
        axes1[2].grid(True, alpha=0.3)
        axes1[2].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(axes1[2].xaxis.get_majorticklabels(), rotation=45, ha='right')
        
        plt.tight_layout()
        pdf.savefig(fig1, bbox_inches='tight')
        plt.close(fig1)
        
        # Page 3: Temperature Tracking
        fig2, ax2 = plt.subplots(figsize=(11, 6))
        fig2.suptitle(f"Temperature Tracking - {entity_name}", fontsize=14, fontweight='bold')
        
        current_temps = [p["current_temp"] for p in data]
        target_temps = [p["target_temp"] for p in data]
        ext_temps = [p["ext_temp"] for p in data]
        on_percents = [p["on_percent"] if p["on_percent"] is not None else 0 for p in data]
        
        ax2.plot(timestamps, current_temps, 'b-', linewidth=1.5, label='Current T°')
        ax2.step(timestamps, target_temps, 'r-', linewidth=1.5, where='post', label='Target T°')
        if any(t is not None for t in ext_temps):
            ax2.plot(timestamps, ext_temps, 'c-', linewidth=1, alpha=0.7, label='External T°')
        
        ax2.set_ylabel('Temperature (°C)')
        ax2.set_xlabel('Time')
        ax2.legend(loc='upper left')
        ax2.grid(True, alpha=0.3)
        
        ax2b = ax2.twinx()
        ax2b.fill_between(timestamps, 0, [p * 100 for p in on_percents], alpha=0.2, color='orange')
        ax2b.set_ylabel('Power (%)', color='orange')
        ax2b.set_ylim(0, 110)
        ax2b.tick_params(axis='y', labelcolor='orange')
        
        ax2.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(ax2.xaxis.get_majorticklabels(), rotation=45, ha='right')
        
        plt.tight_layout()
        pdf.savefig(fig2, bbox_inches='tight')
        plt.close(fig2)
        
        # Page 4: PI Controller State
        fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
        fig3.suptitle(f"PI Controller State - {entity_name}", fontsize=14, fontweight='bold')
        
        kp_vals = [p["Kp"] for p in data]
        ki_vals = [p["Ki"] for p in data]
        
        axes3[0].plot(timestamps, kp_vals, 'b-', linewidth=1, label='Kp')
        axes3[0].set_ylabel('Kp', color='blue')
        axes3[0].tick_params(axis='y', labelcolor='blue')
        axes3[0].set_title('Controller Gains (Kp, Ki)')
        axes3[0].grid(True, alpha=0.3)
        
        ax3_ki = axes3[0].twinx()
        ax3_ki.plot(timestamps, ki_vals, 'r-', linewidth=1, label='Ki')
        ax3_ki.set_ylabel('Ki', color='red')
        ax3_ki.tick_params(axis='y', labelcolor='red')
        
        lines1, labels1 = axes3[0].get_legend_handles_labels()
        lines2, labels2 = ax3_ki.get_legend_handles_labels()
        axes3[0].legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        error_vals = [p["error"] for p in data]
        error_p_vals = [p["error_p"] for p in data]
        axes3[1].plot(timestamps, error_vals, 'b-', linewidth=1, label='Error (e)')
        axes3[1].plot(timestamps, error_p_vals, 'g-', linewidth=1, alpha=0.7, label='Error_p (2DOF)')
        axes3[1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        axes3[1].set_ylabel('Error (°C)')
        axes3[1].set_title('Control Error')
        axes3[1].legend(loc='upper right')
        axes3[1].grid(True, alpha=0.3)
        
        integral_vals = [p["integral_error"] for p in data]
        axes3[2].plot(timestamps, integral_vals, 'purple', linewidth=1, label='Integral Error')
        axes3[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        axes3[2].set_ylabel('Integral')
        axes3[2].set_xlabel('Time')
        axes3[2].set_title('Integral Accumulator')
        axes3[2].legend(loc='upper right')
        axes3[2].grid(True, alpha=0.3)
        
        axes3[2].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(axes3[2].xaxis.get_majorticklabels(), rotation=45, ha='right')
        
        plt.tight_layout()
        pdf.savefig(fig3, bbox_inches='tight')
        plt.close(fig3)
        
        # Page 5: Power Output
        fig4, axes4 = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        fig4.suptitle(f"Power Output Analysis - {entity_name}", fontsize=14, fontweight='bold')
        
        u_ff_vals = [p["u_ff"] if p["u_ff"] is not None else 0 for p in data]
        on_percent_vals = [p["on_percent"] if p["on_percent"] is not None else 0 for p in data]
        
        axes4[0].fill_between(timestamps, 0, [v * 100 for v in u_ff_vals], alpha=0.5, color='blue', label='Feed-forward (u_ff)')
        axes4[0].plot(timestamps, [v * 100 for v in on_percent_vals], 'r-', linewidth=1.5, label='Total Power (on_percent)')
        axes4[0].set_ylabel('Power (%)')
        axes4[0].set_title('Power Output Breakdown')
        axes4[0].legend(loc='upper right')
        axes4[0].set_ylim(0, 110)
        axes4[0].grid(True, alpha=0.3)
        
        cycles_vals = [p["cycles_since_reset"] for p in data]
        axes4[1].plot(timestamps, cycles_vals, 'g-', linewidth=1, label='Cycles since reset')
        axes4[1].set_ylabel('Cycles')
        axes4[1].set_xlabel('Time')
        axes4[1].set_title('Cycles Since Reset')
        axes4[1].legend(loc='upper right')
        axes4[1].grid(True, alpha=0.3)
        
        axes4[1].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(axes4[1].xaxis.get_majorticklabels(), rotation=45, ha='right')
        
        plt.tight_layout()
        pdf.savefig(fig4, bbox_inches='tight')
        plt.close(fig4)
        
        # Page 6: Learning Quality
        fig5, axes5 = plt.subplots(2, 1, figsize=(11, 8))
        fig5.suptitle(f"Learning Quality Analysis - {entity_name}", fontsize=14, fontweight='bold')
        
        learn_ok_vals = [p["learn_ok_count"] for p in data]
        learn_skip_vals = [p["learn_skip_count"] for p in data]
        axes5[0].plot(timestamps, learn_ok_vals, 'g-', linewidth=1.5, label='Learn OK count')
        axes5[0].plot(timestamps, learn_skip_vals, 'r-', linewidth=1, alpha=0.7, label='Learn SKIP count')
        axes5[0].set_ylabel('Count')
        axes5[0].set_title('Learning Progress (Cumulative)')
        axes5[0].legend(loc='upper left')
        axes5[0].grid(True, alpha=0.3)
        axes5[0].xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(axes5[0].xaxis.get_majorticklabels(), rotation=45, ha='right')
        
        reason_dist = summary.get("reason_distribution", {})
        if reason_dist:
            labels = list(reason_dist.keys())
            sizes = list(reason_dist.values())
            colors = ['#2ecc71' if 'ok' in l else '#e74c3c' if 'skip' in l else '#3498db' for l in labels]
            axes5[1].pie(sizes, labels=labels, autopct='%1.1f%%', colors=colors, startangle=90)
            axes5[1].set_title('Learning Reason Distribution')
        else:
            axes5[1].text(0.5, 0.5, 'No data', ha='center', va='center', fontsize=14)
            axes5[1].set_title('Learning Reason Distribution')
        
        plt.tight_layout()
        pdf.savefig(fig5, bbox_inches='tight')
        plt.close(fig5)
    
    if verbose:
        print(f"[DEBUG] PDF saved: {pdf_path}")
    
    return pdf_path


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="AutoPI Algorithm Analysis Report Generator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python autopi_report.py --url http://ha:8123 --token xxx --entity climate.my_thermostat
  python autopi_report.py --url http://ha:8123 --token xxx --entity climate.my_thermostat --days 14 --verbose
        """
    )
    parser.add_argument("--url", required=True, 
                        help="Home Assistant URL (e.g., http://homeassistant.local:8123)")
    parser.add_argument("--token", required=True, 
                        help="Long-Lived Access Token")
    parser.add_argument("--entity", required=True, 
                        help="Climate entity ID (e.g., climate.thermostat_salon)")
    parser.add_argument("--days", type=int, default=7, 
                        help="Number of days of history to fetch (default: 7)")
    parser.add_argument("--output-dir", default="./reports", 
                        help="Directory for output files (default: ./reports)")
    parser.add_argument("--verbose", action="store_true", 
                        help="Enable verbose debug output")

    args = parser.parse_args()

    print(f"\n{'='*70}")
    print(f"  AUTOPI ANALYSIS REPORT")
    print(f"{'='*70}\n")
    print(f"Entity:     {args.entity}")
    print(f"Period:     {args.days} days")
    print(f"Output:     {args.output_dir}")
    print()

    # Fetch history
    print("[INFO] Fetching history from Home Assistant...")
    history = fetch_history(args.url, args.token, args.entity, args.days, args.verbose)

    # Extract data
    print("[INFO] Extracting AutoPI data...")
    data, summary = extract_autopi_data(history, args.verbose)

    if not data:
        print("\n[ERROR] No AutoPI data found. Exiting.")
        sys.exit(1)

    print(f"[INFO] Extracted {len(data)} data points")

    # Generate graphs
    if HAS_MATPLOTLIB:
        print("[INFO] Generating graphs...")
        generated = generate_graphs(data, summary, args.output_dir, args.entity, args.verbose)
        print(f"[INFO] Generated {len(generated)} graphs")
    else:
        print("[WARNING] Skipping graph generation (matplotlib not installed)")

    # Generate text report
    print("[INFO] Generating text report...")
    report = generate_text_report(summary, args.entity, args.days, args.output_dir)
    print()
    print(report)
    
    # Generate PDF report
    if HAS_MATPLOTLIB:
        print("[INFO] Generating PDF report...")
        pdf_path = generate_pdf_report(data, summary, report, args.output_dir, args.entity, args.verbose)
        if pdf_path:
            print(f"[INFO] PDF report saved: {pdf_path}")
    else:
        print("[WARNING] Skipping PDF generation (matplotlib not installed)")

    print(f"\n[INFO] All outputs saved to: {os.path.abspath(args.output_dir)}")
    print("[INFO] Done!")


if __name__ == "__main__":
    main()
