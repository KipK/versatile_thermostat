#!/usr/bin/env python3
"""
SmartPI Analysis Report Script

This script connects to a Home Assistant instance, retrieves climate entity history,
extracts SmartPI algorithm metrics, and generates analysis reports with graphs.

Usage:
    python smartpi_report.py \
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

def get_entity_state(
    base_url: str,
    token: str,
    entity_id: str,
    verbose: bool = False
) -> Dict[str, Any]:
    """Fetch current entity state."""
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
    }
    url = f"{base_url}/api/states/{entity_id}"
    
    if verbose:
        print(f"[DEBUG] Fetching state: {url}")

    try:
        r = requests.get(url, headers=headers, timeout=20)
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Failed to fetch entity state: {e}")
        sys.exit(1)

    if r.status_code != 200:
        print(f"[ERROR] API Error fetching state: {r.status_code} - {r.text}")
        sys.exit(1)

    return r.json()

def fetch_history(
    base_url: str,
    token: str,
    entity_id: str,
    days: Optional[int] = None,
    start_date: Optional[datetime] = None,
    verbose: bool = False
) -> List[List[Dict[str, Any]]]:
    """Fetch entity history from Home Assistant API."""
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json",
    }

    now = datetime.now(timezone.utc)

    if start_date:
        start = start_date
        # If naive, we assume it's appropriate for the query or compatible with comparison
        # But for history API, we usually want a specific timestamp.
        # If it has no timezone, we might want to attach one or leave as is depending on HA expectations.
        # Usually HA stores everything in UTC. If learning_start_dt is naive, it might happen.
        if start.tzinfo is None:
             # Assume UTC if not specified, to be safe for calculations
             start = start.replace(tzinfo=timezone.utc)
    elif days:
        start = now - timedelta(days=days)
    else:
        # Default fallback
        start = now - timedelta(days=7)

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
                    if "smart_pi" in attrs:
                        print(f"[DEBUG]   -> HAS smart_pi attribute!")
                    if "vtherm_over_switch" in attrs:
                        vos = attrs.get("vtherm_over_switch", {})
                        if isinstance(vos, dict):
                            print(f"[DEBUG]   -> vtherm_over_switch.function: {vos.get('function')}")
                    break

    return data


# ------------------------------------------------------------------------------
# Data Extraction
# ------------------------------------------------------------------------------

def extract_smartpi_data(
    history: List[List[Dict[str, Any]]],
    verbose: bool = False
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    Extract SmartPI metrics from climate entity history.
    
    Returns:
        Tuple of (time_series_data, summary_stats)
    """
    if not history or len(history) == 0 or len(history[0]) == 0:
        print("[ERROR] No history data received")
        return [], {}

    states = history[0]
    data_points = []
    smartpi_count = 0
    no_smartpi_count = 0

    # Variables for EMA calculation
    u_pi_avg_prev = 0.0
    alpha = 0.15

    for state in states:
        attrs = state.get("attributes", {}) or {}
        
        # Parse timestamp
        ts_str = state.get("last_changed", "")

        # Check if this is an SmartPI-enabled thermostat
        # SmartPI data is nested inside specific_states.smart_pi
        specific_states = attrs.get("specific_states", {}) or {}
        smart_pi = specific_states.get("smart_pi")
        if verbose:
             # Reduce verbose spam, only print if smart_pi status changes or periodically
             pass
             
        if smart_pi is None:
            # Fallback for legacy AutoPI data in history
            smart_pi = specific_states.get("auto_pi")

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

        # If no smart_pi attribute (and no auto_pi), skip for SmartPI analysis but count
        if smart_pi is None:
            no_smartpi_count += 1
            continue

        smartpi_count += 1
        
        # Extract raw values
        u_ff = smart_pi.get("u_ff")
        on_percent = smart_pi.get("on_percent")
        error_filtered = smart_pi.get("error_filtered")
        
        # --- Derived Metrics Calculation ---
        
        # 1. u_pi (Effort PI)
        # u_pi = on_percent - u_ff
        # Convert to percentage for display consistency (0-100 scale in graphs usually, 
        # but here we keep 0-1 scale internally if u_ff/on_percent are 0-1?)
        # smart_pi values are usually 0.0 to 1.0. Let's keep them as is and multiply by 100 for display later
        # BUT for calculation consistency, let's work with what we have.
        
        u_pi = None
        if on_percent is not None and u_ff is not None:
            u_pi = on_percent - u_ff

        # 2. u_pi_avg (Exponential Moving Average)
        # Condition: update only if abs(error_filtered) < 0.3
        # u_pi_avg = alpha * u_pi + (1 - alpha) * prev
        
        u_pi_avg = u_pi_avg_prev  # Default to holding value
        
        if u_pi is not None and error_filtered is not None:
            # We work in percentage for the EMA logic threshold (usually easier)
            # But the logic says "u_pi_avg | abs <= 3" (percent).
            # So we should probably do the calc in percent or convert.
            # Let's convert to percent for the calculation to match the template logic:
            # u_percent = u_pi * 100
            
            u_pi_percent = u_pi * 100
            err_val = float(error_filtered)
            
            if abs(err_val) < 0.3:
                # Update EMA
                new_avg = (alpha * u_pi_percent) + ((1 - alpha) * u_pi_avg_prev)
                u_pi_avg = new_avg
                u_pi_avg_prev = new_avg
            else:
                # Hold value
                u_pi_avg = u_pi_avg_prev

        # Extract SmartPI metrics
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
            "a": smart_pi.get("a"),
            "b": smart_pi.get("b"),
            "tau_min": smart_pi.get("tau_min"),
            "tau_reliable": smart_pi.get("tau_reliable"),
            # Learning
            "learn_ok_count": smart_pi.get("learn_ok_count"),
            "learn_ok_count_a": smart_pi.get("learn_ok_count_a"),
            "learn_ok_count_b": smart_pi.get("learn_ok_count_b"),
            "learn_skip_count": smart_pi.get("learn_skip_count"),
            "learn_last_reason": smart_pi.get("learn_last_reason"),
            # Controller
            "Kp": smart_pi.get("Kp"),
            "Ki": smart_pi.get("Ki"),
            "integral_error": smart_pi.get("integral_error"),
            "error": smart_pi.get("error"),
            "error_p": smart_pi.get("error_p"),
            "error_filtered": error_filtered,
            "i_mode": smart_pi.get("i_mode"),
            "on_percent": on_percent,
            "on_time_sec": smart_pi.get("on_time_sec"),
            "off_time_sec": smart_pi.get("off_time_sec"),
            "u_ff": u_ff,
            "cycles_since_reset": smart_pi.get("cycles_since_reset"),
            "cycle_min": smart_pi.get("cycle_min"),
            # Derived
            "u_pi": u_pi,          # Raw scale (0-1 typically)
            "u_pi_avg": u_pi_avg,  # Percent scale (0-100)
        }
        data_points.append(point)

    if verbose:
        print(f"[DEBUG] States with smart_pi: {smartpi_count}")
        print(f"[DEBUG] States without smart_pi: {no_smartpi_count}")

    if not data_points:
        print("[WARNING] No SmartPI data found in history.")
        print("Make sure the climate entity uses SmartPI (function: smart_pi)")
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
        "current_u_pi": latest.get("u_pi"),
        "current_u_pi_avg": latest.get("u_pi_avg"),
        "current_error_filtered": latest.get("error_filtered"),
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
    entity_short = entity_id.split(".")[-1]
    entity_name = entity_short.replace("_", " ").title()

    # Configure matplotlib style
    plt.style.use('seaborn-v0_8-whitegrid') if 'seaborn-v0_8-whitegrid' in plt.style.available else None
    plt.rcParams['figure.figsize'] = (14, 6)
    plt.rcParams['font.size'] = 10

    # Helper for date formatting
    def format_x_date(ax):
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # --- Graph 1: Temperature & Setpoint ---
    fig1, ax1 = plt.subplots(figsize=(14, 6))
    fig1.suptitle(f"1. Temperature Tracking - {entity_name}", fontsize=14, fontweight='bold')

    current_temps = [p["current_temp"] for p in data]
    target_temps = [p["target_temp"] for p in data]
    ext_temps = [p["ext_temp"] for p in data]

    ax1.plot(timestamps, current_temps, 'b-', linewidth=1.5, label='Current T°')
    ax1.step(timestamps, target_temps, 'r-', linewidth=1.5, where='post', label='Target T°')
    
    # External Temp on secondary axis
    ax1b = ax1.twinx()
    if any(t is not None for t in ext_temps):
        ax1b.plot(timestamps, ext_temps, 'c-', linewidth=1, alpha=0.7, label='External T°')
    ax1b.set_ylabel('External T° (°C)', color='cyan')
    ax1b.tick_params(axis='y', labelcolor='cyan')

    ax1.set_ylabel('Temperature (°C)')
    ax1.set_xlabel('Time')
    
    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    
    ax1.grid(True, alpha=0.3)
    format_x_date(ax1)

    path1 = os.path.join(output_dir, f"{entity_short}_01_temperature.png")
    fig1.savefig(path1, dpi=150, bbox_inches='tight')
    plt.close(fig1)
    generated_files.append(path1)

    # --- Graph 2: Command Signals (u_ff, on_percent, error_filtered) ---
    fig2, ax2 = plt.subplots(figsize=(14, 6))
    fig2.suptitle(f"2. Command Signals - {entity_name}", fontsize=14, fontweight='bold')

    u_ff_vals = [p["u_ff"] * 100 if p["u_ff"] is not None else 0 for p in data]
    on_percent_vals = [p["on_percent"] * 100 if p["on_percent"] is not None else 0 for p in data]
    err_f_vals = [p["error_filtered"] for p in data]

    ax2.fill_between(timestamps, 0, u_ff_vals, alpha=0.3, color='blue', label='u_ff (%)')
    ax2.plot(timestamps, on_percent_vals, 'k-', linewidth=1.5, label='on_percent (%)')
    ax2.set_ylabel('Power (%)')
    ax2.set_ylim(0, 110)
    
    ax2b = ax2.twinx()
    ax2b.plot(timestamps, err_f_vals, 'r-', linewidth=1, label='Error Filtered (°C)')
    ax2b.set_ylabel('Error (°C)', color='red')
    ax2b.tick_params(axis='y', labelcolor='red')

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    ax2.grid(True, alpha=0.3)
    format_x_date(ax2)

    path2 = os.path.join(output_dir, f"{entity_short}_02_commands.png")
    fig2.savefig(path2, dpi=150, bbox_inches='tight')
    plt.close(fig2)
    generated_files.append(path2)

    # --- Graph 3: Model Parameters (a, b, tau) ---
    fig3, axes3 = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig3.suptitle(f"3. Model Parameters - {entity_name}", fontsize=14, fontweight='bold')

    a_vals = [p["a"] for p in data]
    b_vals = [p["b"] for p in data]
    tau_vals = [p["tau_min"] for p in data]
    
    axes3[0].plot(timestamps, a_vals, 'b-', label='a (°C/min)')
    axes3[0].set_ylabel('a')
    axes3[0].legend(loc='upper right')
    axes3[0].grid(True, alpha=0.3)

    axes3[1].plot(timestamps, b_vals, 'r-', label='b (1/min)')
    axes3[1].set_ylabel('b')
    axes3[1].legend(loc='upper right')
    axes3[1].grid(True, alpha=0.3)

    axes3[2].plot(timestamps, tau_vals, 'g-', label='tau (min)')
    axes3[2].set_ylabel('tau')
    axes3[2].legend(loc='upper right')
    axes3[2].grid(True, alpha=0.3)
    
    format_x_date(axes3[2])

    path3 = os.path.join(output_dir, f"{entity_short}_03_model.png")
    fig3.savefig(path3, dpi=150, bbox_inches='tight')
    plt.close(fig3)
    generated_files.append(path3)

    # --- Graph 4: PI Coefficients (Kp, Ki) ---
    fig4, ax4 = plt.subplots(figsize=(14, 6))
    fig4.suptitle(f"4. PI Coefficients - {entity_name}", fontsize=14, fontweight='bold')

    kp_vals = [p["Kp"] for p in data]
    ki_vals = [p["Ki"] for p in data]

    ax4.plot(timestamps, kp_vals, 'b-', label='Kp')
    ax4.set_ylabel('Kp', color='blue')
    ax4.tick_params(axis='y', labelcolor='blue')

    ax4b = ax4.twinx()
    ax4b.plot(timestamps, ki_vals, 'r-', label='Ki')
    ax4b.set_ylabel('Ki', color='red')
    ax4b.tick_params(axis='y', labelcolor='red')

    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4b.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    ax4.grid(True, alpha=0.3)
    format_x_date(ax4)

    path4 = os.path.join(output_dir, f"{entity_short}_04_pi_coeffs.png")
    fig4.savefig(path4, dpi=150, bbox_inches='tight')
    plt.close(fig4)
    generated_files.append(path4)

    # --- Graph 5: PI Effort (u_pi) vs Error ---
    fig5, ax5 = plt.subplots(figsize=(14, 6))
    fig5.suptitle(f"5. PI Effort (u_pi) - {entity_name}", fontsize=14, fontweight='bold')

    u_pi_vals = [p["u_pi"] * 100 if p["u_pi"] is not None else 0 for p in data]
    
    ax5.plot(timestamps, u_pi_vals, 'b-', linewidth=1.5, label='Effort PI (u_pi %)')
    ax5.set_ylabel('PI Effort (%)', color='blue')
    ax5.tick_params(axis='y', labelcolor='blue')
    ax5.set_ylim(-50, 50)  # Zoom on reasonable range

    ax5b = ax5.twinx()
    ax5b.plot(timestamps, err_f_vals, 'r-', linewidth=1, label='Error Filtered (°C)')
    ax5b.set_ylabel('Error (°C)', color='red')
    ax5b.tick_params(axis='y', labelcolor='red')

    lines1, labels1 = ax5.get_legend_handles_labels()
    lines2, labels2 = ax5b.get_legend_handles_labels()
    ax5.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    ax5.grid(True, alpha=0.3)
    format_x_date(ax5)

    path5 = os.path.join(output_dir, f"{entity_short}_05_pi_effort.png")
    fig5.savefig(path5, dpi=150, bbox_inches='tight')
    plt.close(fig5)
    generated_files.append(path5)

    # --- Graph 6: Diagnostic FF (u_pi_avg) ---
    fig6, ax6 = plt.subplots(figsize=(14, 6))
    fig6.suptitle(f"6. Diagnostic FF (u_pi_avg) - {entity_name}", fontsize=14, fontweight='bold')

    u_pi_avg_vals = [p["u_pi_avg"] for p in data]

    # Plot zones
    ax6.axhspan(-3, 3, color='green', alpha=0.1, label='OK Zone')
    ax6.axhspan(3, 30, color='blue', alpha=0.1, label='FF Too Weak')
    ax6.axhspan(-30, -3, color='red', alpha=0.1, label='FF Too Strong')

    ax6.plot(timestamps, u_pi_avg_vals, 'k-', linewidth=2, label='PI Avg Effort (u_pi_avg %)')
    ax6.set_ylabel('Avg PI Effort (%)')
    ax6.set_ylim(-30, 30)

    ax6b = ax6.twinx()
    ax6b.plot(timestamps, err_f_vals, 'r-', linewidth=0.8, alpha=0.6, label='Error Filtered (°C)')
    ax6b.set_ylabel('Error (°C)', color='red')
    ax6b.tick_params(axis='y', labelcolor='red')

    lines1, labels1 = ax6.get_legend_handles_labels()
    lines2, labels2 = ax6b.get_legend_handles_labels()
    ax6.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    
    ax6.grid(True, alpha=0.3)
    format_x_date(ax6)

    path6 = os.path.join(output_dir, f"{entity_short}_06_diagnostic_ff.png")
    fig6.savefig(path6, dpi=150, bbox_inches='tight')
    plt.close(fig6)
    generated_files.append(path6)

    if verbose:
        for p in generated_files:
            print(f"[DEBUG] Saved: {p}")

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
    lines.append("  SMARTPI ANALYSIS REPORT")
    lines.append("=" * 70)
    lines.append("")
    lines.append(f"Entity:        {entity_id}")
    lines.append(f"Period:        {days} days")
    lines.append(f"Data points:   {summary.get('data_points', 'N/A')}")
    
    if summary.get('learning_start_dt'):
        lines.append(f"Learning Start: {summary['learning_start_dt'].strftime('%Y-%m-%d %H:%M')}")

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

    LINES_TO_ADD = [] # Placeholder for construction

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

    # Diagnostic FF
    lines.append("-" * 70)
    lines.append("DIAGNOSTIC FEED-FORWARD (FF)")
    lines.append("-" * 70)
    
    err = summary.get("current_error_filtered")
    u_pi_avg = summary.get("current_u_pi_avg")
    
    if err is not None and abs(err) < 0.3:
        lines.append(f"  Stability Check     : OK (Error {err:.2f}°C is within ±0.3°C)")
        
        if u_pi_avg is not None:
             lines.append(f"  Average PI Effort   : {u_pi_avg:.1f}%")
             lines.append("")
             lines.append("  ANALYSIS:")
             
             if abs(u_pi_avg) <= 3:
                 lines.append("    The Feed-Forward model is performing well.")
                 lines.append("    The PI controller is applying very little correction (avg < 3%),")
                 lines.append("    indicating that the calculated power (u_ff) matches the room's needs.")
             elif u_pi_avg > 5:
                 lines.append("    The Feed-Forward model is TOO WEAK.")
                 lines.append("    The PI controller is adding significant power (avg > 5%) to maintain temperature.")
                 lines.append("    Possible causes:")
                 lines.append("      - Heating effectiveness 'a' is underestimated (too low).")
                 lines.append("      - Loss coefficient 'b' is overestimated (too high).")
                 lines.append("      - The room has higher losses than modeled (open window, draft).")
             elif u_pi_avg < -5:
                 lines.append("    The Feed-Forward model is TOO STRONG.")
                 lines.append("    The PI controller is reducing power (avg < -5%) to prevent overheating.")
                 lines.append("    Possible causes:")
                 lines.append("      - External heat sources (sun, appliances) are present but not modeled.")
                 lines.append("      - Heating effectiveness 'a' is overestimated (too high).")
                 lines.append("      - Loss coefficient 'b' is underestimated (too low).")
             else:
                 lines.append("    The Feed-Forward model is acceptable but could be improved.")
                 lines.append("    The PI controller is making moderate corrections.")
    else:
        desc = f"{err:.2f}" if err is not None else "N/A"
        lines.append(f"  Stability Check     : UNSTABLE (Error {desc}°C is outside ±0.3°C)")
        lines.append("  Analysis            : Skipped. Please wait for the temperature to stabilize")
        lines.append("                        around the setpoint to allow for accurate diagnosis.")
    lines.append("")
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
    
    # Helper for date formatting
    def format_x_date(ax):
        ax.xaxis.set_major_formatter(mdates.DateFormatter('%m-%d %H:%M'))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')
    
    with PdfPages(pdf_path) as pdf:
        # Page 0: Text Report
        fig_text = plt.figure(figsize=(11, 8.5))
        fig_text.suptitle(f"SmartPI Analysis Report - {entity_name}", fontsize=16, fontweight='bold', y=0.98)
        
        # Add text as a text box
        ax_text = fig_text.add_subplot(111)
        ax_text.axis('off')
        ax_text.text(0.02, 0.95, report_text, transform=ax_text.transAxes, 
                     fontsize=9, verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
        
        pdf.savefig(fig_text, bbox_inches='tight')
        plt.close(fig_text)

        # --- Graph 1: Temperature & Setpoint ---
        fig1, ax1 = plt.subplots(figsize=(11, 6))
        fig1.suptitle(f"1. Temperature Tracking - {entity_name}", fontsize=14, fontweight='bold')

        current_temps = [p["current_temp"] for p in data]
        target_temps = [p["target_temp"] for p in data]
        ext_temps = [p["ext_temp"] for p in data]

        ax1.plot(timestamps, current_temps, 'b-', linewidth=1.5, label='Current T°')
        ax1.step(timestamps, target_temps, 'r-', linewidth=1.5, where='post', label='Target T°')
        
        ax1b = ax1.twinx()
        if any(t is not None for t in ext_temps):
            ax1b.plot(timestamps, ext_temps, 'c-', linewidth=1, alpha=0.7, label='External T°')
        ax1b.set_ylabel('External T° (°C)', color='cyan')
        ax1b.tick_params(axis='y', labelcolor='cyan')

        ax1.set_ylabel('Temperature (°C)')
        ax1.set_xlabel('Time')
        
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax1b.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
        
        ax1.grid(True, alpha=0.3)
        format_x_date(ax1)
        
        fig1.tight_layout()
        pdf.savefig(fig1, bbox_inches='tight')
        plt.close(fig1)

        # --- Graph 2: Command Signals ---
        fig2, ax2 = plt.subplots(figsize=(11, 6))
        fig2.suptitle(f"2. Command Signals - {entity_name}", fontsize=14, fontweight='bold')

        u_ff_vals = [p["u_ff"] * 100 if p["u_ff"] is not None else 0 for p in data]
        on_percent_vals = [p["on_percent"] * 100 if p["on_percent"] is not None else 0 for p in data]
        err_f_vals = [p["error_filtered"] for p in data]

        ax2.fill_between(timestamps, 0, u_ff_vals, alpha=0.3, color='blue', label='u_ff (%)')
        ax2.plot(timestamps, on_percent_vals, 'k-', linewidth=1.5, label='on_percent (%)')
        ax2.set_ylabel('Power (%)')
        ax2.set_ylim(0, 110)
        
        ax2b = ax2.twinx()
        ax2b.plot(timestamps, err_f_vals, 'r-', linewidth=1, label='Error Filtered (°C)')
        ax2b.set_ylabel('Error (°C)', color='red')
        ax2b.tick_params(axis='y', labelcolor='red')

        lines1, labels1 = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2b.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        ax2.grid(True, alpha=0.3)
        format_x_date(ax2)
        
        fig2.tight_layout()
        pdf.savefig(fig2, bbox_inches='tight')
        plt.close(fig2)

        # --- Graph 3: Model Parameters ---
        fig3, axes3 = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
        fig3.suptitle(f"3. Model Parameters - {entity_name}", fontsize=14, fontweight='bold')

        a_vals = [p["a"] for p in data]
        b_vals = [p["b"] for p in data]
        tau_vals = [p["tau_min"] for p in data]
        
        axes3[0].plot(timestamps, a_vals, 'b-', label='a (°C/min)')
        axes3[0].set_ylabel('a')
        axes3[0].legend(loc='upper right')
        axes3[0].grid(True, alpha=0.3)

        axes3[1].plot(timestamps, b_vals, 'r-', label='b (1/min)')
        axes3[1].set_ylabel('b')
        axes3[1].legend(loc='upper right')
        axes3[1].grid(True, alpha=0.3)

        axes3[2].plot(timestamps, tau_vals, 'g-', label='tau (min)')
        axes3[2].set_ylabel('tau')
        axes3[2].legend(loc='upper right')
        axes3[2].grid(True, alpha=0.3)
        
        format_x_date(axes3[2])
        fig3.tight_layout()
        pdf.savefig(fig3, bbox_inches='tight')
        plt.close(fig3)

        # --- Graph 4: PI Coefficients ---
        fig4, ax4 = plt.subplots(figsize=(11, 6))
        fig4.suptitle(f"4. PI Coefficients - {entity_name}", fontsize=14, fontweight='bold')

        kp_vals = [p["Kp"] for p in data]
        ki_vals = [p["Ki"] for p in data]

        ax4.plot(timestamps, kp_vals, 'b-', label='Kp')
        ax4.set_ylabel('Kp', color='blue')
        ax4.tick_params(axis='y', labelcolor='blue')

        ax4b = ax4.twinx()
        ax4b.plot(timestamps, ki_vals, 'r-', label='Ki')
        ax4b.set_ylabel('Ki', color='red')
        ax4b.tick_params(axis='y', labelcolor='red')

        lines1, labels1 = ax4.get_legend_handles_labels()
        lines2, labels2 = ax4b.get_legend_handles_labels()
        ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        ax4.grid(True, alpha=0.3)
        format_x_date(ax4)
        
        fig4.tight_layout()
        pdf.savefig(fig4, bbox_inches='tight')
        plt.close(fig4)

        # --- Graph 5: PI Effort ---
        fig5, ax5 = plt.subplots(figsize=(11, 6))
        fig5.suptitle(f"5. PI Effort (u_pi) - {entity_name}", fontsize=14, fontweight='bold')

        u_pi_vals = [p["u_pi"] * 100 if p["u_pi"] is not None else 0 for p in data]
        
        ax5.plot(timestamps, u_pi_vals, 'b-', linewidth=1.5, label='Effort PI (u_pi %)')
        ax5.set_ylabel('PI Effort (%)', color='blue')
        ax5.tick_params(axis='y', labelcolor='blue')
        ax5.set_ylim(-50, 50)

        ax5b = ax5.twinx()
        ax5b.plot(timestamps, err_f_vals, 'r-', linewidth=1, label='Error Filtered (°C)')
        ax5b.set_ylabel('Error (°C)', color='red')
        ax5b.tick_params(axis='y', labelcolor='red')

        lines1, labels1 = ax5.get_legend_handles_labels()
        lines2, labels2 = ax5b.get_legend_handles_labels()
        ax5.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        ax5.grid(True, alpha=0.3)
        format_x_date(ax5)
        
        fig5.tight_layout()
        pdf.savefig(fig5, bbox_inches='tight')
        plt.close(fig5)

        # --- Graph 6: Diagnostic FF ---
        fig6, ax6 = plt.subplots(figsize=(11, 6))
        fig6.suptitle(f"6. Diagnostic FF (u_pi_avg) - {entity_name}", fontsize=14, fontweight='bold')

        u_pi_avg_vals = [p["u_pi_avg"] for p in data]

        # Plot zones
        ax6.axhspan(-3, 3, color='green', alpha=0.1, label='OK Zone')
        ax6.axhspan(3, 30, color='blue', alpha=0.1, label='FF Too Weak')
        ax6.axhspan(-30, -3, color='red', alpha=0.1, label='FF Too Strong')

        ax6.plot(timestamps, u_pi_avg_vals, 'k-', linewidth=2, label='PI Avg Effort (u_pi_avg %)')
        ax6.set_ylabel('Avg PI Effort (%)')
        ax6.set_ylim(-30, 30)

        ax6b = ax6.twinx()
        ax6b.plot(timestamps, err_f_vals, 'r-', linewidth=0.8, alpha=0.6, label='Error Filtered (°C)')
        ax6b.set_ylabel('Error (°C)', color='red')
        ax6b.tick_params(axis='y', labelcolor='red')

        lines1, labels1 = ax6.get_legend_handles_labels()
        lines2, labels2 = ax6b.get_legend_handles_labels()
        ax6.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
        
        ax6.grid(True, alpha=0.3)
        format_x_date(ax6)
        
        fig6.tight_layout()
        pdf.savefig(fig6, bbox_inches='tight')
        plt.close(fig6)
    
    if verbose:
        print(f"[DEBUG] PDF saved: {pdf_path}")
    
    return pdf_path


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="SmartPI Algorithm Analysis Report Generator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python smartpi_report.py --url http://ha:8123 --token xxx --entity climate.my_thermostat
  python smartpi_report.py --url http://ha:8123 --token xxx --entity climate.my_thermostat --days 14 --verbose
        """
    )
    parser.add_argument("--url", required=True, 
                        help="Home Assistant URL (e.g., http://homeassistant.local:8123)")
    parser.add_argument("--token", required=True, 
                        help="Long-Lived Access Token")
    parser.add_argument("--entity", required=True, 
                        help="Climate entity ID (e.g., climate.thermostat_salon)")
    parser.add_argument("--days", type=int, default=None, 
                        help="Number of days of history to fetch (default: 7, or from learning_start_dt)")
    parser.add_argument("--output-dir", default="./reports", 
                        help="Directory for output files (default: ./reports)")
    parser.add_argument("--verbose", action="store_true", 
                        help="Enable verbose debug output")

    args = parser.parse_args()

    print(f"\n{'='*70}")
    print(f"  SMARTPI ANALYSIS REPORT")
    print(f"{'='*70}\n")
    print(f"Entity:     {args.entity}")
    print(f"Output:     {args.output_dir}")
    print()

    # 1. Fetch current state to find learning_start_dt
    print(f"[INFO] Fetching current state for {args.entity}...")
    current_state = get_entity_state(args.url, args.token, args.entity, args.verbose)
    
    attrs = current_state.get("attributes", {})
    specific_states = attrs.get("specific_states", {})
    # Handle case where specific_states might be None or empty
    if not specific_states: 
        specific_states = {}
        
    smart_pi = specific_states.get("smart_pi", {}) if specific_states else {}
    learning_start_dt_str = smart_pi.get("learning_start_dt")
    
    start_date = None
    learning_start_dt = None

    if learning_start_dt_str:
        try:
            # Parse the datetime string
            learning_start_dt = datetime.fromisoformat(learning_start_dt_str)
            # If naive, assume UTC as HA usually works in UTC internally, or handle as needed
            if learning_start_dt.tzinfo is None:
                learning_start_dt = learning_start_dt.replace(tzinfo=timezone.utc)
        except ValueError:
            print(f"[WARNING] Could not parse learning_start_dt: {learning_start_dt_str}")

    # Determine start date and period
    days_arg = args.days
    
    if days_arg is not None:
        print(f"[INFO] Using user-specified period: {days_arg} days")
        days = days_arg
    elif learning_start_dt:
        print(f"[INFO] Found SmartPI learning start date: {learning_start_dt.strftime('%Y-%m-%d %H:%M:%S')}")
        start_date = learning_start_dt
        # Calculate approximate days for display/logging
        diff = datetime.now(timezone.utc) - start_date
        days = diff.days + 1
    else:
        print(f"[INFO] No learning start date found, using default 7 days")
        days = 7

    # Fetch history
    print("[INFO] Fetching history from Home Assistant...")
    # helper to ensure we don't pass both if we don't want to, but our function handles it
    # We pass start_date if we have it (from learning), or days if we have it (from args or default)
    
    history_start_arg = start_date if (args.days is None and start_date) else None
    
    history = fetch_history(args.url, args.token, args.entity, days=days_arg if days_arg else days, start_date=history_start_arg, verbose=args.verbose)

    # Extract data
    print("[INFO] Extracting SmartPI data...")
    data, summary = extract_smartpi_data(history, args.verbose)
    
    # Add learning start date to summary for report
    if learning_start_dt:
        summary['learning_start_dt'] = learning_start_dt

    if not data:
        print("\n[ERROR] No SmartPI data found. Exiting.")
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
