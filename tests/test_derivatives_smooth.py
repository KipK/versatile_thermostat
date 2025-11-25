import sys
import os
import asyncio
import logging
import numpy as np
from unittest.mock import MagicMock
from datetime import datetime

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, DataPoint

# Configure logging
logging.basicConfig(level=logging.DEBUG)

try:
    from scipy.signal import savgol_filter
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

async def test_derivatives_smooth():
    print("Starting Derivatives Smooth Test...")
    
    hass = MagicMock()
    manager = AutoTpiManager(hass, "test_derivs", cycle_min=5)
    
    # Generate synthetic data with noise
    # Signal: Sine wave + Noise
    # T(t) = 20 + sin(t/3600) -> dT/dt = cos(t/3600)/3600 K/s = cos(t/3600) K/h
    
    print(f"Scipy available: {SCIPY_AVAILABLE}")
    
    start_time = datetime.now().timestamp()
    points = []
    
    # Generate 50 points (every 60s)
    # Enough for window_length=11
    for i in range(50):
        t = i * 60
        # Clean signal
        clean_temp = 20 + np.sin(t / 3600.0)
        # Add noise (std=0.05)
        noise = np.random.normal(0, 0.05)
        temp = clean_temp + noise
        
        dp = DataPoint(
            timestamp=start_time + t,
            room_temp=temp,
            ext_temp=10.0,
            power_percent=50.0,
            target_temp=20.0,
            is_heating=True
        )
        points.append(dp)
        
    manager._data = points
    
    # Force calculation
    if SCIPY_AVAILABLE:
        print("Testing Savitzky-Golay implementation...")
        manager._compute_derivatives()
        
        # Check if derivatives are populated
        derivs = [p.temp_derivative for p in manager._data if p.temp_derivative is not None]
        print(f"Computed {len(derivs)} derivatives")
        
        if len(derivs) == 0:
            print("FAIL: No derivatives computed")
            return

        # Check values roughly match cosine
        # At t=0, deriv should be close to 1 K/h
        # At t=25*60 (1500s), cos(1500/3600) = cos(0.41) = 0.91
        
        avg_deriv = np.mean(derivs)
        print(f"Average derivative: {avg_deriv:.3f} K/h")
        
        # With noise, it fluctuates, but should be around 0.9-1.0
        if 0.5 < avg_deriv < 1.5:
            print("PASS: Derivative magnitude reasonable")
        else:
            print(f"FAIL: Derivative magnitude unexpected {avg_deriv}")
            
        # Verify it used the smooth method (indirectly via debug logs or by checking if all points have derivs)
        # Simple method skips first 'window' points. 
        # Smooth method (savgol) usually returns array of same length (if mode='interp' or similar default)
        # In implementation: self._data[i].temp_derivative = derivs_h[i] for all i
        
        none_count = sum(1 for p in manager._data if p.temp_derivative is None)
        if none_count == 0:
             print("PASS: All points have derivatives (indicates Savitzky-Golay used)")
        else:
             print(f"INFO: {none_count} points missing derivatives")

    # Test Fallback
    print("\nTesting Simple Fallback...")
    
    # Reset derivatives
    for p in manager._data:
        p.temp_derivative = None
        
    manager._compute_derivatives_simple()
    
    derivs_simple = [p.temp_derivative for p in manager._data if p.temp_derivative is not None]
    print(f"Simple computed {len(derivs_simple)} derivatives")
    
    # Simple method skips window (5)
    expected_simple = len(points) - 5
    if len(derivs_simple) == expected_simple:
        print("PASS: Simple method count correct")
    else:
        print(f"FAIL: Simple method count {len(derivs_simple)}, expected {expected_simple}")

    # Compare noise reduction (if scipy available)
    if SCIPY_AVAILABLE:
        # We need to re-run smooth
        for p in manager._data:
            p.temp_derivative = None
        manager._compute_smooth_derivatives()
        derivs_smooth = [p.temp_derivative for p in manager._data if p.temp_derivative is not None]
        
        # Calculate standard deviation of derivatives (roughness)
        std_smooth = np.std(derivs_smooth)
        
        # Run simple again
        for p in manager._data:
            p.temp_derivative = None
        manager._compute_derivatives_simple()
        derivs_simple_vals = [p.temp_derivative for p in manager._data if p.temp_derivative is not None]
        std_simple = np.std(derivs_simple_vals)
        
        print(f"StdDev Smooth: {std_smooth:.4f}")
        print(f"StdDev Simple: {std_simple:.4f}")
        
        if std_smooth < std_simple:
             print("PASS: Smooth method has lower variance (less noise)")
        else:
             print("WARN: Smooth method variance not lower (might be due to signal nature)")

if __name__ == "__main__":
    asyncio.run(test_derivatives_smooth())