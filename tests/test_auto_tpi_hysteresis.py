import sys
import os
import asyncio
import logging
import shutil
from datetime import datetime
from unittest.mock import MagicMock

# Add project root to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, DataPoint

# Configure logging
logging.basicConfig(level=logging.INFO)

async def test_hysteresis_continuity():
    print("Starting Hysteresis Continuity Test...")
    
    # Mock Home Assistant
    hass = MagicMock()
    # Use absolute path to avoid empty dirname issues
    storage_path = os.path.abspath("test_hysteresis_storage.json")
    hass.config.path.return_value = storage_path
    
    # Cleanup previous run
    if os.path.exists(storage_path):
        os.remove(storage_path)
    
    # Fix for await hass.async_add_executor_job
    async def mock_async_add_executor_job(func, *args, **kwargs):
        return func(*args, **kwargs)
    
    hass.async_add_executor_job.side_effect = mock_async_add_executor_job
    
    # Initialize Manager with hysteresis thresholds
    # High = 1.0, Low = -1.0
    manager = AutoTpiManager(hass, "test_hysteresis", cycle_min=5, 
                             tpi_threshold_high=1.0, tpi_threshold_low=-1.0)
    await manager.start_learning()
    
    target_temp = 20.0
    
    # Helper to add point
    async def add_point(room_temp, time_offset_sec=0):
        # We need to manually advance time because update checks for min_interval
        # Force last_update to be old enough if needed, or just mock time
        # Here we just manually append to _data for direct logic testing if update() has checks
        # But better to use update() to test full integration if possible.
        # update() has a check: (now - self._last_update).total_seconds() < min_interval
        # min_interval = cycle_min * 60 * 0.1 = 5 * 60 * 0.1 = 30 seconds
        
        # We will bypass update() validation for speed and testing logic directly
        # or we mock datetime. But let's call update() and ensure we sleep or mock time?
        # Mocking time is complex without freezegun.
        # Let's bypass update() wrapper and call logic methods or just insert data and call compute.
        
        # Actually, let's just insert into _data and call _compute_hysteresis_incremental manually
        # This tests the logic unit specifically.
        
        point = DataPoint(
            timestamp=datetime.now().timestamp() + time_offset_sec,
            room_temp=room_temp,
            ext_temp=10.0,
            power_percent=50.0,
            target_temp=target_temp,
            is_heating=True
        )
        manager._data.append(point)
        manager._compute_hysteresis_incremental()
        return point

    print("Step 1: Initial point (Diff=0)")
    p1 = await add_point(20.0)
    # Default is False. Diff = 0. Not > High (1), Not < Low (-1). Keeps False.
    if not p1.is_gated_off:
        print("PASS: p1 is NOT gated off")
    else:
        print(f"FAIL: p1 is gated off (Diff={p1.room_temp - target_temp})")

    print("Step 2: High overshoot (Diff=2.0)")
    p2 = await add_point(22.0, 10)
    # Diff = 2.0 > 1.0 -> True
    if p2.is_gated_off:
        print("PASS: p2 IS gated off")
    else:
        print(f"FAIL: p2 is NOT gated off (Diff={p2.room_temp - target_temp})")

    print("Step 3: In between (Diff=0.5)")
    p3 = await add_point(20.5, 20)
    # Diff = 0.5. Not > 1.0, Not < -1.0. Should maintain True (from p2)
    if p3.is_gated_off:
        print("PASS: p3 IS gated off (maintained)")
    else:
        print(f"FAIL: p3 is NOT gated off (Diff={p3.room_temp - target_temp})")

    print("Step 4: Low undershoot (Diff=-2.0)")
    p4 = await add_point(18.0, 30)
    # Diff = -2.0 < -1.0 -> False
    if not p4.is_gated_off:
        print("PASS: p4 is NOT gated off")
    else:
        print(f"FAIL: p4 IS gated off (Diff={p4.room_temp - target_temp})")

    print("Step 5: In between (Diff=-0.5)")
    p5 = await add_point(19.5, 40)
    # Diff = -0.5. Should maintain False (from p4)
    if not p5.is_gated_off:
        print("PASS: p5 is NOT gated off (maintained)")
    else:
        print(f"FAIL: p5 IS gated off (Diff={p5.room_temp - target_temp})")

    print("\nStarting Persistence Test...")
    # Save state
    await manager.async_save_data()
    
    # Verify file exists
    if os.path.exists(storage_path):
        print("PASS: Storage file created")
    else:
        print("FAIL: Storage file NOT created")
        return

    # Create new manager and load
    manager2 = AutoTpiManager(hass, "test_hysteresis", cycle_min=5, 
                             tpi_threshold_high=1.0, tpi_threshold_low=-1.0)
    
    # Manually trigger load since it's usually called by init or setup
    await manager2.async_load_data()
    
    print(f"Loaded {len(manager2._data)} points. Last idx: {manager2._last_hysteresis_idx}")
    
    # Verify last point state
    if manager2._data[-1].is_gated_off == False: # p5 was False
        print("PASS: Last point state restored correctly")
    else:
        print("FAIL: Last point state incorrect after load")

    # Verify internal state index
    if manager2._last_hysteresis_idx == 4: # 0-based index for 5 points
        print("PASS: _last_hysteresis_idx restored correctly")
    else:
        print(f"FAIL: _last_hysteresis_idx is {manager2._last_hysteresis_idx}, expected 4")

    # Add new point to new manager
    print("Step 6: New point after load (Diff=2.0)")
    # We need to manually append to this manager too
    p6 = DataPoint(
            timestamp=datetime.now().timestamp() + 50,
            room_temp=22.0,
            ext_temp=10.0,
            power_percent=50.0,
            target_temp=target_temp,
            is_heating=True
        )
    manager2._data.append(p6)
    manager2._compute_hysteresis_incremental()
    
    if p6.is_gated_off:
        print("PASS: p6 IS gated off (Logic continued correctly)")
    else:
        print("FAIL: p6 is NOT gated off")

    # Clean up
    if os.path.exists(storage_path):
        os.remove(storage_path)

if __name__ == "__main__":
    asyncio.run(test_hysteresis_continuity())