import unittest
from unittest.mock import MagicMock
from datetime import datetime

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, DataPoint

class TestCompression(unittest.TestCase):

    def setUp(self):
        self.hass = MagicMock()
        self.hass.config.path.return_value = "dummy_path.json"
        self.hass.async_add_executor_job = MagicMock()
        
        self.manager = AutoTpiManager(self.hass, "test_compression", cycle_min=5)

    def _create_points(self, count: int, start_idx: int = 0):
        """Helper to create a list of data points."""
        points = []
        now = datetime.now().timestamp()
        for i in range(count):
            # timestamp increases by 1s
            dp = DataPoint(
                timestamp=now + i,
                room_temp=20.0,
                ext_temp=10.0,
                power_percent=float(start_idx + i), # Use power as index marker
                target_temp=21.0,
                is_heating=True
            )
            points.append(dp)
        return points

    def test_no_compression_small_data(self):
        """Test that no compression happens if data <= 500 points."""
        self.manager._data = self._create_points(100)
        compressed = self.manager._compress_historical_data()
        
        self.assertEqual(len(compressed), 100)
        self.assertEqual(compressed, self.manager._data)

    def test_compression_logic(self):
        """Test the compression logic (keep recent 500, downsample older)."""
        # Create 1000 points
        # Older: 500 points (indices 0-499)
        # Recent: 500 points (indices 500-999)
        self.manager._data = self._create_points(1000)
        
        compressed = self.manager._compress_historical_data()
        
        # Expected length:
        # Older 500 -> 1 out of 5 -> 100 points
        # Recent 500 -> kept all -> 500 points
        # Total = 600
        self.assertEqual(len(compressed), 600)
        
        # Verify recent data is intact (last 500 points)
        # Their power_percent should be 500.0 to 999.0
        recent_start_val = compressed[-500].power_percent
        recent_end_val = compressed[-1].power_percent
        self.assertEqual(recent_start_val, 500.0)
        self.assertEqual(recent_end_val, 999.0)
        
        # Verify older data is downsampled
        # First point should be 0.0
        # Second point should be 5.0 (step 5)
        self.assertEqual(compressed[0].power_percent, 0.0)
        self.assertEqual(compressed[1].power_percent, 5.0)
        
        # Check transition point
        # The last compressed older point should be index 495
        # The first recent point is index 500
        # compressed[99] should be 495.0
        # compressed[100] should be 500.0
        self.assertEqual(compressed[99].power_percent, 495.0)
        self.assertEqual(compressed[100].power_percent, 500.0)

    def test_compression_boundary(self):
        """Test boundary condition (exactly 505 points)."""
        # 5 older, 500 recent
        self.manager._data = self._create_points(505)
        
        compressed = self.manager._compress_historical_data()
        
        # Older 5 -> 1 point (index 0)
        # Recent 500 -> 500 points
        # Total 501
        self.assertEqual(len(compressed), 501)
        self.assertEqual(compressed[0].power_percent, 0.0)
        self.assertEqual(compressed[1].power_percent, 5.0)

if __name__ == "__main__":
    unittest.main()