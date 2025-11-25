import unittest
import asyncio
from unittest.mock import MagicMock
import numpy as np
from datetime import datetime

from custom_components.versatile_thermostat.auto_tpi_manager import AutoTpiManager, TpiCycle, DataPoint

class TestConceptDrift(unittest.TestCase):

    def setUp(self):
        self.hass = MagicMock()
        self.hass.config.path.return_value = "dummy_path.json"
        self.hass.async_add_executor_job = MagicMock()
        
        # Mock executor job to run immediately
        async def mock_async_add_executor_job(func, *args, **kwargs):
            return func(*args, **kwargs)
        self.hass.async_add_executor_job.side_effect = mock_async_add_executor_job

        self.manager = AutoTpiManager(self.hass, "test_drift", cycle_min=5)

    def _create_cycle(self, temp_diff: float) -> TpiCycle:
        """Helper to create a cycle with a specific temp evolution."""
        start_ts = datetime.now().timestamp()
        p1 = DataPoint(start_ts, 20.0, 10.0, 50.0, 21.0, True)
        p2 = DataPoint(start_ts + 300, 20.0 + temp_diff, 10.0, 50.0, 21.0, True)
        return TpiCycle(start_ts, 300, [p1, p2], start_ts + 300)

    def test_insufficient_data(self):
        """Test that detection returns False if not enough cycles."""
        # Add 39 cycles
        for _ in range(39):
            self.manager._completed_tpi_cycles.append(self._create_cycle(0.5))
        
        self.assertFalse(self.manager._detect_concept_drift())

    def test_no_drift(self):
        """Test stable conditions (no drift)."""
        # Create 40 cycles with same distribution (mean=0.5, small std)
        np.random.seed(42)
        diffs = np.random.normal(loc=0.5, scale=0.05, size=40)
        
        for d in diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
            
        self.assertFalse(self.manager._detect_concept_drift())

    def test_drift_detected(self):
        """Test drift detection with shift in mean."""
        np.random.seed(42)
        
        # Older: 20 cycles with mean 0.5
        older_diffs = np.random.normal(loc=0.5, scale=0.05, size=20)
        for d in older_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
            
        # Recent: 20 cycles with mean 1.0 (significant shift)
        recent_diffs = np.random.normal(loc=1.0, scale=0.05, size=20)
        for d in recent_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
            
        self.assertTrue(self.manager._detect_concept_drift())

    def test_calculate_stops_on_drift(self):
        """Test that calculate returns None if drift is detected."""
        # Setup drift condition
        np.random.seed(42)
        older_diffs = np.random.normal(loc=0.5, scale=0.05, size=20)
        recent_diffs = np.random.normal(loc=1.5, scale=0.05, size=20) # Huge shift
        
        for d in older_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
        for d in recent_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
            
        # Verify drift is detected directly first
        self.assertTrue(self.manager._detect_concept_drift())
        
        # Run calculate
        result = asyncio.run(self.manager.calculate())
        self.assertIsNone(result)

    def test_noise_tolerance(self):
        """Test that slightly higher noise doesn't trigger false positive."""
        np.random.seed(42)
        
        # Older: mean 0.5
        older_diffs = np.random.normal(loc=0.5, scale=0.1, size=20)
        
        # Recent: mean 0.6 (small shift), same scale
        recent_diffs = np.random.normal(loc=0.6, scale=0.1, size=20)
        
        for d in older_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
        for d in recent_diffs:
            self.manager._completed_tpi_cycles.append(self._create_cycle(d))
            
        # Should not be detected (delta < 2 sigma)
        # pooled sigma approx 0.1
        # diff 0.1
        # delta_norm approx 1.0 < 2.0
        self.assertFalse(self.manager._detect_concept_drift())

if __name__ == "__main__":
    unittest.main()