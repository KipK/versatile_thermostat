"""Test the Auto TPI History Import."""
from unittest.mock import patch, MagicMock
from datetime import timedelta
import pytest
from homeassistant.core import HomeAssistant, State
from homeassistant.util import dt as dt_util

from custom_components.versatile_thermostat.auto_tpi_manager import (
    AutoTpiManager,
    DataPoint,
    TpiCycle
)

from .commons import *

@pytest.fixture
def mock_history_data():
    """Create mock history data."""
    now = dt_util.now()
    start_time = now - timedelta(hours=4)
    
    # Create time steps (every 5 minutes)
    timestamps = []
    current = start_time
    while current <= now:
        timestamps.append(current)
        current += timedelta(minutes=5)
        
    climate_entity_id = "climate.test_heater"
    room_temp_entity_id = "sensor.room_temp"
    ext_temp_entity_id = "sensor.ext_temp"
    
    history_data = {
        climate_entity_id: [],
        room_temp_entity_id: [],
        ext_temp_entity_id: []
    }
    
    # Simulate a pattern:
    # 0-30 mins: Heating ON (Power 100%), Room temp rising
    # 30-60 mins: Heating OFF (Power 0%), Room temp falling
    # Repeat
    
    for i, ts in enumerate(timestamps):
        # Determine phase (1 hour cycle)
        minute_in_hour = (ts - start_time).total_seconds() / 60 % 60
        
        is_heating = minute_in_hour < 30
        
        # Room temp simulation
        # Base 19, +2 when heating, -1 when cooling (simplified)
        base_temp = 19.0
        if is_heating:
             # Rising from 19 to 21
             temp = 19.0 + (minute_in_hour / 30.0) * 2.0
        else:
             # Falling from 21 to 19
             temp = 21.0 - ((minute_in_hour - 30.0) / 30.0) * 2.0
             
        # Add some noise
        # temp += (i % 3) * 0.1
        
        # External temp (constant-ish)
        ext_temp = 5.0 + (i / len(timestamps)) * 2.0
        
        # Power
        power = 100.0 if is_heating else 0.0
        hvac_action = "heating" if is_heating else "idle"
        
        # Create states
        history_data[room_temp_entity_id].append(
            State(room_temp_entity_id, str(temp), last_updated=ts, last_changed=ts)
        )
        
        history_data[ext_temp_entity_id].append(
            State(ext_temp_entity_id, str(ext_temp), last_updated=ts, last_changed=ts)
        )
        
        history_data[climate_entity_id].append(
            State(climate_entity_id, "heat", 
                  attributes={"power_percent": power, "hvac_action": hvac_action, "temperature": 20.0},
                  last_updated=ts, last_changed=ts)
        )
        
    return history_data

async def test_import_history_data(hass: HomeAssistant, mock_history_data):
    """Test importing history data."""
    
    manager = AutoTpiManager(
        hass=hass,
        unique_id="test_vtherm",
        name="Test VTherm",
        cycle_min=5, # 5 minutes cycles for testing? No, TPI cycles are usually longer.
                     # The default is 5 mins in config? Let's check commons.
                     # CONF_CYCLE_MIN: 5
    )
    # Actually cycle_min in manager is in minutes.
    # The import logic constructs cycles based on cycle_min.
    # If cycle_min is 5, then a 30 min heating period should result in multiple cycles?
    # Wait, TpiCycle represents a control cycle.
    # If cycle_min=5, then we expect chunks of 5 minutes.
    
    # Mock hass.config.path
    hass.config.path = MagicMock(return_value="/tmp/versatile_thermostat_test.json")
    
    # Mock history.state_changes_during_period
    with patch(
        "homeassistant.components.recorder.history.state_changes_during_period",
        return_value=mock_history_data
    ) as mock_history:
        
        result = await manager.import_history_data(
            source_climate_entity_id="climate.test_heater",
            room_temp_entity_id="sensor.room_temp",
            ext_temp_entity_id="sensor.ext_temp",
            days=1
        )
        
        assert result["error"] is None if "error" in result else True
        
        # Check if cycles were created
        # We simulated 4 hours. 
        # With cycle_min=5, we have 4 * 60 / 5 = 48 cycles roughly.
        assert len(manager._completed_tpi_cycles) > 40
        
        # Check content of a cycle
        cycle = manager._completed_tpi_cycles[0]
        assert isinstance(cycle, TpiCycle)
        assert len(cycle.data_points) > 0
        
        # Verify data points
        dp = cycle.data_points[0]
        assert isinstance(dp, DataPoint)
        assert dp.room_temp is not None
        assert dp.ext_temp is not None
        
        # Verify specific data requirements
        heating_cycles = [c for c in manager._completed_tpi_cycles if c.average_power > 0]
        assert len(heating_cycles) > 0, "Should have some heating cycles"
        
        # Check one heating cycle
        h_cycle = heating_cycles[0]
        assert h_cycle.average_power > 0
        assert h_cycle.is_complete
        
        # Let's inspect the results
        print(f"Imported {len(manager._completed_tpi_cycles)} cycles")
        print(f"Calculated params: {manager._calculated_params}")
        
        assert result["cycles_added"] == len(manager._completed_tpi_cycles)