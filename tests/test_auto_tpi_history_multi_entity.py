"""Test the Auto TPI History Import with multiple entities."""
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
def mock_history_data_multi():
    """Create mock history data for multiple source entities."""
    now = dt_util.now()
    start_time = now - timedelta(hours=4)
    
    # Create time steps (every 5 minutes)
    timestamps = []
    current = start_time
    while current <= now:
        timestamps.append(current)
        current += timedelta(minutes=5)
        
    climate_entity_id_1 = "climate.test_heater_1"
    climate_entity_id_2 = "climate.test_heater_2"
    room_temp_entity_id = "sensor.room_temp"
    ext_temp_entity_id = "sensor.ext_temp"
    
    history_data = {
        climate_entity_id_1: [],
        climate_entity_id_2: [],
        room_temp_entity_id: [],
        ext_temp_entity_id: []
    }
    
    # Split timeline between two entities
    # 0-2 hours: entity 1
    # 2-4 hours: entity 2
    split_idx = len(timestamps) // 2
    
    for i, ts in enumerate(timestamps):
        minute_in_hour = (ts - start_time).total_seconds() / 60 % 60
        is_heating = minute_in_hour < 30
        
        # Room temp simulation
        if is_heating:
            temp = 19.0 + (minute_in_hour / 30.0) * 2.0
        else:
            temp = 21.0 - ((minute_in_hour - 30.0) / 30.0) * 2.0
             
        # External temp
        ext_temp = 5.0
        
        history_data[room_temp_entity_id].append(
            State(room_temp_entity_id, str(temp), last_updated=ts, last_changed=ts)
        )
        
        history_data[ext_temp_entity_id].append(
            State(ext_temp_entity_id, str(ext_temp), last_updated=ts, last_changed=ts)
        )
        
        # Climate state
        power = 100.0 if is_heating else 0.0
        hvac_action = "heating" if is_heating else "idle"
        state = State(
            climate_entity_id_1 if i < split_idx else climate_entity_id_2, 
            "heat", 
            attributes={"power_percent": power, "hvac_action": hvac_action, "temperature": 20.0},
            last_updated=ts, last_changed=ts
        )
        
        if i < split_idx:
            history_data[climate_entity_id_1].append(state)
        else:
            history_data[climate_entity_id_2].append(state)
            
    return history_data

async def test_import_history_data_multi_entity(hass: HomeAssistant, mock_history_data_multi):
    """Test importing history data with a list of source entities."""
    
    manager = AutoTpiManager(
        hass=hass,
        unique_id="test_vtherm_multi",
        name="Test VTherm Multi",
        cycle_min=5, 
    )
    
    hass.config.path = MagicMock(return_value="/tmp/versatile_thermostat_test_multi.json")
    
    # Mock history.state_changes_during_period
    # It receives a list of entity_ids and returns a dict with data for all of them
    with patch(
        "homeassistant.components.recorder.history.state_changes_during_period",
        return_value=mock_history_data_multi
    ) as mock_history:
        
        result = await manager.import_history_data(
            source_climate_entity_id=["climate.test_heater_1", "climate.test_heater_2"],
            room_temp_entity_id="sensor.room_temp",
            ext_temp_entity_id="sensor.ext_temp",
            days=1
        )
        
        assert result.get("error") is None
        
        # We expect cycles to be created from both entities data
        # Total 4 hours, so ~48 cycles
        assert len(manager._completed_tpi_cycles) > 40
        
        # Check if we have data from the second half (handled by entity 2)
        # The last cycle should be close to now
        last_cycle = manager._completed_tpi_cycles[-1]
        assert last_cycle.start_time > (dt_util.now() - timedelta(hours=1)).timestamp()
        
        assert result["cycles_added"] == len(manager._completed_tpi_cycles)