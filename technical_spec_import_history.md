# Technical Specification: Historical Data Import for Auto TPI

## 1. Overview
This feature allows the `Versatile Thermostat` to bootstrap its learning process by importing historical data from Home Assistant's recorder. By analyzing past behavior (temperature response to heating power), the Auto TPI algorithm can calculate optimal PID coefficients immediately without waiting for days of real-time learning.

## 2. Architecture Changes

### 2.1 Modified Components
*   **`custom_components/versatile_thermostat/auto_tpi_manager.py`**:
    *   Add `import_history_data` method.
    *   Add logic to parse, resample, and convert historical states into `TpiCycle` objects.
*   **`custom_components/versatile_thermostat/base_thermostat.py`** (Context only):
    *   Will eventually need to expose a service to trigger this import (out of scope for this specific task, but assumed as the trigger).

### 2.2 Dependencies
*   `homeassistant.components.recorder.history`: For fetching historical state data.
*   `numpy`: For efficient time-grid resampling (already a dependency).

## 3. Detailed Design: `auto_tpi_manager.py`

### 3.1 Method Signature
```python
async def import_history_data(
    self, 
    source_climate_entity_id: str, 
    room_temp_entity_id: str,
    ext_temp_entity_id: str,
    humidity_entity_id: Optional[str] = None,
    days: int = 30
) -> Dict[str, Any]:
```
*   **Returns**: A summary dict (e.g., `{"cycles_added": 50, "data_points": 5000}`).
*   **Note**: We pass sensor IDs explicitly to decouple dependency on `base_thermostat` structure inside the manager, though they could be passed from the thermostat instance calling this method.

### 3.2 Algorithm Steps

#### Step 1: Data Fetching
Use `history.get_significant_states` (or `state_changes_during_period`) to fetch data for all entities in one query to ensure time alignment.

*   **Period**: `now() - timedelta(days=days)` to `now()`.
*   **Entities**: `[source_climate_entity_id, room_temp_entity_id, ext_temp_entity_id]` + `[humidity_entity_id]` (if valid).
*   **Configuration**: `minimal_response=True`, `no_attributes=False` (we need attributes for power).

#### Step 2: Data Resampling (Alignment)
Raw history data is asynchronous (events occur at different times). We must align them onto a common time grid to create valid `DataPoint` snapshots.

1.  **Time Grid**: Create a numpy array of timestamps from Start to End with a step of 5 minutes (`300s`).
2.  **Fill-Forward Strategy**:
    *   For each entity, sort states by time.
    *   For each timestamp `t` in the grid, find the state effective at `t` (the latest state where `state.last_updated <= t`).
    *   If no state exists before `t` (gap at start), discard the point.

#### Step 3: Attribute Extraction & normalization
For each aligned timestamp, extract values:

*   **Room Temp**: `float(state.state)`
*   **Ext Temp**: `float(state.state)`
*   **Humidity**: `float(state.state)` (or `None`)
*   **Power**:
    *   Check attribute `power_percent` on the climate entity.
    *   **Fallback**: If `power_percent` is missing, check `hvac_action`.
        *   `heating` -> 100.0
        *   `off` / `idle` -> 0.0
    *   **Fallback 2**: If `hvac_action` missing, check `state`.
        *   `heat` -> 100.0 (assuming simplistic on/off thermostat behavior implies full power when active) -> *Wait, `state` 'heat' just means mode. We need `hvac_action` for actual activity. If `hvac_action` is not available, this data is likely useless for learning dynamics unless we assume `state=heat` means heating, which is risky.* -> **Decision**: Rely on `power_percent` attribute or `hvac_action`.

#### Step 4: Cycle Construction
1.  Group the aligned `DataPoints` into chunks of duration `self._cycle_min`.
2.  For each chunk:
    *   Create a `TpiCycle`.
    *   Set `start_time` = timestamp of first point.
    *   Set `duration_target` = `cycle_min`.
    *   Set `data_points` = list of `DataPoint`.
    *   Set `end_time` = timestamp of last point.
3.  **Filter**:
    *   Discard cycles with too many `None` / missing values.
    *   Discard cycles where `room_temp` or `ext_temp` are static for unlikely long periods (optional validity check).

#### Step 5: Integration & Calculation
1.  **Merge**: Append valid `TpiCycle` objects to `self._completed_tpi_cycles`.
2.  **Sort**: Ensure `self._completed_tpi_cycles` is sorted by time.
3.  **Prune**: Keep only the last `MAX_STORED_CYCLES`.
4.  **Save**: Call `self.async_save_data()`.
5.  **Calculate**: Trigger `self.calculate()` to update coefficients immediately.

## 4. Implementation Details

### 4.1 Handling "Unavailable" Data
*   If any sensor (Room/Ext) is `unavailable` or `unknown` at a grid point, that point cannot be used.
*   If a cycle has > 10% invalid points, discard the entire cycle to avoid polluting the learning model.

### 4.2 Code Structure (Snippet)

```python
# Pseudo-code for resampling
timestamps = np.arange(start_ts, end_ts, 300)
aligned_data = []

# Pre-fetch all states
history_data = await get_instance(self._hass).async_add_executor_job(
    history.state_changes_during_period, ...
)

for ts in timestamps:
    # Find latest state for each entity
    room_state = find_state_at(history_data[room_id], ts)
    ext_state = find_state_at(history_data[ext_id], ts)
    climate_state = find_state_at(history_data[climate_id], ts)
    
    if not valid(room_state, ext_state, climate_state):
        continue
        
    # Extract power
    power = extract_power(climate_state)
    
    # Create point
    dp = DataPoint(...)
    aligned_data.append(dp)
```

## 5. Constraints & Edge Cases
*   **Memory**: Fetching 30 days of history can be heavy.
    *   *Mitigation*: We might need to fetch in chunks (e.g., 7 days at a time) or rely on HA's database efficiency. Given we filter specific entities, it should be manageable.
*   **Sensor Noise**: The existing `AutoTpiManager` has outlier detection. We should run the imported points through a similar check or reuse `_is_outlier` if applicable, though `_is_outlier` relies on a sliding window of *stored* data. When batch importing, we should construct the window dynamically.
*   **Power Attribute**: Crucial dependency. If the user points to a generic thermostat that doesn't expose power or accurate action state, the learned model will be garbage (GIGO).

## 6. Next Steps
1.  Implement `import_history_data` in `auto_tpi_manager.py`.
2.  Expose service `versatile_thermostat.import_history` in `base_thermostat.py` (or `climate.py` platform setup) that calls this method.