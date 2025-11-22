import sys
import os
import types
from unittest.mock import MagicMock

# Create a full mock structure for Home Assistant
def create_mock_module(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod

# 1. Mock 'homeassistant' and its submodules
if "homeassistant" not in sys.modules:
    ha = create_mock_module("homeassistant")
    ha.core = create_mock_module("homeassistant.core")
    ha.const = create_mock_module("homeassistant.const")
    ha.helpers = create_mock_module("homeassistant.helpers")
    ha.helpers.event = create_mock_module("homeassistant.helpers.event")
    ha.helpers.config_validation = create_mock_module("homeassistant.helpers.config_validation")
    ha.config_entries = create_mock_module("homeassistant.config_entries")
    ha.components = create_mock_module("homeassistant.components")
    climate = create_mock_module("homeassistant.components.climate")
    ha.components.climate = climate
    climate.const = create_mock_module("homeassistant.components.climate.const")
    ha.exceptions = create_mock_module("homeassistant.exceptions")
    ha.util = create_mock_module("homeassistant.util")
    ha.util.dt = create_mock_module("homeassistant.util.dt")

# 2. Populate constants
sys.modules["homeassistant.const"].CONF_NAME = "name"
sys.modules["homeassistant.const"].SERVICE_RELOAD = "reload"
sys.modules["homeassistant.const"].EVENT_HOMEASSISTANT_START = "homeassistant_start"
sys.modules["homeassistant.const"].EVENT_HOMEASSISTANT_STARTED = "homeassistant_started"
sys.modules["homeassistant.const"].Platform = MagicMock()
sys.modules["homeassistant.const"].STATE_UNAVAILABLE = "unavailable"
sys.modules["homeassistant.const"].STATE_UNKNOWN = "unknown"
sys.modules["homeassistant.components.climate.const"].ClimateEntityFeature = MagicMock()
sys.modules["homeassistant.components.climate.const"].HVACMode = MagicMock()
# Mock HVACMode enum values as simple strings or MagicMock objects
sys.modules["homeassistant.components.climate.const"].HVACMode.OFF = "off"
sys.modules["homeassistant.components.climate.const"].HVACMode.HEAT = "heat"
sys.modules["homeassistant.components.climate.const"].HVACMode.COOL = "cool"
sys.modules["homeassistant.components.climate.const"].HVACMode.AUTO = "auto"
sys.modules["homeassistant.components.climate.const"].HVACMode.DRY = "dry"
sys.modules["homeassistant.components.climate.const"].HVACMode.FAN_ONLY = "fan_only"
sys.modules["homeassistant.components.climate.const"].HVACMode.HEAT_COOL = "heat_cool"

sys.modules["homeassistant.components.climate.const"].PRESET_NONE = "none"
sys.modules["homeassistant.components.climate.const"].PRESET_ECO = "eco"
sys.modules["homeassistant.components.climate.const"].PRESET_COMFORT = "comfort"
sys.modules["homeassistant.components.climate.const"].PRESET_BOOST = "boost"
sys.modules["homeassistant.components.climate.const"].PRESET_ACTIVITY = "activity"
sys.modules["homeassistant.components.climate.const"].PRESET_AWAY = "away"
sys.modules["homeassistant.components.climate.const"].PRESET_HOME = "home"
sys.modules["homeassistant.components.climate.const"].PRESET_SLEEP = "sleep"

sys.modules["homeassistant.exceptions"].HomeAssistantError = Exception
sys.modules["homeassistant.config_entries"].ConfigEntry = MagicMock()
sys.modules["homeassistant.config_entries"].ConfigType = dict

# 3. Define the MockHomeAssistant class
class MockConfig:
    """Mock Home Assistant Config."""
    def path(self, path):
        # Create a local storage directory for simulation artifacts
        base_dir = os.path.join(os.getcwd(), "auto_tpi_sim", "storage")
        os.makedirs(base_dir, exist_ok=True)
        return os.path.join(base_dir, os.path.basename(path))

class MockHomeAssistant:
    """Mock Home Assistant Core Object."""
    def __init__(self):
        self.config = MockConfig()
        # Create a bus mock
        self.bus = MagicMock()
        self.bus.fire = MagicMock()
        self.data = {}
    
    async def async_add_executor_job(self, target, *args):
        # In this simulation, we execute "executor jobs" synchronously
        return target(*args)

# 4. Assign the mock class to the module
sys.modules["homeassistant.core"].HomeAssistant = MockHomeAssistant
sys.modules["homeassistant.core"].CoreState = MagicMock()
sys.modules["homeassistant.core"].Event = MagicMock()
sys.modules["homeassistant.core"].State = MagicMock()
sys.modules["homeassistant.core"].CALLBACK_TYPE = MagicMock()
sys.modules["homeassistant.core"].callback = lambda x: x  # Mock callback decorator

# 5. Additional mocks needed for imports
sys.modules["homeassistant.helpers.service"] = MagicMock()
sys.modules["homeassistant.helpers.entity"] = MagicMock()
sys.modules["homeassistant.helpers.entity"].Entity = MagicMock()
sys.modules["homeassistant.helpers.restore_state"] = MagicMock()
sys.modules["homeassistant.helpers.restore_state"].RestoreEntity = MagicMock()
sys.modules["homeassistant.helpers.device_registry"] = MagicMock()
sys.modules["homeassistant.helpers.device_registry"].DeviceInfo = MagicMock()
sys.modules["homeassistant.helpers.device_registry"].DeviceEntryType = MagicMock()
sys.modules["homeassistant.helpers.entity_platform"] = MagicMock()
sys.modules["homeassistant.helpers.entity_component"] = MagicMock()
sys.modules["homeassistant.helpers.reload"] = MagicMock()

# Event helpers
sys.modules["homeassistant.helpers.event"] = MagicMock()
sys.modules["homeassistant.helpers.event"].async_track_state_change_event = MagicMock()
sys.modules["homeassistant.helpers.event"].async_track_time_interval = MagicMock()
sys.modules["homeassistant.helpers.event"].async_call_later = MagicMock()

# Components
sys.modules["homeassistant.components.sensor"] = MagicMock()
sys.modules["homeassistant.components.binary_sensor"] = MagicMock()
sys.modules["homeassistant.components.number"] = MagicMock()
sys.modules["homeassistant.components.switch"] = MagicMock()
sys.modules["homeassistant.components.input_boolean"] = MagicMock()
sys.modules["homeassistant.components.input_number"] = MagicMock()
sys.modules["homeassistant.components.select"] = MagicMock()
sys.modules["homeassistant.components.input_select"] = MagicMock()
sys.modules["homeassistant.components.input_datetime"] = MagicMock()
sys.modules["homeassistant.components.person"] = MagicMock()

# Climate consts
sys.modules["homeassistant.components.climate.const"].ATTR_PRESET_MODE = "preset_mode"
sys.modules["homeassistant.components.climate.const"].ATTR_FAN_MODE = "fan_mode"
sys.modules["homeassistant.components.climate.const"].ATTR_HVAC_MODE = "hvac_mode"
sys.modules["homeassistant.components.climate.const"].ATTR_SWING_MODE = "swing_mode"
sys.modules["homeassistant.components.climate.const"].ATTR_CURRENT_TEMPERATURE = "current_temperature"
sys.modules["homeassistant.components.climate.const"].ATTR_CURRENT_HUMIDITY = "current_humidity"
sys.modules["homeassistant.components.climate.const"].ATTR_MIN_TEMP = "min_temp"
sys.modules["homeassistant.components.climate.const"].ATTR_MAX_TEMP = "max_temp"
sys.modules["homeassistant.components.climate.const"].ATTR_TARGET_TEMP_HIGH = "target_temp_high"
sys.modules["homeassistant.components.climate.const"].ATTR_TARGET_TEMP_LOW = "target_temp_low"
sys.modules["homeassistant.components.climate.const"].ATTR_TARGET_TEMPERATURE_STEP = "target_temp_step"

sys.modules["homeassistant.components.climate"].ClimateEntity = MagicMock()
sys.modules["homeassistant.components.climate"].HVACAction = MagicMock()
sys.modules["homeassistant.components.climate"].HVACAction.HEATING = "heating"
sys.modules["homeassistant.components.climate"].HVACAction.COOLING = "cooling"
sys.modules["homeassistant.components.climate"].HVACAction.IDLE = "idle"
sys.modules["homeassistant.components.climate"].HVACAction.OFF = "off"

# Consts
sys.modules["homeassistant.const"].ATTR_TEMPERATURE = "temperature"
sys.modules["homeassistant.const"].ATTR_ENTITY_ID = "entity_id"
sys.modules["homeassistant.const"].STATE_ON = "on"
sys.modules["homeassistant.const"].STATE_OFF = "off"
sys.modules["homeassistant.const"].STATE_UNAVAILABLE = "unavailable"
sys.modules["homeassistant.const"].STATE_UNKNOWN = "unknown"
sys.modules["homeassistant.const"].CONF_NAME = "name"
sys.modules["homeassistant.const"].Platform = MagicMock()
# Some imports might rely on other parts of homeassistant