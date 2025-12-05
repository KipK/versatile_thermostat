# Auto TPI Feature

> **Note**: This feature is available starting from version xxx

## Introduction

The **Auto TPI** (or self-learning) feature is a major advancement in Versatile Thermostat. It allows the thermostat to **automatically** adjust its regulation coefficients (Kp and Ki) by analyzing the thermal behavior of your room.

In TPI (Time Proportional & Integral) mode, the thermostat calculates an opening percentage or heating time based on the gap between the target and indoor temperature (`Kp`), and the influence of the outdoor temperature (`Ki`).

Finding the right coefficients (`tpi_coef_int` and `tpi_coef_ext`) is often complex and requires many trials. **Auto TPI does this for you.**

## Prerequisites

For Auto TPI to work effectively:
1.  **Reliable Temperature Sensor**: The sensor must not be directly influenced by the heat source (do not place it on the radiator!).
2.  **Outdoor Temperature Sensor**: Accurate measurement of the outdoor temperature is essential.
3.  **TPI Mode Enabled**: This feature only applies if you use the TPI algorithm (thermostat over switch, valve, or climate in TPI mode).
4.  **Correct Power Configuration**: Correctly define the parameters related to heating time (see below).

## Configuration

Configuration is done via the Home Assistant integration interface (Configure -> Modify -> Auto TPI & Learning).

### Main Parameters

| Parameter | Description |
|---|---|
| **Enable Auto TPI** | Check this box to enable learning. |
| **Calculation Method** | Choose between `Average` or `EMA` (Exponential Moving Average). **Recommended: EMA**. |
| **Indoor Coefficient Cap** | Maximum allowed value for the indoor coefficient (`Kp`). Default: 0.6. Prevents excessive oscillations. |
| **Outdoor Coefficient Cap** | Maximum value for the outdoor coefficient (`Ki`). Default: 0.04. |

### Thermal Configuration (Critical)

The algorithm needs to understand the responsiveness of your heating system.

#### `heater_heating_time` (Thermal Response Time)
This is the total time required for the system to have a measurable effect on the room temperature.

It must include:
*   The heating time of the radiator (material inertia).
*   The heat propagation time in the room to the sensor.

**Suggested Values:**

| Heater Type | Suggested Value |
|---|---|
| Electric radiator (convector), sensor nearby | 2-5 min |
| Inertia radiator (oil-filled, cast iron), sensor nearby | 5-10 min |
| Underfloor heating, or large room with distant sensor | 10-20 min |

> An incorrect value can skew the efficiency calculation and prevent learning.

#### `heater_cooling_time` (Cooling Time)
Time required for the radiator to become cold after stopping. Used to estimate if the radiator is "hot" or "cold" at the start of a cycle.

### Heating/Cooling Capacity Configuration

The algorithm uses the concept of **Capacity** (°C gained per hour at 100% power) rather than arbitrary rates.

*   **Automatic Mode (Recommended)**: Leave the `use_capacity_as_rate` option enabled (if available) or let the system automatically detect the maximum capacity during cycles at 100% power.
*   **Manual Mode**: You can manually define:
    *   `auto_tpi_heating_rate`: Heating capacity in °C/h (e.g., 1.0 = heating at full power gains 1°C per hour).
    *   `auto_tpi_cooling_rate`: Cooling capacity in °C/h.

## How it Works

Auto TPI operates cyclically:

1.  **Observation**: At each cycle (e.g., every 10 min), the thermostat measures the temperature at the start and end, as well as the power used.
2.  **Validation**: It checks if the cycle is valid for learning:
    *   Power was not saturated (between 0% and 100% excluded).
    *   The temperature difference is significant.
    *   The system is stable (no consecutive failures).
3.  **Calculation (Learning)**:
    *   **Priority 1: Indoor Coefficient (`Kp`)**. If the temperature moved in the right direction, it calculates the ratio between the real evolution and the expected theoretical evolution. It adjusts `Kp` to reduce the gap.
    *   **Priority 2: Outdoor Coefficient (`Ki`)**. If indoor learning is not possible (e.g., stable temperature but gap with target), it adjusts `Ki` to compensate for thermal losses related to the outside.
4.  **Update**: The new coefficients are smoothed (according to the chosen method) and saved. They are immediately used for the next cycle.

## Attributes and Sensors

A dedicated sensor `sensor.<thermostat_name>_auto_tpi_learning_state` allows tracking the learning status.

**Available Attributes:**

*   `active`: Learning is enabled.
*   `heating_cycles_count`: Total number of observed cycles.
*   `coeff_int_cycles`: Number of times the indoor coefficient has been adjusted.
*   `coeff_ext_cycles`: Number of times the outdoor coefficient has been adjusted.
*   `model_confidence`: Confidence index (0 to 100%) in the quality of the settings.
*   `last_learning_status`: Reason for the last success or failure (e.g., `learned_indoor_heat`, `power_out_of_range`).
*   `calculated_coef_int` / `calculated_coef_ext`: Current values of the coefficients.

## Services

### Reset Learning

If you change radiators or move the sensor, it is advisable to reset the learning.

Use the service `versatile_thermostat.set_auto_tpi_mode`:
```yaml
service: versatile_thermostat.set_auto_tpi_mode
target:
  entity_id: climate.my_thermostat
data:
  enable: true
  reset: true # Forces reset of counters and coefficients to defaults
```

Or via the Developer Tools interface.