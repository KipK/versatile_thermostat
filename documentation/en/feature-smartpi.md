# The SmartPI Algorithm

- [The SmartPI Algorithm](#the-smartpi-algorithm)
  - [How it works](#how-it-works)
  - [Operating Phases](#operating-phases)
  - [Advanced Features](#advanced-features)
  - [Configuration](#configuration)
  - [Diagnostic Metrics](#diagnostic-metrics)
  - [Services](#services)

## How it works

The **SmartPI** algorithm is an adaptive controller that automatically learns the thermal behavior of your room. It is designed to replace the hassle of manually tuning PID/TPI coefficients with a self-learning approach.

### How does it work?

1.  **Continuous Learning (Heartbeat)**: SmartPI analyzes the room's response continuously (every minute) via a sliding window, without waiting for heating cycles to complete.
2.  **Thermal modeling**: It builds a robust internal model (via Median & MAD statistical method) characterized by:
    *   **a** (Efficiency): Temperature gain per minute at 100% power.
    *   **b** (Heat loss): Temperature loss per minute per degree of difference with the outside.
    *   **L** (Dead Time): Delay between the heater start and the actual temperature rise.
3.  **Gain adaptation**: Coefficients (Kp, Ki) are dynamically recalculated based on the room's inertia (Tau) and the detected dead time.

## Operating Phases

### Phase 1: Hysteresis (Bootstrap and Initial Learning)

At the very first startup (or after a learning reset), the thermal model is empty. To ensure immediate comfort while generating quality learning data, SmartPI **must** start with a **Bootstrap** phase in **Hysteresis** mode:

*   **ON**: When the temperature drops below `Setpoint - 0.3°C`.
*   **OFF**: When the temperature rises above `Setpoint + 0.5°C`.
*   **Hold**: Between these two thresholds, the previous state is maintained.

This phase generates clear and distinct heating cycles, essential for identifying parameters `a` and `b`, but most importantly for learning the initial **Dead Time**.

> **Transition**: The algorithm automatically switches to **STABLE** phase as soon as it has collected enough reliable measurements (minimum 11 reliable measurements).
> **Note**: In Hysteresis mode, shut-off is **immediate** as soon as the temperature exceeds the upper threshold, interrupting the current PWM cycle to prevent overheating.

### Phase 2: Stable (Adaptive PI Regulation)

Once the model is reliable, SmartPI activates its advanced PI controller:

*   **Feed-Forward (Prediction)**: Calculates the base power needed to compensate for thermal losses (based on outdoor temperature).
*   **PI (Correction)**): Adds or removes power to correct the precise deviation from the setpoint.
*   **Continuous refinement**: The algorithm continues to refine its model continuously to adapt to seasonal changes or insulation (via robust Median/MAD estimation).

### Phase 3: Forced Calibration (Model Maintenance)

If the algorithm detects that its **Dead Time** data is no longer reliable or if no calibration has taken place for more than 48 hours, it can trigger a **Forced Calibration** phase.

*   The thermostat temporarily switches back to hysteresis mode to perform a full cycle (Cooling -> Heating -> Cooling).
*   This allows for precise recalibration of the system's reaction delays.
*   This phase can also be triggered manually via a service.

## Advanced Features

SmartPI introduces several refinements to improve stability and comfort:

### 1. Dead Time Estimation
SmartPI automatically detects the delay (**L**) between the command to turn on and the actual temperature reaction.
*   This allows for finer tuning rules (IMC - Internal Model Control) to avoid oscillations on systems with lag (e.g., underfloor heating, oil-filled radiators).
*   Detection is active even in **Hysteresis** mode (using natural oscillations).

### 2. Auto-Adaptive Near-Band (Auto Near-Band)
To avoid overshoots, SmartPI reduces its gains when approaching the setpoint.
*   This "smooth zone" is automatically calculated based on the room's inertia and dead time.
*   In Heating mode, this zone is asymmetric: it starts earlier "below" the setpoint for a soft landing, and tightens "above" to cut off quickly in case of overshoot.

### 3. Setpoint Boost
If you increase the setpoint by more than **0.3°C** (e.g., changing from Eco to Comfort mode), SmartPI temporarily activates a "Boost" mode:
*   The rate-limiter is relaxed to allow a rapid power ramp-up.
*   The proportional action is made more aggressive to reach the target as quickly as possible.

### 4. Asymmetric Setpoint Filter (Soft Landing)
To avoid overshooting the target during a temperature rise, SmartPI applies a smart filter to the internal setpoint:
*   **Rise**: The internal setpoint climbs progressively once past the mid-point, forcing the controller to slow down before impact.
*   **Drop**: The setpoint is followed instantly to cut heating without delay (energy saving).

### 5. Thermal Guard
If you lower the setpoint (e.g., Comfort to Eco), a "Thermal Guard" activates:
*   It prevents the integral (memory of past errors) from continuing to rise even if the temperature is still below the *old* setpoint.
*   This avoids storing "virtual heat" that would cause an overshoot once the new setpoint is reached.

## Configuration

Default parameters are suitable for most cases.

| Parameter | Description | Recommended Value |
|-----------|-------------|-------------------|
| **Deadband** | Tolerance zone around the setpoint (±X°C). | 0.05°C |
| **Setpoint Filter** | Enables "Soft Landing". | Disabled |

> **Tip**: If temperature oscillates too much, try to increase the deadband.

## Diagnostic Metrics

For advanced users, the climate entity exposes detailed attributes:

| Attribute | Description |
|-----------|-------------|
| `regulation_mode` | Current mode: `hysteresis` (learning) or `smartpi` (regulated) |
| `phase` | Current algorithm phase: `Hysteresis`, `Stable`, or `Calibration` |
| `hysteresis_state`| Hysteresis state: `on`, `off` or `band` |
| `tau_min` | Room thermal inertia (minutes). E.g., 600 = 10h |
| `deadtime_heat_s` | Estimated dead time in seconds (heating lag) |
| `deadtime_cool_s` | Estimated dead time in seconds (cooling lag) |
| `deadtime_reliable`| `true` if dead time has been correctly identified |
| `deadtime_skip_count_a` | Counter of ignored learning (parameter a) due to dead time |
| `deadtime_skip_count_b` | Counter of ignored learning (parameter b) due to dead time |
| `in_deadtime_window` | `true` if the system is currently in a dead time window |
| `a` | Heating efficiency (°C/min at 100%) |
| `b` | Loss coefficient (1/min) |
| `learn_ok_count` | Number of validated learning episodes |
| `learn_last_reason` | Reason for last learning attempt (success or rejection reason) |
| `error` | Setpoint - Temperature deviation |
| `u_ff` | "Feed-Forward" power share (weather anticipation) |
| `u_pi` | "PI" power share (error correction) |
| `Kp`, `Ki` | Calculated regulator gains |
| `Kp_reel`, `Ki_reel` | Actually applied gains (including reductions like Near-Band) |
| `kp_source` | Gain Kp source: `IMC`, `Heuristic` or `Safe` |
| `on_percent` | Target total power (0.0 to 1.0) |
| `u_applied` | Real applied power after all limitations |
| `in_deadband` | `true` if temperature is within the comfort zone (Deadband) |
| `in_near_band` | `true` if system is in the slowdown zone (Near-Band) |
| `setpoint_boost_active` | `true` if Boost mode is enabled |
| `calibration_state` | Forced calibration state: `Idle`, `CoolDown`, `HeatUp`, etc. |
| `last_calibration_time` | Date and time of the last successful calibration |
| `governance_regime` | Detected physical regime (Governance) |
| `freeze_reason_thermal` | Reason for freezing thermal parameters learning (a, b) |
| `freeze_reason_gains` | Reason for freezing gains adaptation (Kp, Ki) |


## Services

### `reset_smart_pi_learning`

Use this service if you change radiators or insulation. It resets all learned parameters (`a`, `b`, `deadtime`, etc.) to zero and forces a return to **Bootstrap / Hysteresis** phase for a fresh learning process.

### `force_smart_pi_calibration`

Forces the thermostat to enter the **Forced Calibration** phase immediately. Useful if you notice that the regulation is hunting or if the displayed dead time seems incorrect.
