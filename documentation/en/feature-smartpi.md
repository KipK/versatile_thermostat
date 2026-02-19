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

> **Transition**: The algorithm automatically switches to **STABLE** phase as soon as it has collected enough reliable measurements (minimum 31 measurements).
> **Note**: In Hysteresis mode, shut-off is **immediate** as soon as the temperature exceeds the upper threshold, interrupting the current PWM cycle to prevent overheating.

### Phase 2: Stable (Adaptive PI Regulation)

Once the model is reliable, SmartPI activates its advanced PI controller:

*   **Feed-Forward (Prediction)**: Calculates the base power needed to compensate for thermal losses (based on outdoor temperature).
*   **PI (Correction)**: Adds or removes power to correct the precise deviation from the setpoint.
*   **Continuous refinement**: The algorithm continues to refine its model continuously to adapt to seasonal changes or insulation (via robust Median/MAD estimation).

### Phase 3: Automated Calibration (Maintenance)

SmartPI doesn't just learn once; it constantly monitors the quality of its own model. If it detects that the learning has stagnated or that the parameters (heating/cooling delays) are no longer consistent with reality, it triggers an **Automated Calibration**.

#### 1. The Snapshot System
As soon as the algorithm becomes stable for the first time, it takes a "Snapshot" of its reference parameters. This baseline is used as a point of comparison to detect any future drift.

#### 2. Continuous Monitoring (Supervision)
Every hour, the **AutoCalibTrigger** supervisor evaluates several criteria:
*   **Stagnation**: Is the algorithm failing to acquire new reliable data?
*   **Quality**: Are the estimates (a, b) showing high statistical dispersion?
*   **Reliability**: Are the dead times (reaction delays) still marked as reliable?
*   **Timer**: A rolling snapshot is taken every 5 days to keep the baseline up to date.

#### 3. The Calibration Cycle
If a problem is detected, or if a manual calibration is requested, the system enters a 3-step cycle in **Hysteresis** mode:
1.  **Cool Down**: The heating is cut until the temperature drops to `Setpoint - 0.3°C`.
2.  **Heat Up**: Heating is forced at 100% until `Setpoint + 0.5°C`. This captures the **Heating Dead Time**.
3.  **Cool Down Final**: Heating is cut again until the lower threshold. This captures the **Cooling Dead Time**.

#### 4. Post-Calibration Validation
Once the cycle is complete, the supervisor checks if the model has improved.
*   **Success**: A new snapshot is taken, and the system returns to **STABLE** mode.
*   **Retry**: If the results are poor, a new attempt is scheduled after a few hours of rest.
*   **Degraded Mode**: After 3 failed attempts, the system continues to operate but signals a "degraded model" state in its diagnostics.

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
| `tau_reliable` | `true` if the inertia estimate is reliable |
| `a` | Heating efficiency (°C/min at 100%) |
| `b` | Loss coefficient (1/min) |
| `learn_ok_count` | Total number of validated learning episodes |
| `learn_ok_count_a` | Number of validated learning episodes for parameter `a` |
| `learn_ok_count_b` | Number of validated learning episodes for parameter `b` |
| `learn_last_reason` | Reason for last learning attempt (success or rejection reason) |
| `error` | Setpoint - Temperature deviation |
| `u_ff` | "Feed-Forward" power share (weather anticipation) |
| `ff_raw` | Raw Feed-Forward power before scaling (0.0 to 1.0) |
| `ff_reason` | Reason for current Feed-Forward state/scaling |
| `ff_scale` | Dynamic scaling factor for Feed-Forward (0.0=off, 1.0=full) |
| `ff_H_inertia_s` | Inertia buffering duration for FF smoothing (seconds) |
| `ff_d_inertia_deg` | Inertia buffering temperature delta for FF smoothing (°C) |
| `u_pi` | "PI" power share (error correction) |
| `Kp`, `Ki` | Calculated regulator gains |
| `kp_source` | Gain Kp source: `imc_deadtime`, `heuristic`, `safe`, `frozen`, etc. |
| `on_percent` | Target total power (0.0 to 1.0) |
| `u_applied` | Real applied power after all limitations |
| `in_deadband` | `true` if temperature is within the comfort zone (Deadband) |
| `in_near_band` | `true` if system is in the slowdown zone (Near-Band) |
| `near_band_below_deg` | Near-Band width below setpoint (°C, auto-calculated) |
| `near_band_above_deg` | Near-Band width above setpoint (°C, auto-calculated) |
| `near_band_source` | Near-Band calculation source: `auto_model_aware`, `manual`, etc. |
| `setpoint_boost_active` | `true` if Boost mode is enabled |
| `deadtime_heat_s` | Estimated dead time in seconds (heating lag) |
| `deadtime_heat_reliable` | `true` if heating dead time has been correctly identified |
| `deadtime_cool_s` | Estimated dead time in seconds (cooling lag) |
| `deadtime_cool_reliable` | `true` if cooling dead time has been correctly identified |
| `in_deadtime_window` | `true` if the system is currently in a dead time window |
| `governance_regime` | Detected physical regime (Governance) |
| `governance_cycle_regimes` | List of regimes traversed during the current cycle |
| `freeze_reason_thermal` | Reason for freezing thermal parameters learning (a, b) |
| `freeze_reason_gains` | Reason for freezing gains adaptation (Kp, Ki) |
| `last_decision_thermal` | Governance decision for thermal learning |
| `last_decision_gains` | Governance decision for gains adaptation |
| `calibration_state` | Current calibration state: `Idle`, `CoolDown`, `HeatUp`, `CoolDownFinal` |
| `last_calibration_time` | Timestamp of last successful calibration |
| `calibration_retry_count` | Number of calibration retries |
| `autocalib_last_trigger_ts` | Last time an automatic calibration was triggered |
| `autocalib_next_check_ts` | Next scheduled verification of the time constant (tau) |
| `autocalib_snapshot_age_h` | Age of the reference baseline snapshot in hours |


## Services

### `reset_smart_pi_learning`

Use this service if you change radiators or insulation. It resets all learned parameters (`a`, `b`, `deadtime`, etc.) to zero and forces a return to **Bootstrap / Hysteresis** phase for a fresh learning process.

### `force_smart_pi_calibration`

Forces the thermostat to enter the **Forced Calibration** phase immediately. Useful if you notice that the regulation is hunting or if the displayed dead time seems incorrect.
