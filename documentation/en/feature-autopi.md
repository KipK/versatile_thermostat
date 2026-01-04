# The AutoPI Algorithm

- [The AutoPI Algorithm](#the-autopi-algorithm)
  - [How it works](#how-it-works)
  - [What does it do concretely?](#what-does-it-do-concretely)
  - [Configuration](#configuration)
  - [Available parameters](#available-parameters)
  - [Recommended use cases](#recommended-use-cases)
  - [Differences with TPI and Auto-TPI](#differences-with-tpi-and-auto-tpi)

## How it works

The **AutoPI** algorithm is an adaptive controller that automatically learns the thermal behavior of your room. Unlike TPI which uses fixed coefficients, AutoPI continuously adapts to the characteristics of your installation.

### How does it work?

1. **Continuous learning**: At each heating cycle, AutoPI observes how the temperature evolves based on the applied power
2. **Thermal modeling**: It builds a mathematical model that represents your room (inertia, heat losses, heater power) via conditional EWMA estimation
3. **Gain adaptation**: The controller parameters are automatically adjusted using a heuristic based on the thermal time constant

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Temperature ──► Learning ──► Room model           │
│   Power       ──► (EWMA cond.) ──► (a, b)          │
│                         │                           │
│                         ▼                           │
│               Tau reliability check                 │
│                         │                           │
│                         ▼                           │
│               Gain calculation (Kp, Ki)             │
│               via adaptive heuristic                │
│                         │                           │
│                         ▼                           │
│               Power command (%)                     │
└─────────────────────────────────────────────────────┘
```

## What does it do concretely?

- **At startup**: AutoPI uses conservative default values
- **After a few hours**: It starts understanding your installation
- **After a few days**: The model is refined and regulation becomes optimal

### Advantages
 
 ✅ **No manual tuning**: No need to find the right Kint/Kext coefficients  
 ✅ **Adapts to changes**: If you change your heater or insulate, it re-adapts  
 ✅ **Avoids oscillations**: Automatic gain reduction near setpoint (near-band scheduling)  
 ✅ **Robust to disturbances**: The algorithm ignores sudden variations to preserve model accuracy  
 ✅ **Inertia management**: Accounts for the thermal time constant of your room  
 ✅ **Anti-overshoot**: 2-DOF PI protection and soft integral discharge (sign-flip leak)
 ✅ **Model reliability**: Uses "safe" gains and progressive feed-forward warmup
 ✅ **Anti-windup**: Conditional integration to prevent saturation

## Configuration

To enable AutoPI:

1. Create or modify a VTherm of type **switch** or **valve**
2. In **Underlyings**, select the **AutoPI** algorithm
3. Configure the parameters in the **AutoPI** menu

## Available parameters

| Parameter | Description | Default value | Recommendation |
|-----------|-------------|---------------|----------------|
| **Deadband (°C)** | Zone around the setpoint where no action is taken. Avoids micro-regulations | 0.05°C | 0.05 - 0.1°C |
| **Aggressiveness** | Response factor of the controller. Higher = more reactive (higher gains) | 0.5 | 0.3 (very smooth) to 1.0 (very reactive) |

### Aggressiveness tuning

Aggressiveness directly influences the controller gains (Kp). A higher value results in higher gains, giving a faster response but with higher risk of oscillation.

- **0.3**: Very smooth response, for high inertia rooms or those prone to overshoot
- **0.5**: Balance between performance and stability. Recommended starting point
- **1.0**: Very reactive response, for fast electric heaters with low inertia

The algorithm automatically reduces Kp and Ki gains within a band close to the setpoint (±0.3°C by default) via **near-band scheduling**.

## Detailed operation of the algorithm

AutoPI operation can be broken into 5 cyclic steps:

### 1. Measurement and Observation
At each cycle, the algorithm collects:
- Current temperature ($T_{int}$)
- Outdoor temperature ($T_{ext}$)
- Power sent to the heater ($u$)

### 2. Modeling (Conditional EWMA)
It updates its internal room model defined by two parameters:
- **a** (Efficiency): How many degrees gained per minute at 100% power
- **b** (Heat loss): How many degrees lost per minute per degree of difference with outside

The algorithm uses **conditional** EWMA (exponential moving average) estimation:
- **b** is learned primarily during OFF phases (u < 5%), when cooling is observable
- **a** is learned primarily during ON phases (u > 20%), when heating is significant

This approach avoids interference between the two parameters and ignores noisy measurements.

### 3. Model Reliability Check
Before using the learned model, the algorithm verifies its **reliability**:
- At least 6 successful learning cycles
- τ (time constant) in a plausible range (10 to 2000 minutes)
- b is stable (coefficient of variation < 35%)

If these criteria are not met, conservative **"safe" gains** are used (Kp=0.55, Ki calculated).

### 4. Gain Calculation (Heuristic)
Once the model is reliable, gains are calculated via a simple heuristic:
- **τ**: Thermal time constant = 1/b
- **Kp** = 0.35 + 0.9 × (τ / 200), bounded between 0.10 and 2.50
- **Ki** = Kp / max(τ, 10), bounded between 0.001 and 0.050

### 5. Command Application
Finally, it calculates the power to send to the heater:
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)**: Power needed to compensate thermal losses. This term progressively ramps up at startup (warmup) and is capped at 30% until the model is reliable.
- **$u_{pi}$ (Correction)**: The surplus to correct the current deviation from setpoint. Uses a weighted error (2-DOF) for proportional action.

### Anti-windup: Conditional Integration
The algorithm uses an anti-windup technique called **conditional integration**:
- If output is **saturated at 1.0** AND error is **positive** → integral is NOT updated (I:SKIP)
- If output is **saturated at 0.0** AND error is **negative** → integral is NOT updated (I:SKIP)
- Otherwise → normal integration (I:RUN)

This prevents excessive accumulation of integral error ("windup") when the actuator cannot apply more power.

### Integrator Hold (dead time)
If the integrator is in **HOLD** mode (during "dead time" periods after a switching), the integral is frozen to prevent pumping.

### Sign-Flip Leak (soft discharge)
When the error changes sign (temperature crosses above or below setpoint), the integral is partially discharged (30% per cycle for 2 cycles). This allows a softer "landing" and avoids oscillations around the setpoint.

### Near-Band Scheduling (gain reduction)
Within a ±0.3°C band around the setpoint, Kp and Ki gains are reduced (×0.70 and ×0.50) for smoother behavior approaching the target.

### 2-DOF PI (setpoint weighting)
The proportional action uses a weighted error `e_p = 0.4 × setpoint - temperature` (instead of `setpoint - temperature`), which reduces overshoot on setpoint changes.

### Overshoot protection
The power change rate is limited (25% per minute) to prevent sudden changes.

## Diagnostic metrics

The algorithm exposes several metrics in the climate entity attributes:

| Metric | Description |
|--------|-------------|
| **a** | Heating efficiency (°C/min at 100% power) |
| **b** | Heat loss coefficient (1/min) |
| **tau_min** | Thermal time constant of the room (in minutes) |
| **tau_reliable** | Indicates if the thermal model is considered reliable |
| **learn_ok_count** | Number of successful learning cycles |
| **learn_last_reason** | Reason for the last learning result |
| **Kp**, **Ki** | PI controller gains (calculated or "safe") |
| **u_ff** | Feed-forward component of the command |
| **i_mode** | Integrator state (I:RUN, I:SKIP, I:HOLD, I:LEAK) |
| **sat** | Saturation state (NO_SAT, SAT_HI, SAT_LO) |
| **error** | Difference between setpoint and current temperature |
| **error_p** | 2-DOF weighted error used for proportional action |
| **error_filtered** | EMA-filtered error (smoothing for quantized sensors) |
| **integral_error** | Accumulated integral error |
| **cycles_since_reset** | Number of cycles since last reset |
| **sign_flip_leak_left** | Remaining cycles of integral discharge |

The `tau_reliable` metric becomes `true` when the model has accumulated enough reliable data (at least 6 successful learning cycles, τ in the 10-2000 minute range, and stable b).

## Tips for optimal learning

For the algorithm to effectively learn the thermal behavior of your room, follow these recommendations:

### ✅ Do

- **Set a stable setpoint** for at least 2-3 days
- **Moderate setpoint**: 1-2°C above current temperature (generates partial cycles, more informative)
- **Keep the system in HEAT mode** (the algorithm only learns during heating)
- **Wait for day/night cycles** (outdoor temperature variations help learn thermal losses)

### ❌ Don't

- **Change the setpoint frequently**: creates transients that disturb learning
- **Set temperature too high**: causes 100% power cycles, less informative
- **Open windows**: disrupts the thermal model being learned
- **Manually turn off heating**: interrupts data collection

### Recommended procedure

1. Activate AutoPI and verify the thermostat is in HEAT mode
2. Set the target to ~1-2°C above current temperature
3. Let it run for 2-3 days without changing the setpoint
4. Check in attributes that `learn_ok_count` > 6
5. The `tau_reliable` metric should become `true`

> **Note**: The algorithm continues learning permanently. The initial phase establishes a baseline, then it continuously refines to adapt to changes (weather, insulation, etc.).

## Recommended use cases

AutoPI is particularly suited for:

- 🏠 Installations where you don't know the right TPI coefficients
- 🔄 Rooms whose thermal characteristics change (sun exposure, variable occupancy)
- ⚡ Users who want a "plug and play" solution

## Differences with TPI and Auto-TPI

| Aspect | TPI | Auto-TPI | AutoPI |
|--------|-----|----------|--------|
| **Coefficients** | Fixed (manual) | Learned then fixed | Continuously adaptive |
| **Configuration** | Complex (Kint, Kext) | Medium | Simple (2 parameters) |
| **Adaptation time** | Immediate | ~50 cycles | Continuous |
| **Learning** | None | Finite phase | Permanent |
| **Model type** | Simple proportional | Statistical observation | Thermal model (EWMA) |
| **Tuning method** | Manual | Heuristic | Adaptive heuristic |

> **Note**: AutoPI is a complementary approach to TPI and Auto-TPI. It is particularly suited for users who prefer an automatic solution without configuration.

## Services

### reset_auto_pi_learning

This service completely resets the AutoPI algorithm learning. All learned data (thermal model, controller gains) are reset to their default values.

This can be useful if:
- You have performed significant insulation work
- You have changed the heater
- The thermostat behavior no longer seems optimal after a disruptive external event

Learning will restart from scratch at the next heating cycle.

```yaml
service: versatile_thermostat.reset_auto_pi_learning
target:
  entity_id: climate.my_thermostat
```
