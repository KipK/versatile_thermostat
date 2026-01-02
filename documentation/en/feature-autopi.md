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
2. **Thermal modeling**: It builds a mathematical model that represents your room (inertia, heat losses, heater power)
3. **Gain adaptation**: The controller parameters are automatically adjusted to optimize the response

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Temperature ──► Learning ──► Room model           │
│   Power       ──► (Robust Est.) ──► (a, b)          │
│                         │                           │
│                         ▼                           │
│               Gain calculation (Kp, Ki)             │
│                         │                           │
│                         ▼                           │
│               Power command (%)                     │
└─────────────────────────────────────────────────────┘
```

## What does it do concretely?

- **At startup**: AutoPI uses reasonable default values
- **After a few hours**: It starts understanding your installation
- **After a few days**: The model is refined and regulation becomes optimal

### Advantages
 
 ✅ **No manual tuning**: No need to find the right Kint/Kext coefficients  
 ✅ **Adapts to changes**: If you change your heater or insulate, it re-adapts  
 ✅ **Avoids oscillations**: The built-in deadband prevents unnecessary regulations near the setpoint  
 ✅ **Robust to disturbances**: The algorithm ignores sudden variations (e.g., window opening) to avoid corrupting the model.
 ✅ **Inertia management**: Takes into account the reaction time ("Dead time") of your heating to avoid heating too early or too late.
 ✅ **Gain Scheduling**: Adjusts the smoothness of regulation based on distance from setpoint (smoother when approaching the goal).

## Configuration

To enable AutoPI:

1. Create or modify a VTherm of type **switch** or **valve**
2. In **Underlyings**, select the **AutoPI** algorithm
3. Configure the parameters in the **AutoPI** menu

## Available parameters

| Parameter | Description | Default value | Recommendation |
|-----------|-------------|---------------|----------------|
| **Deadband (°C)** | Zone around the setpoint where no action is taken. Avoids micro-regulations | 0.05°C | 0.05 - 0.1°C |
| **Aggressiveness** | Response speed of the controller. Lower = faster | 0.5 | 0.3 (fast) to 1.0 (slow) |

### Aggressiveness tuning

- **0.1 - 0.3**: Very fast response. For rooms with low inertia (small rooms, direct electric heating)
- **0.5**: Balanced. Good starting point for most installations
- **1.0 - 2.0**: Slow response. For high inertia systems (underfloor heating, water radiators)

## Detailed operation of the "Robust" algorithm

AutoPI operation can be broken down into 5 cyclic steps:

### 1. Measurement and Observation
At each cycle, the algorithm collects:
- Current temperature ($T_{int}$)
- Outdoor temperature ($T_{ext}$)
- Power sent to the heater ($u$)

### 2. Modeling (Robust Estimator)
It updates its internal room model defined by two parameters:
- **a** (Efficiency): How many degrees I gain per minute if I heat at 100%.
- **b** (Heat loss): How many degrees I lose per minute per degree of difference with outside.

This is where the "Robust" approach comes in, filtering anomalies (e.g., sudden temperature drop) to avoid corrupting these $a$ and $b$ parameters.

### 3. Gain Calculation (PI Tuning)
Once it knows the room ($a$, $b$) and its inertia (Dead time), it calculates the ideal coefficients for the PI controller:
- **Kp** (Proportional): Calculated to react quickly but without overshooting.
- **Ki** (Integral): Calculated to eliminate residual error.

*"Gain Scheduling" reduces these gains when close to the setpoint for a smooth landing.*

### 4. Response Time Learning (Dead Time)

AutoPI automatically learns the **response time** of your heating system:
- It observes the delay between heater activation and the start of temperature rise
- This information is used to:
  - **Freeze the integrator**: During dead time, the integral doesn't accumulate (avoids oscillations)
  - **Adjust gains**: A longer dead time leads to more conservative gains

The algorithm collects multiple samples and uses the median for robust estimation.

### 5. Command Application
Finally, it calculates the power to send to the heater:
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)**: The power just needed to maintain temperature (based on $T_{ext}$ and heat losses $b$).
- **$u_{pi}$ (Correction)**: The surplus to correct the current deviation from setpoint.

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
| **Model type** | Simple proportional | Statistical observation | Thermal model (Robust Estimator) |

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
