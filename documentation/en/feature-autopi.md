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
 ✅ **Robust to disturbances**: The algorithm adapts to changing conditions via continuous learning.
 ✅ **Overshoot protection**: Feed-forward and integral are automatically reduced when temperature exceeds setpoint.

## Configuration

To enable AutoPI:

1. Create or modify a VTherm of type **switch** or **valve**
2. In **Underlyings**, select the **AutoPI** algorithm
3. Configure the parameters in the **AutoPI** menu

## Available parameters

| Parameter | Description | Default value | Recommendation |
|-----------|-------------|---------------|----------------|
| **Deadband (°C)** | Zone around the setpoint where no action is taken. Avoids micro-regulations | 0.05°C | 0.05 - 0.1°C |
| **Aggressiveness** | Response factor of the controller. Lower = more aggressive (higher gains) | 0.5 | 0.25 (aggressive) to 1.0 (smooth) |

### Aggressiveness tuning

Aggressiveness controls the PI controller gains (Kp and Ki). A lower value gives higher gains and thus a faster response. The default value (0.5) gives the base gains calculated by the algorithm.

- **0.25**: Doubled gains. Very aggressive response, for rooms that respond slowly
- **0.5**: Base gains. Recommended starting point for most installations
- **1.0**: Gains halved. Smoother response, to avoid oscillations

## Detailed operation of the algorithm

AutoPI operation can be broken down into 4 cyclic steps:

### 1. Measurement and Observation
At each cycle, the algorithm collects:
- Current temperature ($T_{int}$)
- Outdoor temperature ($T_{ext}$)
- Power sent to the heater ($u$)

### 2. Modeling (RLS - Recursive Least Squares)
It updates its internal room model defined by two parameters:
- **a** (Efficiency): How many degrees I gain per minute if I heat at 100%.
- **b** (Heat loss): How many degrees I lose per minute per degree of difference with outside.

The RLS algorithm learns these parameters continuously with a forgetting factor to adapt to changing conditions.

### 3. Gain Calculation (PI Tuning)
Once it knows the room ($a$, $b$), it calculates the ideal coefficients for the PI controller:
- **Kp** (Proportional): Calculated based on thermal time constant.
- **Ki** (Integral): Calculated to eliminate residual error.

### 4. Command Application
Finally, it calculates the power to send to the heater:
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)**: The power just needed to maintain temperature (based on $T_{ext}$ and heat losses $b$). This term is automatically reduced when overshooting.
- **$u_{pi}$ (Correction)**: The surplus to correct the current deviation from setpoint.

### Overshoot protection
The algorithm integrates protection against temperature overshoot:
- Feed-forward is progressively reduced to zero when temperature exceeds setpoint
- Integral is automatically reduced during overshoot

## Diagnostic metrics

The algorithm exposes several metrics in the climate entity attributes:

| Metric | Description |
|--------|-------------|
| **a** | Heating efficiency (°C/min at 100% power) |
| **b** | Heat loss coefficient (1/min) |
| **tau_min** | Thermal time constant of the room (in minutes) |
| **confidence_a** | Confidence in parameter a (0-100%) |
| **confidence_b** | Confidence in parameter b (0-100%) |
| **model_confidence** | Overall model confidence (average of a and b) |
| **Kp**, **Ki** | Automatically calculated PI controller gains |
| **u_ff** | Feed-forward component of the command |
| **error** | Difference between setpoint and current temperature |
| **integral_error** | Accumulated integral error |

The `model_confidence` metric is particularly useful: a value close to 0% means the algorithm has just started, while a high value (>80%) indicates the thermal model is well established.


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
