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
3. **Gain adaptation**: The controller parameters are automatically adjusted using the SIMC (Skogestad IMC) method

```
┌─────────────────────────────────────────────────────┐
│                    AutoPI                           │
│                                                     │
│   Temperature ──► Learning ──► Room model           │
│   Power       ──► (Robust Est.) ──► (a, b)          │
│                         │                           │
│                         ▼                           │
│               Gain calculation (Kp, Ki)             │
│               via SIMC method                       │
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
 ✅ **Avoids oscillations**: Built-in gain scheduling for smooth regulation near setpoint  
 ✅ **Robust to disturbances**: The algorithm ignores sudden variations to preserve model accuracy  
 ✅ **Inertia management**: Accounts for heating response time via SIMC method  
 ✅ **Anti-overshoot**: Multi-level protection (adaptive feedforward, proportional unwinding)

## Configuration

To enable AutoPI:

1. Create or modify a VTherm of type **switch** or **valve**
2. In **Underlyings**, select the **AutoPI** algorithm
3. Configure the parameters in the **AutoPI** menu

## Available parameters

| Parameter | Description | Default value | Recommendation |
|-----------|-------------|---------------|----------------|
| **Deadband (°C)** | Zone around the setpoint where no action is taken. Avoids micro-regulations | 0.05°C | 0.05 - 0.1°C |
| **Aggressiveness** | Response factor of the controller. Higher = smoother (lower gains) | 0.5 | 0.3 (reactive) to 1.0 (very smooth) |

### Aggressiveness tuning

Aggressiveness controls the closed-loop time constant (τc) used in SIMC gain calculation. A higher value gives lower gains and thus a slower but more stable response.

- **0.3**: Reactive response, for well-insulated rooms with low inertia
- **0.5**: Balance between performance and stability. Recommended starting point
- **1.0**: Very smooth response, to avoid oscillations in rooms with high inertia

## Detailed operation of the algorithm

AutoPI operation can be broken down into 5 cyclic steps:

### 1. Measurement and Observation
At each cycle, the algorithm collects:
- Current temperature ($T_{int}$)
- Outdoor temperature ($T_{ext}$)
- Power sent to the heater ($u$)

### 2. Modeling (RLS - Recursive Least Squares)
It updates its internal room model defined by two parameters:
- **a** (Efficiency): How many degrees gained per minute at 100% power
- **b** (Heat loss): How many degrees lost per minute per degree of difference with outside

The RLS algorithm learns these parameters continuously with a forgetting factor to adapt to changing conditions.

### 3. Gain Calculation (SIMC Tuning)
Once it knows the room ($a$, $b$), it calculates the ideal coefficients for the PI controller using the **SIMC (Skogestad IMC)** method:
- **τ**: Thermal time constant = 1/b
- **θ**: Estimated dead time (heating response delay)
- **τc**: Closed-loop time constant (controlled by aggressiveness)
- **Kp** = τ / (a × (τc + θ))
- **Ki** = Kp / min(τ, 4×(τc + θ))

### 4. Gain Scheduling
When temperature approaches the setpoint (error < 1.5°C), effective gains are progressively reduced to prevent oscillations:
- At 1.5°C from setpoint: 100% of gains
- At 0°C from setpoint: 50% of gains

### 5. Command Application
Finally, it calculates the power to send to the heater:
$$ u = u_{ff} + u_{pi} $$
- **$u_{ff}$ (Feed-forward)**: Power needed to compensate thermal losses. This term is limited during learning phase and reduced to zero during overshoot.
- **$u_{pi}$ (Correction)**: The surplus to correct the current deviation from setpoint.

### Overshoot protection
The algorithm integrates protections against temperature overshoot:
- Integral is reduced proportionally to overshoot magnitude
- Power change rate is limited, even more strictly near setpoint

## Diagnostic metrics

The algorithm exposes several metrics in the climate entity attributes:

| Metric | Description |
|--------|-------------|
| **a** | Heating efficiency (°C/min at 100% power) |
| **b** | Heat loss coefficient (1/min) |
| **tau_min** | Thermal time constant of the room (in minutes) |
| **learning_cycles** | Number of learning cycles observed |
| **model_confidence** | Overall model confidence (0-100%, based on cycles) |
| **Kp**, **Ki** | Base PI controller gains calculated by SIMC |
| **effective_Kp**, **effective_Ki** | Effective gains after gain scheduling |
| **u_ff** | Feed-forward component of the command |
| **error** | Difference between setpoint and current temperature |
| **integral_error** | Accumulated integral error |

The `model_confidence` metric is based on the number of learning cycles: it reaches 100% after 50 observed heating cycles, which truly reflects the quality of learned data.

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
4. Check in attributes that `learning_cycles` > 50
5. The `model_confidence` should reach 100%

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
| **Model type** | Simple proportional | Statistical observation | Thermal model (RLS) |
| **Tuning method** | Manual | Heuristic | SIMC (industrial) |

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
