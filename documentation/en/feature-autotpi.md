# Auto TPI (Automatic TPI Coefficient Learning)

**Auto TPI** (Time Proportional Integral) is an intelligent feature of Versatile Thermostat that automatically learns the thermal characteristics of your room to adjust the TPI coefficients (`tpi_coef_int` and `tpi_coef_ext`) optimally.

Instead of guessing coefficients like `Kp` (proportional) and `Ki` (integral), the thermostat observes your room's behavior and calculates the best values for you.

## How to Activate

Auto TPI is activated via a specific service call in Home Assistant. It is not enabled by default in the configuration.

1.  **Go to Developer Tools > Services**.
2.  **Select the Service**: `Versatile Thermostat: Set Auto TPI mode` (or `versatile_thermostat.set_auto_tpi_mode`).
3.  **Choose Targets**: Select the thermostat entities you want to enable Auto TPI for.
4.  **Set Mode**: Toggle the "Auto TPI mode" option to **True** (Enable).
5.  **Call Service**.

You can also use this service in your automations to enable or disable learning dynamically.

**Note**: After activation, the system needs to observe several heating cycles (typically 5 to 10 cycles) before it starts producing its own parameters. A "cycle" is a period where the heater turns ON and then eventually OFF (or modulates).

## Operating Principle

Auto TPI observes the behavior of your room (indoor temperature, outdoor temperature, heating power) over a given period to build a thermal model.

It uses a learning algorithm based on weighted least squares regression to identify the physical parameters of the room.

The engine uses advanced algorithms to continuously adapt to your environment:
1.  **Smart Learning**: Uses advanced filtering (Savitzky-Golay) to ignore sensor noise and understand the true temperature trend.
2.  **Adaptive Responsiveness**: Automatically adjusts how fast the thermostat reacts based on your room's insulation.
3.  **Humidity Awareness**: If your humidity sensor is available, the algorithm takes humidity into account.
4.  **Self-Cleaning**: It detects "bad" data and excludes it from learning.
5.  **Drift Detection**: If your heating system changes (e.g., seasonal shifts), it detects the change and can pause learning.

## Mathematical Model

The model is based on the following thermal differential equation:

$$ \frac{dT_{room}}{dt} = -\alpha(T_{room} - T_{ext}) + \beta \cdot Power - \gamma(T_{room} - T_{target}) $$

Where:
*   $\frac{dT_{room}}{dt}$ is the variation of indoor temperature (derivative).
*   $\alpha$ (alpha) is the thermal loss coefficient (insulation).
*   $\beta$ (beta) is the heating efficiency (power/volume).
*   $\gamma$ (gamma) is a correction term related to inertia and the control loop.
*   $Power$ is the applied heating power (0-100%).

### Calculation Algorithm

1.  **Data Collection**: The system periodically records temperatures and power.
2.  **Filtering**: Outliers are discarded via an IQR (Interquartile Range) method.
3.  **Regression**: A linear regression (weighted least squares) is performed to find the optimal values of $\alpha$, $\beta$, and $\gamma$ that minimize the error between the model and reality. Recent data has more weight (progressive forgetting of old data with a half-life of 7 days).
4.  **Validation**: The quality of the model is evaluated via the coefficient of determination $R^2$. If $R^2 < 0.3$, the model is considered insufficient.

### TPI Coefficient Calculation

Once $\alpha$ and $\beta$ are determined, the TPI coefficients are calculated as follows:

*   **External Coefficient ($K_{ext}$)**:
    $$ K_{ext} = \frac{\alpha}{\beta} $$
    It represents the power required (in %) to compensate for a 1°C difference with the outside.
    Value bounded between 0.01 and 0.20.

*   **Internal Coefficient ($K_{int}$)**:
    $$ K_{int} = \frac{1}{\beta \times \tau_{target}} $$
    Where $\tau_{target}$ is the desired response time (fixed at 30 minutes). It represents the reactivity needed to correct an indoor temperature deviation.
    Value bounded between 0.01 and 1.0.

## Learning Prerequisites

For learning to be validated, the following are required:
*   At least 100 data points.
*   At least 10 complete heating cycles detected.
*   Sufficient model quality ($R^2$) (at least "Fair" / > 0.3).

## Quality Indicators & Monitoring

The learning status is visible via the thermostat attributes:
*   `learning_active`: `true` means it's currently gathering data.
*   `learning_quality`: insufficient, poor, fair, good, excellent.
*   `confidence`: Percentage of confidence in the model ($R^2 \times 100$).
*   `time_constant`: Thermal time constant of the room (inertia).

### Interpreting Attributes

*   **Quality**:
    *   **Insufficient**: Not enough data yet. Keep using the thermostat normally.
    *   **Poor/Fair**: The model has some data but predictions are not perfect. It will improve over time.
    *   **Good/Excellent**: The system has a very good understanding of your room.

*   **Time Constant ($\tau$)**:
    *   **Low (< 2h)**: Your room heats up and cools down quickly (low inertia). The thermostat will be more aggressive.
    *   **High (> 10h)**: Your room has high thermal mass (e.g., underfloor heating, thick stone walls). The thermostat will be slower and smoother.

*   **Confidence (R²)**: A percentage showing how well the mathematical model fits the reality. > 70% is great.

## Troubleshooting

*   **Learning is stuck at "Insufficient"**: Ensure your temperature sensors are reliable and updated frequently. Large gaps in data can slow down learning.
*   **Heating is erratic**: If the "Concept Drift" warning appears in logs, it means the thermal behavior changed drastically. The system usually pauses learning to be safe. You might want to reset the learning if you made major changes to the room (e.g., new windows).

## Algorithm Philosophy (FAQ)

**Why not use the current coefficients to calculate the new ones?**

The algorithm uses a **Physical Model Identification** approach and not an iterative approach (like "trial and error" or "gradient descent").

1.  **Physics Independence**: The thermal characteristics of your room (insulation $\alpha$, power $\beta$) are physical constants. They do not depend on the thermostat settings. Whether your thermostat is poorly tuned (oscillations) or well tuned, the physical relationship `Power -> Temperature Variation` remains the same.
2.  **Convergence Speed**: By directly identifying the physics of the room, we can mathematically calculate the "ideal" coefficients in one go (as soon as we have enough data). An iterative approach that tries to adjust coefficients little by little would take weeks (because each heating cycle is slow) to converge.
3.  **Role of Old Coefficients**: The old coefficients still have an impact: they determine the `Power` applied during learning. If the old coefficients are "bad", they will cause temperature variations (oscillations). Paradoxically, these variations help the algorithm because they "excite" the system and allow better identification of its reactions. A perfectly stable system is sometimes harder to identify (less dynamic data).

The algorithm therefore does not seek to "correct" the old coefficients, but to "understand" the room to directly propose the right settings.

## Privacy & Data

All learning data is stored locally on your Home Assistant instance in `.storage/versatile_thermostat_*.json`. No data is sent to the cloud.
