# Auto TPI (Automatic TPI Coefficient Learning)

This feature allows the Versatile Thermostat to automatically learn the thermal characteristics of your room to adjust the TPI coefficients (`tpi_coef_int` and `tpi_coef_ext`) optimally.

## Operating Principle

Auto TPI observes the behavior of your room (indoor temperature, outdoor temperature, heating power) over a given period to build a thermal model.

It uses a learning algorithm based on weighted least squares regression to identify the physical parameters of the room.

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

## Quality Indicators

The learning status is visible via the thermostat attributes:
*   `learning_quality`: insufficient, poor, fair, good, excellent.
*   `confidence`: Percentage of confidence in the model ($R^2 \times 100$).
*   `time_constant`: Thermal time constant of the room (inertia).

## Algorithm Philosophy (FAQ)

**Why not use the current coefficients to calculate the new ones?**

The algorithm uses a **Physical Model Identification** approach and not an iterative approach (like "trial and error" or "gradient descent").

1.  **Physics Independence**: The thermal characteristics of your room (insulation $\alpha$, power $\beta$) are physical constants. They do not depend on the thermostat settings. Whether your thermostat is poorly tuned (oscillations) or well tuned, the physical relationship `Power -> Temperature Variation` remains the same.
2.  **Convergence Speed**: By directly identifying the physics of the room, we can mathematically calculate the "ideal" coefficients in one go (as soon as we have enough data). An iterative approach that tries to adjust coefficients little by little would take weeks (because each heating cycle is slow) to converge.
3.  **Role of Old Coefficients**: The old coefficients still have an impact: they determine the `Power` applied during learning. If the old coefficients are "bad", they will cause temperature variations (oscillations). Paradoxically, these variations help the algorithm because they "excite" the system and allow better identification of its reactions. A perfectly stable system is sometimes harder to identify (less dynamic data).

The algorithm therefore does not seek to "correct" the old coefficients, but to "understand" the room to directly propose the right settings.
