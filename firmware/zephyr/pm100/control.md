# ADRC Control Analysis and Parameter Derivation

This document details the theoretical derivation and practical tuning of the Active Disturbance Rejection Control (ADRC) parameters implemented in the PM100 power limiter firmware. This design is optimized for a small brushless DC (BLDC) motor propulsion system on an RC aircraft, controlled via Electronic Speed Controller (ESC) PWM signals ($1000\,\mu\text{s}$ to $2000\,\mu\text{s}$) under a **4S (14.8V) to 6S (22.2V) LiPo** battery pack.

---

## 1. System Modeling and Input-Output Mapping

In a second-order ADRC formulation, the system acceleration of the output variable $y(t)$ (Active Power in Watts) is coupled to the control input $u(t)$ (ESC PWM pulse width in microseconds) through:

$$\ddot{y}(t) = b_0 u(t) + f(t, y, \dot{y}, d)$$

Where:
*   **Output ($y$):** Active power $P(t)$ in Watts (measured by the high-side INA226 power monitor). Nominal target limit set to **$600\,\text{W}$**.
*   **Control Input ($u$):** Output throttle command $PWM_{out}$ in microseconds. Absolute physical bounds are $[1000\,\mu\text{s}, 2000\,\mu\text{s}]$, representing zero to full throttle.
*   **Total Disturbance ($f$):** Aggregates all internal unmodeled dynamics (motor/ESC efficiency curves, rotor friction, heat losses) and external disturbances (aerodynamic propeller load torque variations, battery voltage sag under high discharge rates).
*   **Control Gain Factor ($b_0$):** Denotes the estimated instantaneous sensitivity of power acceleration $\ddot{P}(t)$ to changes in the PWM input signal.

---

## 2. Derivation of the Gain Factor ($b_0$) first guess

To estimate a realistic initial value for $b_0$, we analyze the step response characteristics of a small RC aircraft BLDC motor:

1.  **Mechanical Time Constant ($\tau_m$):** Small BLDC motors (e.g., 2212 or 2814 stator size) paired with lightweight carbon fiber or plastic propellers exhibit highly responsive mechanical time constants, typically:
    $$\tau_m \approx 0.1\,\text{seconds}\quad (100\,\text{ms})$$
2.  **Open-Loop Bandwidth ($w_m$):**
    $$w_m = \frac{1}{\tau_m} \approx 10\,\text{rad/s}$$
3.  **Step Acceleration ($\ddot{P}$):**
    When applying a full-throttle step command from rest (delta input of $\Delta u = 1000\,\mu\text{s}$), the motor accelerates to its maximum power rating ($P_{max} \approx 800\,\text{W}$ on 4S to 6S setups) in approximately $\tau_m$ seconds. The initial second-order acceleration rate is approximated as:
    $$\ddot{P}(0) \approx \frac{P_{max}}{\tau_m^2} = \frac{800\,\text{W}}{(0.1\,\text{s})^2} = 80,000\,\frac{\text{W}}{\text{s}^2}$$
4.  **Derivation of $b_0$:**
    The control gain $b_0$ acts as the scaling coefficient mapping input microseconds directly to output power acceleration:
    $$b_0 \approx \frac{\ddot{P}(0)}{\Delta u} = \frac{80,000\,\text{W/s}^2}{1000\,\mu\text{s}} = 80.0\,\frac{\text{W}}{\text{s}^2\cdot\mu\text{s}}$$

To ensure that the state observer (LESO) does not experience initial peaking or cause excessive control action saturation, we select a conservative and physically stable first guess:
$$b_0 = 50.0\,\frac{\text{W}}{\text{s}^2\cdot\mu\text{s}}$$

---

## 3. Bandwidth-Based Controller Tuning ($K_p$, $K_d$)

Using **Gao's Bandwidth Parameterization** for a second-order system, we select a closed-loop controller bandwidth $w_c$ that balances rapid command tracking with high noise rejection.

1.  **Selection of Controller Bandwidth ($w_c$):**
    We select $w_c \approx 20\,\text{rad/s}$ (which is 2 times faster than the open-loop motor bandwidth of $10\,\text{rad/s}$ and well below the Nyquist limit of the 1kHz control loop).
2.  **Proportional Gain ($K_p$):**
    $$K_p = w_c^2 = 20^2 = 400.0$$
3.  **Derivative Gain ($K_d$):**
    $$K_d = 2 w_c = 2 \times 20 = 40.0$$

*This critical parameterization ($K_d = 2\sqrt{K_p}$) guarantees that the tracking loop remains critically damped ($\zeta = 1.0$), eliminating throttle overshoot during rapid pilot command transitions.*

---

## 4. Extended State Observer Tuning ($w_o$)

The **Linear Extended State Observer (LESO)** is responsible for dynamically estimating the measured power $z_1(t) \approx P(t)$, its derivative $z_2(t) \approx \dot{P}(t)$, and the total system disturbance state $z_3(t) \approx f(t)$ at a 1kHz sampling rate.

*   **Bandwidth Rule of Thumb:** To ensure fast and stable estimation without phase lag, the observer bandwidth $w_o$ should be set 5 times faster than the controller bandwidth:
    $$w_o = 5 \times w_c = 100.0\,\text{rad/s}$$
*   **Observer Pole Placement:** The three poles of the LESO are placed at $-w_o$, generating continuous-time Euler integration gains mapping to:
    *   $l_1 = 3 w_o$
    *   $l_2 = 3 w_o^2$
    *   $l_3 = w_o^3$

---

## 5. Practical Tuning Guidelines for Propulsion Engineers

When deploying this ADRC loop on your physical 4S to 6S aircraft propulsion rig, use the following calibration guidelines:

| Symptom / Observation | Root Cause | Corrective Action |
|---|---|---|
| High-frequency motor "hissing" or ESC jittering on standby | $w_o$ is too high, making the observer oversensitive to INA226 current sensor noise. | Decrease $w_o$ in steps of 10 (e.g., from 100 down to 70) to filter the noise. |
| Power limiting reacts too slowly, allowing initial overshoot past 600W. | Controller bandwidth $w_c$ is too low or system gain $b_0$ is overestimated. | Decrease $b_0$ (e.g., from 50 down to 30) or increase $w_c$ to scale up $K_p$ and $K_d$. |
| Motor is jittery or exhibits jerky thrust behavior on rapid throttle changes. | $b_0$ is underestimated, causing the controller to output excessively large corrective steps. | Increase $b_0$ (e.g., from 50 up to 80 or 100) to dampen setpoint error sensitivity. |
| Voltage sag on 6S LIPO battery causes sudden thrust drop under load. | LESO is slow to estimate the voltage drop's effect on motor torque. | Increase $w_o$ (e.g., from 100 to 120) to accelerate disturbance estimation. |
