# Orbital GNC Simulation

A 2D spacecraft Guidance, Navigation, and Control (GNC) simulation built in Python. The project models orbital dynamics, closed-loop orbit control, noisy position and velocity measurements, and Kalman-filter-based state estimation.

## Overview

This project simulates a spacecraft initially in a 400 km circular Earth orbit and uses a closed-loop controller to transfer it toward a desired circular orbit at an 8,000 km radius.

The simulation is structured around three major GNC components:

* **Guidance** — defines the desired orbital radius and velocity.
* **Navigation** — estimates spacecraft position and velocity using an Extended Kalman Filter (EKF).
* **Control** — calculates the required tangential acceleration to reduce orbital errors.

The project is designed to explore how estimation uncertainty and sensor noise affect a closed-loop spacecraft system.

## Current Features

### Orbital Dynamics

The spacecraft is modeled as a point mass in a 2D Earth-centered orbit.

The gravitational acceleration is modeled as:

$$
a_x = -\frac{\mu x}{r^3}
$$

$$
a_y = -\frac{\mu y}{r^3}
$$

where:

* $$r = \sqrt{x^2+y^2}\$$
* $$\mu$$ is Earth's gravitational parameter

The equations of motion are numerically integrated using `scipy.integrate.solve_ivp`.

### Orbit Controller

The controller uses radial position, radial velocity, and orbital speed errors:

$$
e_r = r_d-r
$$

$$
\dot e_r = -\frac{xv_x+yv_y}{r}
$$

$$
e_v = v_d-v
$$

The tangential control acceleration is calculated using:

$$
a_t = K_p e_r + K_d\dot e_r + K_v e_v
$$

The resulting acceleration is applied in the direction of the spacecraft's velocity vector.

### Sensor Model

The simulation includes independent noisy position and velocity sensors.

Current measurement noise:

* Position standard deviation: **1,000 m**
* Velocity standard deviation: **50 m/s**

The sensors generate measurements by adding Gaussian noise to the true spacecraft state.

### Extended Kalman Filter

Because orbital dynamics are nonlinear, the project uses an Extended Kalman Filter rather than a standard linear Kalman Filter.

The EKF follows the cycle:

**Predict → Measure → Correct**

The nonlinear orbital dynamics are locally linearized using a Jacobian matrix. The covariance is propagated using:

$$
P^- = FPF^T+Q
$$

where:

* \(P\) is the state covariance
* \(F\) is the state-transition matrix
* \(Q\) is the process-noise covariance

The current implementation directly measures the four state variables:

$$
[x,\ y,\ v_x,\ v_y]^T
$$

so the measurement matrix is the identity matrix.

## Results

Using the current controller and EKF configuration, the spacecraft successfully converges toward the desired 8,000 km orbital radius while the controller operates on the EKF's estimated state rather than the true spacecraft state.

One test began with a deliberately offset state estimate:

* Initial position estimate: **6.5 × 10⁶ m**
* Actual initial position: approximately **6.771 × 10⁶ m**
* Initial radial estimation error: approximately **271 km**

The EKF converged from this initial error while the closed-loop spacecraft simulation remained stable.

In one run:

* Settling time: approximately **7,198 s**
* RMS radial estimation error: approximately **1.41 km**
* Maximum radial estimation error: **271 km**, occurring at initialization

The maximum error is dominated by the intentionally incorrect initial state estimate; the estimate converges substantially closer to the true state afterward.

## Process Noise Covariance Experiment

To investigate how uncertainty in the spacecraft dynamics affects the Extended Kalman Filter, I tested several values of the process noise covariance matrix `Q`.

The diagonal elements of `Q` represent the variance of the assumed process noise for each state:

* Position: $$m^2$$
* Velocity: $$\frac{m}{s}^2$$

The baseline sensor noise was:

Position standard deviation: `1000 m`
Velocity standard deviation: `50 m/s`

### Experiment results:

|Process Noise                    | Settling Time | Overshoot |
|---------------------------------|---------------|-----------|
|Q = 0	                          | 7198 s        |	0.00009%  |
|Q = diag([5e5, 5e5, 1250, 1250]) |	7244 s	      | 0.0873%   |
|Q = diag([1e6, 1e6, 2500, 2500]) |	7117 s	      | 0.07%     |

The larger values of `Q` represent greater uncertainty in the spacecraft model. Since the EKF predicts covariance using

$$P^{-} = FPF^{T} + Q$$

increasing `Q` increases the predicted uncertainty `P⁻`. This generally increases the Kalman gain, causing the filter to place more weight on the sensor measurements.

Because the controller uses the EKF state estimate, changes in the filter's weighting also affect the closed-loop spacecraft response. Larger process noise therefore allowed more measurement noise to influence the controller, which produced small changes in settling time and overshoot.

### Conclusion

The experiment demonstrated that `Q` is not simply a parameter that should be minimized. It represents uncertainty in the system model and should reflect how accurately the dynamics are known.

For this simulation, the differences between the tested values were relatively small, and the measurements contain random noise, so individual settling-time differences should not be treated as definitive. The experiment primarily demonstrated how process uncertainty affects the balance between model prediction and sensor measurements.

## Project Structure

```text
orbital-gnc/
│
├── main.py
├── spacecraft.py
├── sensor.py
├── controller.py
├── kf.py
├── ekf.py
├── README.md
└── .gitignore
```

### File Descriptions

| File            | Purpose                                              |
| --------------- | ---------------------------------------------------- |
| `main.py`       | Runs the simulation and performs analysis            |
| `spacecraft.py` | Models spacecraft dynamics and numerical propagation |
| `sensor.py`     | Generates noisy spacecraft measurements              |
| `controller.py` | Implements the closed-loop orbit controller          |
| `kf.py`         | Basic Kalman Filter implementation                   |
| `ekf.py`        | Extended Kalman Filter implementation                |

## Technologies

* Python
* NumPy
* SciPy
* Matplotlib
* Git / GitHub

## Future Work

* Tune and analyze EKF process-noise covariance \(Q\)
* Compare EKF performance against the basic Kalman Filter
* Analyze estimation error in all four state variables
* Investigate covariance and Kalman gain behavior
* Improve numerical and covariance propagation
* Explore more realistic spacecraft sensor and process-noise models
* Extend the simulation toward more advanced GNC algorithms

## Motivation

This project was developed to gain practical experience with spacecraft Guidance, Navigation, and Control concepts while combining aerospace engineering and computer science.

The project emphasizes building the estimator and controller from first principles and experimentally investigating how modeling assumptions, sensor noise, and estimation uncertainty affect closed-loop spacecraft behavior.