from controller import Controller
from kf import KalmanFilter
from ekf import ExtendedKalmanFilter
from sensor import Sensor
from spacecraft import Spacecraft

import numpy as np
import matplotlib.pyplot as plt

# ----- DEFINED CONSTANTS -----
Re = 6.371e6 # Earth radius [m]
mu = 3.986e14 # Earth's gravitational parameter [m^3/s^2]

# ----- INITIAL ORBIT CONDITIONS -----
h = 400e3          # Altitude [m]
r = Re + h         # Distance from Earth's center [m]
v = np.sqrt(mu/r) # Orbit velocity [m/s]

# ----- INITIAL STATE -----
state = np.array([r, 0, 0, v])

# ----- INITIALIZE SPACECRAFT -----
spacecraft = Spacecraft(state, mu)

# ----- INITIALIZE SENSORS -----
position_sensor = Sensor(1e3, 2)
velocity_sensor = Sensor(50, 2)

# ----- KALMAN FILTER PARAMS -----
P = np.diag([1e6, 1e6, 2500, 2500])
R = np.diag([1e6, 1e6, 2500, 2500])
Q = np.diag([0, 0, 0, 0])

# ----- INITIALIZE KALMAN FILTER -----
filter = ExtendedKalmanFilter(P, R, Q)

# ----- INITIAL STATE ESTIMATE  -----
state_estimate = np.array([6.5e6, 0, 0, 6.8e3])

# ----- CONTROLLER PARAMS -----
rd = 8e6 # Desired radius [m]
vr = np.sqrt(mu / rd) # Required velocity [m/s]
kp = 2e-5 # Proportional constant [1/s^2]
kd = 0.06897 # Derivative constant [1/s]
kv = 6.0569e-4 # Velocity constant [1/s]

# ----- INITIALIZE CONTROLLER -----
controller = Controller(rd, vr, kp, kd, kv)

# ----- INITIALIZE T TO ZERO -----
t = 0

# ----- IMPORTANT VALUES FOR ANALYSIS -----
all_states = [spacecraft.get_state()]
all_estimates = [state_estimate.copy()]
all_times = [t]

# ----- BEGIN SIMULATOR -----
while t < 60000:
    # ----- FIND THRUST COMPONENT -----
    control_acceleration = controller.get_thrust(state_estimate)

    # ----- PROPOGATE THE SPACECRAFT -----
    spacecraft.propagate(control_acceleration, 1)

    # ----- INCREMENT T BY 1 -----
    t = t + 1
    
    # ----- SENSOR VARIABLES -----
    xm, ym = position_sensor.measure(spacecraft.get_state()[:2])
    vxm, vym = velocity_sensor.measure(spacecraft.get_state()[2:])
    measurement = np.array([xm, ym, vxm, vym])

    # ----- UPDATE STATE ESTIMATE -----
    state_estimate = filter.update(control_acceleration,
                                    measurement,
                                    state_estimate, 
                                    mu, 
                                    1)

    all_states.append(spacecraft.get_state())
    all_estimates.append(state_estimate.copy())
    all_times.append(t)

# ----- ANALYSIS SECTION -----
states = np.array(all_states).T
estimates = np.array(all_estimates).T

r_vals = np.sqrt(states[0]**2 + states[1]**2)
r_estimates = np.sqrt(estimates[0]**2 + estimates[1]**2)

# Calculate eccentricity
ra = np.max(r_vals)
rp = np.min(r_vals)
e = (ra - rp) / (ra + rp)

radius_error = r_vals - r_estimates
max_radius_error = np.max(np.abs(radius_error))
rms_radius_error = np.sqrt(np.mean(radius_error**2))

print("Maximum Radius Estimation Error:", max_radius_error, "m")
print("RMS Radius Estimation Error:", rms_radius_error, "m")

# Calculate overshoot
overshoot = ra - rd
overshoot_percent = (overshoot / rd) * 100

# Calculate settling time
settling_band = 0.02 * rd
outside_band = np.abs(r_vals - rd) > settling_band

if not np.any(outside_band):
    settling_time = all_times[0]
elif outside_band[-1]:
    settling_time = None
else:
    last_outside = np.where(outside_band)[0][-1]
    settling_time = all_times[last_outside + 1]

print("Settling Time:", settling_time)
print("Overshoot Percentage:", overshoot_percent)

# Create plot
all_times = np.array(all_times)
plt.plot(all_times, r_vals)

plt.xlabel("Time [s]")
plt.ylabel("Radius [m]")
plt.title("Orbital Radius vs Time")
plt.axhline(8e6, linestyle="--", label="Desired Radius")
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(all_times, radius_error)

plt.xlabel("Time [s]")
plt.ylabel("Radius Estimation Error [m]")
plt.title("EKF Radius Estimation Error")
plt.axhline(0, linestyle="--")
plt.grid()
plt.show()