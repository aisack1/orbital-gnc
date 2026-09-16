import numpy as np
from scipy.integrate import solve_ivp

class Spacecraft:
    def __init__(self, initial_state, mu : float):
        self.state = initial_state
        self.mu = mu

    def dynamics(self, t, state, control_acceleration):
        x, y, vx, vy = state
        ax_control, ay_control = control_acceleration

        r = np.sqrt(x**2 + y**2)

        ax = (-self.mu / r**3) * x + ax_control
        ay = (-self.mu / r**3) * y + ay_control
        
        return np.array([vx, vy, ax, ay])

    def propagate(self, control_acceleration, dt):
        sol = solve_ivp(
                self.dynamics,
                [0, dt],
                self.state,
                args=(control_acceleration,),
                t_eval=[dt],
                rtol=1e-9,
                atol=1e-9
            )

        self.state = sol.y[:, -1]

    def get_state(self):
        return self.state.copy()