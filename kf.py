from spacecraft import Spacecraft
from controller import Controller

import numpy as np

class KalmanFilter:
    def __init__(self, P_vals, R_vals, Q):
        self.P_vals = np.array(P_vals)
        self.R_vals = np.array(R_vals)
        self.Q = Q


    def filter(self, measurement, state_estimate, controller, mu):
        xm, ym, vxm, vym = measurement
        x, y, vx, vy = self.predict(controller, state_estimate, mu)

        P_minus = self.P_vals + self.Q
        K = P_minus/(P_minus + self.R_vals)

        x = x + K[0]*(xm-x)
        y = y + K[1]*(ym-y)
        vx = vx + K[2]*(vxm-vx)
        vy = vy + K[3]*(vym-vy)

        self.P_vals = (1-K)*P_minus

        return np.array([x, y, vx, vy])

    def predict(self, controller : Controller, state, mu):
        spacecraft = Spacecraft(state, mu)

        control_acceleration = controller.get_thrust(state)
        spacecraft.propagate(control_acceleration, 1)

        return spacecraft.get_state()

