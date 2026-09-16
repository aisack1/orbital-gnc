from spacecraft import Spacecraft
from controller import Controller

import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, P, R, Q):
        self.P = np.array(P)
        self.R = np.array(R)
        self.Q = Q

    def __jacobian(self, state_estimate, mu):
        x, y, vx, vy = state_estimate

        r = np.sqrt(x**2 + y**2)

        A = [[0, 0, 1, 0], [0, 0, 0, 1], 
             [-mu*((r**(-3)) - 3*(x**2)*(r**(-5))), 3*mu*x*y*(r**(-5)), 0, 0],
             [3*mu*x*y*(r**(-5)), -mu*((r**(-3)) - 3*(y**2)*(r**(-5))), 0, 0]]

        return np.array(A)

    def __state_transition_matrix(self, state_estimate, mu, dt):
        I = np.eye(4)
        A = self.__jacobian(state_estimate, mu)
        F = I + A*dt

        return F

    def __predict_covariance(self, state_estimate, mu, dt):
        F = self.__state_transition_matrix(state_estimate, mu, dt)
        P_minus = F @ self.P @ F.T + self.Q

        return P_minus

    def update(self, controller, measurement, state_estimate, H, mu, dt):
        state_prediction = self.predict(controller, state_estimate, mu, dt)

        P_minus = self.__predict_covariance(state_estimate, mu, dt)

        y = measurement - (H @ state_prediction)
        S = H @ P_minus @ H.T + self.R

        K = P_minus @ H.T @ np.linalg.inv(S)

        state_estimate = state_estimate + K @ y
        self.P = (np.eye(4) - K @ H) @ P_minus

        return state_estimate

    def predict(self, controller : Controller, state, mu, dt):
        spacecraft = Spacecraft(state, mu)
    
        control_acceleration = controller.get_thrust(state)
        spacecraft.propagate(control_acceleration, dt)
    
        return spacecraft.get_state()
             
