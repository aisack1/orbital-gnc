import numpy as np

class Controller:
    def __init__(self, rd, vr, kp, kd, kv):
        self.rd = rd
        self.vr = vr
        self.kp = kp
        self.kd = kd
        self.kv = kv

    def get_thrust(self, state):
        x, y, vx, vy = state

        # Calculate r
        r = np.sqrt(x**2 + y**2)

        # Calculate velocity magnitude
        v = np.sqrt(vx**2 + vy**2)

        # Caculate position error
        er = self.rd - r
        erdot = -(x*vx + y*vy)/r

        # Calculate velocity error
        ev = self.vr - v

        # Calculate thrust
        P = self.kp*er
        D = self.kd*erdot
        V = self.kv*ev
        u = P + D + V
        
        return np.array([u*(vx/v), u*(vy/v)])