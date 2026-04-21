import numpy as np


class RelativeEKF:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros(6)  # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 0.1

        self.Q = np.eye(6) * 0.001
        self.R = np.eye(3) * 0.05**2

    def predict(self):
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def state_vector(self):
        return self.x
