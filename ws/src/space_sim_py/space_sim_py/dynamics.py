import numpy as np


def propagate_state(state: np.ndarray, dt: float) -> np.ndarray:
    x, y, z, vx, vy, vz = state
    return np.array([
        x + vx * dt,
        y + vy * dt,
        z + vz * dt,
        vx,
        vy,
        vz
    ], dtype=float)
