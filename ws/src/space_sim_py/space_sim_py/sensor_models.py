import numpy as np


def noisy_position_measurement(state: np.ndarray, sigma: float = 0.05) -> np.ndarray:
    pos = state[:3]
    noise = np.random.normal(0.0, sigma, size=3)
    return pos + noise
