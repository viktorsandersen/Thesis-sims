import numpy as np


def K_(quat, Ko):
    return 2 * E(quat[0], quat[1:]).T @ Ko

def E(eta, eps):
    return eta*np.eye(eps.shape[0]) - skewSymmetric(eps)

def skewSymmetric(eta):
    return np.array([
        [0, -eta[2], eta[1]],
        [eta[2], 0, -eta[0]],
        [-eta[1], eta[0], 0]
    ])
