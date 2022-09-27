import numpy as np
from .physical_params import u_max

# q0 = 100.0 / u_max[0]**2
# q1 = 0.1 / u_max[1]**2
q0 = 1.0 / u_max[0]**2
q1 = 10.0 / u_max[1]**2
q1 = 10000.0 / u_max[1]**2

# q0 = 10.0 / u_max[0]**2
# q1 = 1.0 / u_max[1]**2
q2 = 2 * np.max([q0, q1])


def objective_accel_and_steering(u_nom):
    if len(u_nom) % 2 == 0:
        Q = np.diag(int(len(u_nom) / 2) * [q0, q1])
    else:
        Q = np.diag(int(len(u_nom) / 2) * [q0, q1] + [q2])

    Q = 1 / 2 * Q

    p = -Q @ u_nom
    return Q, p


def objective_accel_only(u_nom, agent):
    Q = q0*np.eye(len(u_nom))
    Q[agent, agent] = Q[agent, agent] / 50.0
    p = -q0 * u_nom
    return 1 / 2 * Q, p
