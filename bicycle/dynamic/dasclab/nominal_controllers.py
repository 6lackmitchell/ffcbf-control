import numpy as np
from control import lqr
from nptyping import NDArray
from core.controllers.controller import Controller
from bicycle.dynamic.physical_params import LW, u_max
from bicycle.dynamic.models import f
from bicycle.dynamic.dasclab.initial_conditions import *


class LqrController(Controller):

    def __init__(self,
                 ego_id: int):
        super().__init__()
        self.ego_id = ego_id
        self.complete = False

    def _compute_control(self,
                         t: float,
                         z: NDArray) -> (int, str):
        """Computes the nominal input for a vehicle in the intersection situation.

        INPUTS
        ------
        t: time (in sec)
        z: full state vector

        OUTPUTS
        -------
        code: success (1) / error (0, -1, ...) code
        status: more informative success / error flag

        """
        # Modified LQR Controller from Black et al. 2022 (https://arxiv.org/pdf/2204.00127v1.pdf)
        ze = z[self.ego_id]

        # Get desired position
        xd = xg[self.ego_id]
        yd = yg[self.ego_id]

        # Get desired velocity
        speed_d = 0.5
        vd = speed_d * np.min([1, 1 / 4 * np.linalg.norm([ze[0] - xd, ze[1] - yd])])
        th = np.arctan2(yd - ze[1], xd - ze[0])
        vxd = vd * np.cos(th)
        vyd = vd * np.sin(th)

        q_star = np.array([xd, yd, vxd, vyd])  # desired state
        zeta = np.array([ze[0], ze[1], f(ze)[0], f(ze)[1]])  # double integrator state
        tracking_error = zeta - q_star
        if np.linalg.norm(tracking_error) < 0.25:
            tracking_error = np.array([0, 0, f(ze)[0], f(ze)[1]])
            self.complete = True

        A_di = np.array([[0, 0, 1, 0],
                         [0, 0, 0, 1],
                         [0, 0, 0, 0],
                         [0, 0, 0, 0]])
        B_di = np.array([[0, 0],
                         [0, 0],
                         [1, 0],
                         [0, 1]])

        gain = 4 * np.min([1.0 / (0.01 + (tracking_error[0])**2 + (tracking_error[1])**2), 10.0])
        # print(gain)
        # gain = 1
        # gain = 0.1
        Q = gain * np.diag([10, 10, 2, 2])
        R = np.diag([1, 1])

        # Compute LQR control input for double integrator model
        K, _, _ = lqr(A_di, B_di, Q, R)
        mu = -K @ tracking_error

        # Create transformation matrix
        S = np.array([[-ze[3] * np.sin(ze[2]) / np.cos(ze[4]) ** 2, np.cos(ze[2]) - np.sin(ze[2]) * np.tan(ze[4])],
                      [ze[3] * np.cos(ze[2]) / np.cos(ze[4]) ** 2, np.sin(ze[2]) + np.cos(ze[2]) * np.tan(ze[4])]])

        if ze[3] > abs(0.01):
            vec = np.array([mu[0] + f(ze)[1] * f(ze)[2], mu[1] - f(ze)[0] * f(ze)[2]])
            u = np.linalg.inv(S) @ vec
            omega = u[0]
            ar = u[1]
        else:
            omega = 0.0
            theta = np.arctan2(mu[1], mu[0]) - (ze[2] + ze[4])
            sign_ar = 1 if (-np.pi / 2 < theta < np.pi / 2) else -1
            ar = np.linalg.norm(mu) * sign_ar

        ar = np.clip(ar, -u_max[1], u_max[1])
        if not self.complete:
            omega = np.clip(omega, -u_max[0], u_max[0])
        else:
            omega = 0.0


        self.u = np.array([omega, ar])

        if self.ego_id == 0:
            print("Rover{} Nominal Control: {}".format(self.ego_id, self.u))
            print("Tracking Error: {}".format(tracking_error))
            print("Goal: {}".format((xd, yd)))

        return self.u, 1, "Optimal"


class ZeroController(Controller):

    def __init__(self,
                 ego_id: int):
        super().__init__()
        self.ego_id = ego_id
        self.complete = False

    def _compute_control(self,
                         t: float,
                         z: NDArray) -> (int, str):
        """Computes the nominal input for a vehicle in the intersection situation.

        INPUTS
        ------
        t: time (in sec)
        z: full state vector

        OUTPUTS
        -------
        code: success (1) / error (0, -1, ...) code
        status: more informative success / error flag

        """
        # Modified LQR Controller from Black et al. 2022 (https://arxiv.org/pdf/2204.00127v1.pdf)
        ze = z[self.ego_id]

        # if t == 0.0 and self.ego_id > 2:
        #     vi[self.ego_id] = cruise_speed + np.random.uniform(low=-1.0, high=1.0)
        omega = 0.0

        if self.ego_id == 7:
            if ze[0] < -5:
                a = 0.0
            elif t < 25:
                a = -1 / 3 * ze[3]
            else:
                a = 1/3 * (1 - ze[3])
        elif self.ego_id == 8:
            if ze[0] < -7:
                a = 0.0
            elif t < 25:
                a = -1 / 3 * ze[3]
            else:
                a = 1/3 * (1 - ze[3])
        else:
            a = 0.0

        self.u = np.array([omega, a])

        return self.u, 1, "Optimal"

# class ZeroController(Controller):
#
#     def __init__(self,
#                  ego_id: int):
#         super().__init__()
#         self.ego_id = ego_id
#
#     def _compute_control(self,
#                          t: float,
#                          z: NDArray) -> (int, str):
#         """Computes the nominal input for a vehicle in the intersection situation.
#
#         INPUTS
#         ------
#         t: time (in sec)
#         z: full state vector
#
#         OUTPUTS
#         -------
#         code: success (1) / error (0, -1, ...) code
#         status: more informative success / error flag
#
#         """
#         self.u = np.array([0.0, np.random.uniform(low=-0.1, high=0.1)])
#
#         return self.u, 1, "Optimal"
