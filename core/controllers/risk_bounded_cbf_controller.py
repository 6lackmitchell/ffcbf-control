import numpy as np
import builtins
from typing import Callable, List
from importlib import import_module
from sys import exit
from nptyping import NDArray
from scipy.special import erf, erfinv
from scipy.linalg import block_diag
from .cbfs.cbf import Cbf
from .cbf_qp_controller import CbfQpController
from ..mathematics.analytical_functions import ramp, dramp
# from .cbfs import tau as Tau, tau_star as TauStar, Tmax

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
system_model = builtins.PROBLEM_CONFIG['system_model']
mod = 'simdycosys.' + vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'sigma': getattr(module, 'sigma_{}'.format(system_model))})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e


class RbCbfController(CbfQpController):

    adaptive_risk_bound = False

    def __init__(self,
                 u_max: List,
                 nAgents: int,
                 objective_function: Callable,
                 nominal_controller: Callable,
                 cbfs_individual: List,
                 cbfs_pairwise: List,
                 ignore: List = None):
        super().__init__(u_max,
                         nAgents,
                         objective_function,
                         nominal_controller,
                         cbfs_individual,
                         cbfs_pairwise,
                         ignore)
        self._stochastic = True
        nCBF = len(self.cbf_vals)
        self.integrator_state = np.zeros((nCBF,))
        self.LfB = np.zeros((nCBF,))
        self.LgB = np.zeros((nCBF, nAgents * self.nu))

    def _generate_cbf_condition(self,
                                cbf: Cbf,
                                h: float,
                                Lfh: float,
                                Lgh: float,
                                idx: int,
                                adaptive: bool = False) -> (NDArray, float):
        """Generates the matrix A and vector b for the Risk-Bounded CBF constraint of the form Au <= b."""
        k = 0.25
        sig = sigma([])
        B = np.exp(-k * h)
        B = np.min([B, 1.0])
        self.LfB[idx] = -k * B * Lfh
        self.LgB[idx, :] = -k * B * Lgh
        if sig.shape[0] == cbf.dhdx_value.shape[0]:
            self.LsB = -k * B * cbf.dhdx_value @ sig
        else:
            self.LsB = -k * B * cbf.dhdx_value @ block_diag(sig, sig)

        # Define required parameters -- hard coded for now
        rho_max = 0.90
        gamma = B

        if self.adaptive_risk_bound:
            # T = np.max([0, float(Tau([TauStar(ze, zo)]))])
            # Tdot = 0.0  # TauDot(ze, zo)  # need to do
            # Placeholders
            T = 2.0
            Tdot = 0.0

        else:
            T = 2.0
            Tdot = 0.0

        # Approach for eta: assume that the distance between the vehicles will never be greater than it is now
        eta = np.linalg.norm(np.abs(self.LsB))

        # Define derived parameters
        rho_min = 0.01
        if eta > 0 and gamma < 1:
            rho_min_updated = 1 - erf(np.nan_to_num((1 - gamma) / (np.sqrt(2) * T * eta), nan=np.inf))
            if rho_min_updated > rho_min:
                rho_min = rho_min_updated

        rho, rho_dot = self.get_risk(B, rho_min, rho_max)
        hp = 1 - gamma - np.sqrt(2) * eta * T * erfinv(1 - rho) - self.integrator_state[idx]

        erf_dot_term = np.sqrt(np.pi) / 2 * np.exp(-(erfinv(1 - rho)) ** 2)
        tv_term = np.sqrt(2) * eta * Tdot * erfinv(1 - rho) - np.sqrt(2) * eta * T * erf_dot_term * rho_dot

        return cbf.generate_cbf_condition(hp, -self.LfB[idx] - tv_term, -self.LgB[idx, :], adaptive)

    def rho_law(self,
                B: float) -> (float, float):
        """Default risk value. """
        # rho0 = ((T + 1) * h0) / (Tmax * h0max)
        # rho0dot = (Tdot * h0 + (T + 1) * h0dot) / (Tmax * h0max)

        # return t / Tmax, tdot / Tmax
        # return np.max([B - 0.01, 0.01]), 0.0
        return 0.01, 0.0

    def get_risk(self,
                 B: float,
                 rmin: float,
                 rmax: float) -> (float, float):
        """Computes the tolerable risk for the controller."""
        # Parameters
        gain = 1000.0
        rho0, rho0_dot = self.rho_law(B)

        # Rho with bounded support
        rho = rmin + (rho0 - rmin) * ramp(rho0, gain, rmin) - (rho0 - (rmax - rmin)) * ramp(rho0, gain, (rmax - rmin))

        # Compute rhodot
        rho_dot = rho0_dot * ramp(rho0, gain, rmin) + (rho0 - rmin) * dramp(rho0, gain, rmin) * rho0_dot - \
                  rho0_dot * ramp(rho0, gain, rmax) - (rho0 - (rmax - rmin)) * dramp(rho0, gain, (rmax - rmin)) * rho0_dot

        return rho, rho_dot

    def AB(self):
        return self.LfB + self.LgB[:, self.nu * self.ego_id: self.nu * (self.ego_id + 1)] @ self.u








#
#
#
#
#
#     # Maybe this needs to be modified, maybe not
#     def __init__(self,
#                  objective_function: Callable,
#                  nominal_controller: Callable,
#                  ignore: List = None):
#         super().__init__()
#         self.objective = objective_function
#         self.nominal_controller = nominal_controller
#         self.ignored_agents = ignore
#         self.code = 0
#         self.status = "Initialized"
#
#     def _compute_control(self,
#                          t: float,
#                          z: NDArray) -> (NDArray, NDArray, int, str, float):
#         """Computes the vehicle's control input based on the Risk-Bounded CBF framework.
#
#         INPUTS
#         ------
#         t: time (in sec)
#         z: full state vector for all vehicles
#         extras: anything else
#
#         OUTPUTS
#         ------
#         u_act: actual control input used in the system
#         u_nom: nominal input used if safety not considered
#         code: error/success code
#         status: more info on error/success
#
#         """
#         # Ignore agent if necessary (i.e. if comparing controllers for given initial conditions)
#         ego = self.ego_id
#         if self.ignored_agents is not None:
#             for ignore in self.ignored_agents:
#                 z = np.delete(z, ignore, 0)
#                 if ego > ignore:
#                     ego = ego - 1
#
#         # Partition state into ego and other
#         ze = z[ego, :]
#         zo = np.vstack([z[:ego, :], z[ego + 1:, :]])
#
#         # Compute nominal control input for ego only -- assume others are zero
#         u_nom = np.zeros((len(z), 2))
#         u_nom[ego, :] = self.nominal_controller(t, ze, ego)
#         self.u_nom = u_nom[ego, :]
#
#         # Get matrices and vectors for QP controller
#         Q, p, A, b, G, h, cbf, LfB, LgB = self.formulate_qp(t, ze, zo, u_nom, ego)
#
#         # Solve QP
#         sol = solve_qp_cvxopt(Q, p, A, b, G, h)
#
#         # Check solution
#         if not sol['code']:
#             self.u = np.zeros((A.shape[1],))
#         else:
#             self.u = np.array(sol['x'][2 * ego: 2 * (ego + 1)]).flatten()
#
#         return sol['code'], sol['status'],
#
#     def formulate_qp(self,
#                      t: float,
#                      ze: NDArray,
#                      zz: NDArray,
#                      u_nom: NDArray,
#                      ego: int) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray, float):
#         """Configures the Quadratic Program parameters (Q, p for objective function, A, b for inequality constraints,
#         G, h for equality constraints).
#
#         """
#         # Parameters
#         na = 1 + len(zz)
#         ns = len(ze)
#
#         # Objective function
#         Q, p = self.objective(u_nom.flatten())
#
#         # Input constraints (accel only)
#         au = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
#         Au = block_diag(*na * [au])
#         bu = np.array(na * [np.pi / 4, np.pi / 4, 9.81, 9.81])
#
#         # Initialize inequality constraints
#         Ai = np.zeros((len(zz), self.nu * na))
#         bi = np.zeros((len(zz),))
#
#         for ii, zo in enumerate(zz):
#             idx = ii + (ii >= ego)
#
#             # CBF and Partial Derivatives
#             h0 = H0(ze, zo)
#             h0dot = dH0dx(ze, zo)[:Ns] @ f(ze) + dH0dx(ze, zo)[Ns:] @ f(zo)
#             h = H(ze, zo)
#             dhdx = dHdx(ze, zo)
#             d2hdx2 = d2Hdx2(ze, zo)
#
#             # Lie Derivatives
#             Lfh = dhdx[:Ns] @ f(ze) + dhdx[Ns:] @ f(zo) + 0.5 * (np.trace(sigma(ze).T @ d2hdx2[:Ns, :Ns] @ sigma(ze)) +
#                                                                  np.trace(sigma(zo).T @ d2hdx2[Ns:, Ns:] @ sigma(zo)))
#             Lgh = np.zeros((Na * Nu,))
#             Lgh[Nu * ego:Nu * (ego + 1)] = dhdx[:Ns] @ g(ze)
#             Lgh[Nu * idx:Nu * (idx + 1)] = dhdx[:Ns] @ g(zo)
#
#             # Risk-Bounded CBF
#             k = 0.1
#             B = np.exp(-k * float(h))
#             LfB = -k * B * Lfh
#             LgB = -k * B * Lgh
#             LsB = -k * B * dhdx @ np.diag(np.concatenate([np.diagonal(sigma(ze)), np.diagonal(sigma(zo))]))
#
#             # Define required parameters -- hard coded for now
#             rho_max = 0.90
#             T = np.max([0, float(Tau([TauStar(ze, zo)]))])
#             Tdot = 0.0  # TauDot(ze, zo)  # need to do
#             gamma = B
#
#             # Approach for eta: assume that the distance between the vehicles will never be greater than it is now
#             eta = np.linalg.norm(np.abs(LsB))
#             class_k = 10.0  # used when alpha = 10, beta = 4
#
#             # Define derived parameters
#             global INTEGRATOR_STATE
#             rho_min = 0.01
#             if eta > 0:
#                 rho_min_updated = 1 - erf(np.nan_to_num((1 - gamma) / (np.sqrt(2) * T * eta), nan=np.inf))
#                 if rho_min_updated > rho_min:
#                     rho_min = rho_min_updated
#
#             rho, rhodot = get_adaptive_risk(T, h0, h0dot, rho_min, rho_max)
#             hp = 1 - gamma - np.sqrt(2) * eta * T * erfinv(1 - rho) - INTEGRATOR_STATE
#
#             erfdot_term = np.sqrt(np.pi) / 2 * np.exp(-(erfinv(1 - rho)) ** 2)
#             tv_term = np.sqrt(2) * eta * Tdot * erfinv(1 - rho) - np.sqrt(2) * eta * T * erfdot_term * rhodot
#
#             # CBF Condition
#             Ai[ii, :] = LgB
#             bi[ii] = class_k * hp - LfB - tv_term
#
#             if h0 < 0:
#                 print("SAFETY VIOLATION: {:.2f}".format(-h0))
#
#             cbf[ii, :] = np.array([h, h0])
#
#         A = np.vstack([Au, Ai])
#         b = np.hstack([bu, bi])
#
#         return Q, p, A, b, G, h, cbf, LfB, LgB
#
#
# ###############################################################################
# ################################## Functions ##################################
# ###############################################################################
#
#
# def compute_control(t: float,
#                     z: NDArray,
#                     extras: dict) -> (NDArray, NDArray, int, str, float):
#     """ Solves
#
#     INPUTS
#     ------
#     t: time (in sec)
#     z: state vector (Ax5) -- A is number of agents
#     extras: contains time additional information needed to compute the control
#
#     OUTPUTS
#     -------
#     u_act: actual control input vector (2x1) = (time-derivative of body slip angle, rear-wheel acceleration)
#     u_0: nominal control input vector (2x1)
#     code: error/success code (0 or 1)
#     status: string containing error message (if relevant)
#     cbf: min cbf value
#
#     """
#     # Error checking variables
#
#     ego = extras['agent']
#
#     # Ignore agent if necessary (i.e. if comparing controllers for given initial conditions)
#     if 'ignore' in extras.keys():
#         ignore = extras['ignore']
#         z = np.delete(z, ignore, 0)
#         if ego > ignore:
#             ego = ego - 1
#
#     # Partition ego (e) and other (o) states
#     ze = z[ego, :]
#     zo = np.vstack([z[:ego, :], z[ego+1:, :]])
#
#     # # Compute nominal control inputs
#     # u_nom = np.zeros((len(z), 2))
#     # for aa, zz in enumerate(z):
#     #     omega, ar = compute_nominal_control(t, zz, aa)
#     #     u_nom[aa, :] = np.array([omega, ar])
#
#     # Compute nominal control input for ego only -- assume others are zero
#     u_nom = np.zeros((len(z), 2))
#     omega, ar = compute_nominal_control(t, ze, ego)
#     u_nom[ego, :] = np.array([omega, ar])
#
#     # # Get matrices and vectors for QP controller
#     # Q, p, A, b, G, h, cbf, LfB, LgB = get_constraints_accel_only(t, ze, zo, u_nom, ego)
#
#     # Get matrices and vectors for QP controller
#     Q, p, A, b, G, h, cbf, LfB, LgB = get_constraints(t, ze, zo, u_nom, ego)
#
#     # Solve QP
#     sol = solve_qp_cvxopt(Q, p, A, b, G, h)
#
#     # If accel-only QP is infeasible, add steering control as decision variable
#     if not sol['code']:
#         # # Get matrices and vectors for QP controller
#         # Q, p, A, b, G, h, cbf, LfB, LgB = get_constraints_accel_and_steering(t, ze, zo, u_nom, ego)
#         #
#         # # Solve QP
#         # sol = solve_qp_cvxopt(Q, p, A, b, G, h)
#         #
#         # # Return error if this is also infeasible
#         # if not sol['code']:
#         return np.zeros((2,)), u_nom[ego, :], sol['code'], sol['status'], np.zeros((3, 2)),
#
#         # # Format control solution -- accel and steering solution
#         # u_act = np.array(sol['x'][2 * ego: 2 * (ego + 1)]).flatten()
#
#     else:
#         # Format control solution -- nominal steering, accel solution
#         u_act = np.array([u_nom[ego, 0], sol['x'][ego]])
#
#     u_act = np.clip(u_act, [-np.pi / 4, -9.81], [np.pi / 4, 9.81])
#     u_0 = u_nom[ego, :]
#
#     # Augmented for integrator state
#     global INTEGRATOR_STATE
#     INTEGRATOR_STATE = INTEGRATOR_STATE + (LfB + LgB[ego] * u_act[1]) * dt
#
#     return u_act, u_0, sol['code'], sol['status'], cbf
#
#
# ############################# Safe Control Inputs #############################
#
#
# def get_constraints(t: float,
#                     ze: NDArray,
#                     zz: NDArray,
#                     u_nom: NDArray,
#                     ego: int) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray, float):
#     """ Generates and returns inter-agent objective and constraint functions for the single-agent QP controller
#     of the form:
#
#     J = u.T * Q * u + p * u
#     subject to
#     Au <= b
#     Gu = h
#
#     INPUTS
#     ------
#     t: time (in sec)
#     za: state vector for agent in question (5x1)
#     zo: array of state vectors for remaining agents in system (Ax5)
#     u_nom: array of nominal control inputs
#     agent: index for agent in question
#
#     OUTPUTS
#     -------
#     Q: (AUxAU) positive semi-definite matrix for quadratic decision variables in QP objective function
#     p: (AUx1) vector for decision variables with linear terms
#     A: ((A-1)xAU) matrix multiplying decision variables in inequality constraints for QP
#     b: ((A-1)x1) vector for inequality constraints for QP
#     G: None -- no equality constraints right now
#     h: None -- no equality constraints right now
#     cbf: cbf val
#
#     """
#     # Parameters
#     Na = 1 + len(zz)
#     Nu = 2
#     Ns = len(ze)
#
#     cbf = np.zeros((zz.shape[0], 2))
#
#     # Objective function
#     Q, p = objective_accel_and_steering(u_nom.flatten())
#
#     # Input constraints (accel only)
#     au = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
#     Au = block_diag(*Na*[au])
#     bu = np.array(Na*[np.pi / 4, np.pi / 4, 9.81, 9.81])
#
#     # Initialize inequality constraints
#     Ai = np.zeros((len(zz), Nu*Na))
#     bi = np.zeros((len(zz),))
#
#     for ii, zo in enumerate(zz):
#         idx = ii + (ii >= ego)
#
#         # CBF and Partial Derivatives
#         h0 = H0(ze, zo)
#         h0dot = dH0dx(ze, zo)[:Ns] @ f(ze) + dH0dx(ze, zo)[Ns:] @ f(zo)
#         h = H(ze, zo)
#         dhdx = dHdx(ze, zo)
#         d2hdx2 = d2Hdx2(ze, zo)
#
#         # Lie Derivatives
#         Lfh = dhdx[:Ns] @ f(ze) + dhdx[Ns:] @ f(zo) + 0.5 * (np.trace(sigma(ze).T @ d2hdx2[:Ns, :Ns] @ sigma(ze)) +
#                                                              np.trace(sigma(zo).T @ d2hdx2[Ns:, Ns:] @ sigma(zo)))
#         Lgh = np.zeros((Na*Nu,))
#         Lgh[Nu * ego:Nu * (ego + 1)] = dhdx[:Ns] @ g(ze)
#         Lgh[Nu * idx:Nu * (idx + 1)] = dhdx[:Ns] @ g(zo)
#
#         # Risk-Bounded CBF
#         k = 0.1
#         B = np.exp(-k * float(h))
#         LfB = -k * B * Lfh
#         LgB = -k * B * Lgh
#         LsB = -k * B * dhdx @ np.diag(np.concatenate([np.diagonal(sigma(ze)), np.diagonal(sigma(zo))]))
#
#         # Define required parameters -- hard coded for now
#         rho_max = 0.90
#         T = np.max([0, float(Tau([TauStar(ze, zo)]))])
#         Tdot = 0.0  # TauDot(ze, zo)  # need to do
#         gamma = B
#
#         # Approach for eta: assume that the distance between the vehicles will never be greater than it is now
#         eta = np.linalg.norm(np.abs(LsB))
#         class_k = 10.0  # used when alpha = 10, beta = 4
#
#         # Define derived parameters
#         global INTEGRATOR_STATE
#         rho_min = 0.01
#         if eta > 0:
#             rho_min_updated = 1 - erf(np.nan_to_num((1 - gamma) / (np.sqrt(2) * T * eta), nan=np.inf))
#             if rho_min_updated > rho_min:
#                 rho_min = rho_min_updated
#
#         rho, rhodot = get_adaptive_risk(T, h0, h0dot, rho_min, rho_max)
#         hp = 1 - gamma - np.sqrt(2) * eta * T * erfinv(1 - rho) - INTEGRATOR_STATE
#
#         erfdot_term = np.sqrt(np.pi) / 2 * np.exp(-(erfinv(1 - rho)) ** 2)
#         tv_term = np.sqrt(2) * eta * Tdot * erfinv(1 - rho) - np.sqrt(2) * eta * T * erfdot_term * rhodot
#
#         # CBF Condition
#         Ai[ii, :] = LgB
#         bi[ii] = class_k * hp - LfB - tv_term
#
#         if h0 < 0:
#             print("SAFETY VIOLATION: {:.2f}".format(-h0))
#
#         cbf[ii, :] = np.array([h, h0])
#
#     A = np.vstack([Au, Ai])
#     b = np.hstack([bu, bi])
#
#     return Q, p, A, b, None, None, cbf, LfB, LgB
#
#
# def get_adaptive_risk(T: float,
#                       h0: float,
#                       h0dot: float,
#                       rmin: float,
#                       rmax: float) -> (float, float):
#     """Computes the tolerable risk for the controller."""
#     # Parameters
#     h0max = 50
#     rho0 = ((T + 1) * h0) / (Tmax * h0max)
#     Tdot = 0.0  # Need to correct
#
#     # Compute rho
#     rho = rho0 * ramp(rho0, 1000, rmin) - (rho0 - rmax) * ramp(rho0, 1000, rmax)
#
#     # Compute rhodot
#     rho0dot = (Tdot * h0 + (T + 1) * h0dot) / (Tmax * h0max)
#     rhodot = rho0dot * ramp(rho0, 1000, rmin) + rho0 * dramp(rho0, 1000, rmin) - \
#              rho0dot * ramp(rho0, 1000, rmax) - (rho0 - rmax) * dramp(rho0, 1000, rmax)
#
#     return 0.99, 0.0
#     return rho, rhodot
#
# ################################################
#
#
# def get_constraints_accel_only(t: float,
#                                ze: NDArray,
#                                zz: NDArray,
#                                u_nom: NDArray,
#                                ego: int) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray, float):
#     """ Generates and returns inter-agent objective and constraint functions for the single-agent QP controller
#     of the form:
#
#     J = u.T * Q * u + p * u
#     subject to
#     Au <= b
#     Gu = h
#
#     INPUTS
#     ------
#     t: time (in sec)
#     ze: state vector for ego agent (5x1)
#     zo: array of state vectors for remaining agents in system (Ax5)
#     u_nom: array of nominal control inputs
#     agent: index for agent in question
#
#     OUTPUTS
#     -------
#     Q: (AUxAU) positive semi-definite matrix for quadratic decision variables in QP objective function
#     p: (AUx1) vector for decision variables with linear terms
#     A: ((A-1)xAU) matrix multiplying decision variables in inequality constraints for QP
#     b: ((A-1)x1) vector for inequality constraints for QP
#     G: None -- no equality constraints right now
#     h: None -- no equality constraints right now
#     cbf: cbf value
#
#     """
#     # Parameters
#     Na = 1 + len(zz)
#     Ns = len(ze)
#     discretization_error = 0.0
#     cbf = np.zeros((zz.shape[0], 2))
#
#     # Objective function
#     Q, p = objective_accel_only(u_nom[:, 1], ego)
#
#     # Input constraints (accel only)
#     au = np.array([1, -1])
#     Au = block_diag(*Na*[au]).T
#     bu = np.array(2*Na*[9.81])
#
#     # Initialize inequality constraints
#     Ai = np.zeros((2*len(zz), Na))
#     bi = np.zeros((2*len(zz),))
#
#     for ii, zo in enumerate(zz):
#         idx = ii + (ii >= ego)
#
#         # omega -- assume zero for now
#         omega_e = u_nom[ego, 0]
#         omega_o = u_nom[idx, 0]
#
#         # CBF and Partial Derivatives
#         h0 = H0(ze, zo)
#         h = H(ze, zo)
#         dhdx = dHdx(ze, zo)
#         d2hdx2 = d2Hdx2(ze, zo)
#
#         # Lie Derivatives
#         Lfh = dhdx[:Ns] @ (f(ze) + g(ze)[:, 0] * omega_e) + dhdx[Ns:] @ (f(zo) + g(zo)[:, 0] * omega_o) + \
#               0.5 * (np.trace(sigma(ze).T @ d2hdx2[:Ns, :Ns] @ sigma(ze)) -
#                      np.trace(sigma(zo).T @ d2hdx2[Ns:, Ns:] @ sigma(zo)))
#         Lgh = np.zeros((Na,))
#         Lgh[ego] = dhdx[:Ns] @ g(ze)[:, 1]
#         Lgh[idx] = dhdx[:Ns] @ g(zo)[:, 1]
#
#         # Risk-Bounded CBF
#         k = 0.1
#         B = np.exp(-k * float(h))
#         LfB = -k * B * Lfh
#         LgB = -k * B * Lgh
#         LsB = -k * B * dhdx @ np.diag(np.concatenate([np.diagonal(sigma(ze)), np.diagonal(sigma(zo))]))
#
#         # Define required parameters -- hard coded for now
#         rho_max = 0.5  # Maximum allowable risk
#         T = float(Tau([TauStar(ze, zo)]))
#         Tdot = 0.0  # TauDot(ze, zo)  # need to do
#         gamma = B
#
#         # Approach for eta: assume that the distance between the vehicles will never be greater than it is now
#         eta = np.linalg.norm(np.abs(LsB))
#         class_k = 2.5  # used when alpha = 10, beta = 4
#
#         # Define derived parameters
#         global INTEGRATOR_STATE
#         if eta > 0:
#             rho_min = 1 - erf(np.nan_to_num((1 - gamma) / (np.sqrt(2) * T * eta)), nan=np.inf)
#         else:
#             rho_min = 0.001
#
#         kT = np.log(rho_max / rho_min) / Tmax
#         rho = rho_min * np.exp(kT * T)
#         rho_dot = rho_min * kT * np.exp(kT * T) * Tdot
#         hp = 1 - gamma - np.sqrt(2) * eta * T * erfinv(1 - rho) - INTEGRATOR_STATE
#
#         erfdot_term = np.sqrt(np.pi) / 2 * np.exp(-(erfinv(1 - rho))**2)
#         tv_term = np.sqrt(2) * eta * Tdot * erfinv(1 - rho) - np.sqrt(2) * eta * T * erfdot_term * rho_dot
#
#         # CBF Condition
#         Ai[ii, :] = LgB
#         bi[ii] = class_k * hp - LfB - tv_term
#
#         if h0 < 0:
#             print("SAFETY VIOLATION: {:.2f}".format(-h0))
#
#         cbf[ii, :] = np.array([hp, h0])
#
#     A = np.vstack([Au, Ai])
#     b = np.hstack([bu, bi])
#
#     return Q, p, A, b, None, None, cbf, LfB, LgB
#
#
# def get_constraints_accel_only_old(t: float,
#                                    za: NDArray,
#                                    zo: NDArray,
#                                    u_nom: NDArray,
#                                    agent: int) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray):
#     """ Generates and returns inter-agent objective and constraint functions for the single-agent QP controller
#     of the form:
#
#     J = u.T * Q * u + p * u
#     subject to
#     Au <= b
#     Gu = h
#
#     INPUTS
#     ------
#     t: time (in sec)
#     za: state vector for agent in question (5x1)
#     zo: array of state vectors for remaining agents in system (Ax5)
#     u_nom: array of nominal control inputs
#     agent: index for agent in question
#
#     OUTPUTS
#     -------
#     Q: (AUxAU) positive semi-definite matrix for quadratic decision variables in QP objective function
#     p: (AUx1) vector for decision variables with linear terms
#     A: ((A-1)xAU) matrix multiplying decision variables in inequality constraints for QP
#     b: ((A-1)x1) vector for inequality constraints for QP
#     G: None -- no equality constraints right now
#     h: None -- no equality constraints right now
#
#     """
#     # Parameters
#     Na = 1 + len(zo)
#     discretization_error = 0.5
#
#     # Objective function
#     Q, p = objective_accel_only(u_nom[:, 1], agent)
#
#     # Input constraints (accel only)
#     au = np.array([1, -1])
#     Au = block_diag(*Na*[au]).T
#     bu = np.array(2*Na*[9.81])
#
#     # Initialize inequality constraints
#     Ai = np.zeros((len(zo), Na))
#     bi = np.zeros((len(zo),))
#
#     for ii, zz in enumerate(zo):
#         idx = ii + (ii >= agent)
#
#         # omega -- assume zero for now
#         omega_a = u_nom[agent, 0]
#         omega_z = u_nom[idx, 0]
#
#         # x and y differentials
#         dx = za[0] - zz[0]
#         dy = za[1] - zz[1]
#
#         # vx and vy differentials
#         dvx = f(za)[0] - f(zz)[0]
#         dvy = f(za)[1] - f(zz)[1]
#
#         # ax and ay differentials (uncontrolled)
#         axa_unc = -za[3] / Lr * np.tan(za[4]) * f(za)[1] - omega_a * za[3] * np.sin(za[2]) / np.cos(za[4])**2
#         aya_unc = za[3] / Lr * np.tan(za[4]) * f(za)[0] + omega_a * za[3] * np.cos(za[2]) / np.cos(za[4])**2
#         axo_unc = -zz[3] / Lr * np.tan(zz[4]) * f(zz)[1] - omega_z * zz[3] * np.sin(zz[2]) / np.cos(zz[4])**2
#         ayo_unc = zz[3] / Lr * np.tan(zz[4]) * f(zz)[0] + omega_z * zz[3] * np.cos(zz[2]) / np.cos(zz[4])**2
#         dax_unc = axa_unc - axo_unc
#         day_unc = aya_unc - ayo_unc
#
#         # ax and ay differentials (controlled)
#         axa_con = np.zeros((Na,))
#         aya_con = np.zeros((Na,))
#         axo_con = np.zeros((Na,))
#         ayo_con = np.zeros((Na,))
#         axa_con[agent] = np.cos(za[2]) - np.sin(za[2]) * np.tan(za[4])
#         aya_con[agent] = np.sin(za[2]) + np.cos(za[2]) * np.tan(za[4])
#         axo_con[idx] = np.cos(zz[2]) - np.sin(zz[2]) * np.tan(zz[4])
#         ayo_con[idx] = np.sin(zz[2]) + np.cos(zz[2]) * np.tan(zz[4])
#         dax_con = axa_con - axo_con
#         day_con = aya_con - ayo_con
#
#         # x scale: designed to enforce larger safety distance in direction of travel
#         x_scale = 1  #0.2 -- < 1 for highway scenario
#         dx = x_scale * dx
#         dvx = x_scale * dvx
#         dax_unc = x_scale * dax_unc
#         dax_con = x_scale * dax_con
#
#         # Build da's
#         dax = [dax_unc, dax_con]
#         day = [day_unc, day_con]
#
#         # CVaR CBF
#         h0 = dx**2 + dy**2 - (2*R)**2
#         ht, tau, tau_dot_unc, tau_dot_con = get_risk_cbf(dx, dy, dvx, dvy, dax, day, za[3], 'Expectation')
#
#         # CBF Derivatives (Lie derivative notation -- Lfh = dh/dx * f(x))
#         Lfh0 = 2 * (dx * dvx + dy * dvy)
#         Lfht = Lfh0 + 2 * tau * (dvx**2 + dvy**2 + dx * dax_unc + dy * day_unc) + 2 * tau_dot_unc * \
#                (dx * dvx + dy * dvy + tau * (dvx**2 + dvy**2)) + 2 * tau**2 * (dvx * dax_unc + dvy * day_unc)
#         Lght = 2 * tau * tau_dot_con * (dvx**2 + dvy**2) + 2 * tau**2 * (dvx * dax_con + dvy * day_con) + \
#                2 * tau_dot_con * (dx * dvx + dy * dvy) + 2 * tau * (dx * dax_con + dy * day_con)
#
#         # CBF Condition: Lfht + Lght + l0 * ht >= 0 --> Au <= b
#         l0 = 20.0
#         Ai[ii, :] = -Lght
#         bi[ii] = Lfht + l0 * ht
#
#         if h0 < 0:
#             print("SAFETY VIOLATION: {:.2f}".format(-h0))
#
#     A = np.vstack([Au, Ai])
#     b = np.hstack([bu, bi])
#
#     return Q, p, A, b, None, None
#
#
# def get_risk_cbf(dx: float,
#                  dy: float,
#                  dvx: float,
#                  dvy: float,
#                  dax: list,
#                  day: list,
#                  vel: float,
#                  risk: str) -> (float, float, float, NDArray):
#     """ Builds pdf and returns the Conditional Value-at-Risk CBF.
#
#     INPUTS
#     ------
#     dx: differential x position
#     dy: differential y  position
#     dvx: differential x velocity
#     dvy: differential y velocity
#     dax: list consisting of dax_uncontrolled and dax_controlled
#     day: list consisting of day_uncontrolled and day_controlled
#     vel: velocity of other vehicle
#     risk: risk metric to be considered for CBF
#
#     OUTPUTS
#     -------
#     cvarCBF: conditional value-at-risk cbf
#
#     """
#     # Unpack dax, day
#     dax_unc = dax[0]
#     dax_con = dax[1]
#     day_unc = day[0]
#     day_con = day[0]
#
#     # Solve for minimizer (tau) of ||[dx dy] + [dvx dvy]t|| from 0 to T
#     T = 5.0
#     kh = 1000.0
#     epsilon = 1e-3
#     tau_star = -(dx * dvx + dy * dvy) / (dvx ** 2 + dvy ** 2 + epsilon)
#     tau = tau_star * ramp(tau_star, kh, 0.0) - (tau_star - T) * ramp(tau_star, kh, T)
#
#     # # Get n samples from ht -- assumed Gaussian
#     # n = 50
#     # mean_diff_pos = np.array([dx, dy]) + np.array([dvx, dvy]) * tau
#     # covariance = np.array([[1.0, 0.0], [0.0, 1.0]])
#     # ffcbf_samples = []
#     # silvermans = 3  # ((4 * covariance[0, 0]) / (3 * n))**(1 / 5) # Not really Silverman's in current implementation
#     # for ii in range(n):
#     #     sample_x = gauss(mean_diff_pos[0], covariance[0,0])
#     #     sample_y = gauss(mean_diff_pos[1], covariance[1,1])
#     #     ffcbf_samples.append(sample_x**2 + sample_y**2 - (2*R)**2)
#
#     # Get all available samples from data within 1 sec of computed tau value
#
#     # Get relevant tau samples
#     indices_t1 = np.where(ngsim_highway_data[:, 1, 4] < tau + 0.5)[0]
#     indices_t2 = np.where(ngsim_highway_data[:, 1, 4] > tau - 0.5)[0]
#     indices_t = np.intersect1d(indices_t1, indices_t2)
#
#     # Get relevant velocity samples
#     indices_v1 = np.where(ngsim_highway_data[:, 0, 5] < vel + 2.5)[0]
#     indices_v2 = np.where(ngsim_highway_data[:, 0, 5] > vel - 2.5)[0]
#     indices_v = np.intersect1d(indices_v1, indices_v2)
#
#     # Synthesize all relevant samples
#     indices = np.intersect1d(indices_t, indices_v)
#
#     nSamples = 100
#     dx = ngsim_highway_data[indices, 1, 2] * FT_TO_M
#     dy = ngsim_highway_data[indices, 1, 3] * FT_TO_M
#     ffcbf_samples = dx ** 2 + dy ** 2 - (2*R)**2
#     ffcbf_samples = resample(ffcbf_samples, nSamples)
#     ffcbf_samples.sort()
#     ds = np.ediff1d(ffcbf_samples)  # (np.max(ffcbf_samples) - np.min(ffcbf_samples))
#
#     # Get ht pdf values and stats
#     confidence = np.max([1 - tau / T, 0.0])  # Adaptive confidence interval: relax as tau increases
#     bandwidth = ((4 * np.std(ffcbf_samples)**5) / (3 * nSamples))**(1 / 5)  # Silverman's rule of thumb
#     ht_pdf = kde_pdf(data=ffcbf_samples, kernel_func=gaussian_pdf, bandwidth=bandwidth)
#     cum_sum = np.cumsum([ds[ee] * ht_pdf(sample) for ee, sample in enumerate(ffcbf_samples[:-1])])
#     normalized_cdf = cum_sum / np.max(cum_sum)
#     var = ffcbf_samples[np.max(np.where(normalized_cdf < (1 - confidence)))]  # Value-at-Risk
#
#     # Risk Metrics
#     var_idx = np.max(np.where(np.array(ffcbf_samples) < var))
#     expectation_normalization = np.sum([ht_pdf(sample) for sample in ffcbf_samples])
#     expected_value = sum([sample * ht_pdf(sample) for sample in ffcbf_samples]) / expectation_normalization
#     cvar_normalization = np.sum([ht_pdf(sample) for sample in ffcbf_samples[:var_idx]])
#     cvar = sum([sample * ht_pdf(sample) for sample in ffcbf_samples[:var_idx]]) / cvar_normalization
#
#     # Get Expected Value
#     if risk.lower() == 'expectation':
#         ht = expected_value
#     elif risk.lower() == 'cvar':
#         ht = cvar
#     else:
#         ht = 0
#
#     print("Expectation: {:.2f}".format(expected_value))
#     print("CVaR: {:.2f}".format(cvar))
#
#     # # In-line debugging
#     # import matplotlib.pyplot as plt
#     # import seaborn as sns
#     #
#     # sns.set(color_codes=True)
#     # plt.rcParams["figure.figsize"] = (15, 10)
#     #
#     # fig = plt.figure()
#     #
#     # ax1 = fig.add_subplot(1, 1, 1)
#     # y_exp = [ht_pdf(i) for i in ffcbf_samples]
#     # y_cvar = [ht_pdf(i) for i in ffcbf_samples[:var_idx+1]]
#     # ax1.scatter(ffcbf_samples[var_idx], y_exp[var_idx])
#     # ax1.plot(ffcbf_samples, y_exp)
#     # ax1.plot(ffcbf_samples[:var_idx+1], y_cvar)
#     # plt.tight_layout()
#     # plt.show()
#
#     # Derivatives of tau (controllable and uncontrollable)
#     tau_star_dot_unc = -(dax_unc * (2 * dvx * tau_star + dx) + day_unc * (2 * dvy * tau_star + dy) +
#                          (dvx ** 2 + dvy ** 2)) / (dvx ** 2 + dvy ** 2 + epsilon)
#     tau_star_dot_con = -(dax_con * (2 * dvx * tau_star + dx) + day_con * (2 * dvy * tau_star + dy)) / \
#                        (dvx ** 2 + dvy ** 2 + epsilon)
#     tau_dot_unc = tau_star_dot_unc * (ramp(tau_star, kh, 0.0) - ramp(tau_star, kh, T)) + \
#                   tau_star * tau_star_dot_unc * (dramp(tau_star, kh, 0.0) - dramp(tau_star, kh, T))
#     tau_dot_con = tau_star_dot_con * (ramp(tau_star, kh, 0.0) - ramp(tau_star, kh, T)) + \
#                   tau_star * tau_star_dot_con * (dramp(tau_star, kh, 0.0) - dramp(tau_star, kh, T))
#
#     return ht, tau, tau_dot_unc, tau_dot_con
#
#
# #
# #     # CBF Definitions
# #     h0 = dx ** 2 + dy ** 2 - (2 * R) ** 2
# #     ht = h0 + tau ** 2 * (dvx ** 2 + dvy ** 2) + 2 * tau * (dx * dvx + dy * dvy) - discretization_error
# #
# #     return cvarCBF
#
#
# def get_constraints_accel_and_steering_old(t: float,
#                                            za: NDArray,
#                                            zo: NDArray,
#                                            u_nom: NDArray,
#                                            agent: int) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray):
#     """ Generates and returns inter-agent objective and constraint functions for the single-agent QP controller
#     of the form:
#
#     J = u.T * Q * u + p * u
#     subject to
#     Au <= b
#     Gu = h
#
#     INPUTS
#     ------
#     t: time (in sec)
#     za: state vector for agent in question (5x1)
#     zo: array of state vectors for remaining agents in system (Ax5)
#     u_nom: array of nominal control inputs
#     agent: index for agent in question
#
#     OUTPUTS
#     -------
#     Q: (AUxAU) positive semi-definite matrix for quadratic decision variables in QP objective function
#     p: (AUx1) vector for decision variables with linear terms
#     A: ((A-1)xAU) matrix multiplying decision variables in inequality constraints for QP
#     b: ((A-1)x1) vector for inequality constraints for QP
#     G: None -- no equality constraints right now
#     h: None -- no equality constraints right now
#
#     """
#     # Parameters
#     Na = 1 + len(zo)
#     Nu = 2
#     discretization_error = 0.5
#
#     # Objective function
#     Q, p = objective_accel_and_steering(u_nom.flatten())
#
#     # Input constraints (accel only)
#     au = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
#     Au = block_diag(*Na*[au])
#     bu = np.array(Na*[np.pi / 4, np.pi / 4, 9.81, 9.81])
#
#     # Initialize inequality constraints
#     Ai = np.zeros((len(zo), Nu*Na))
#     bi = np.zeros((len(zo),))
#
#     for ii, zz in enumerate(zo):
#         idx = ii + (ii >= agent)
#
#         # x and y differentials
#         dx = za[0] - zz[0]
#         dy = za[1] - zz[1]
#
#         # vx and vy differentials
#         dvx = f(za)[0] - f(zz)[0]
#         dvy = f(za)[1] - f(zz)[1]
#
#         # ax and ay differentials (uncontrolled)
#         axa_unc = -za[3] / Lr * np.tan(za[4]) * f(za)[1]
#         aya_unc = za[3] / Lr * np.tan(za[4]) * f(za)[0]
#         axo_unc = -zz[3] / Lr * np.tan(zz[4]) * f(zz)[1]
#         ayo_unc = zz[3] / Lr * np.tan(zz[4]) * f(zz)[0]
#         dax_unc = axa_unc - axo_unc
#         day_unc = aya_unc - ayo_unc
#
#         # ax and ay differentials (controlled)
#         axa_con = np.zeros((Nu * Na,))
#         aya_con = np.zeros((Nu * Na,))
#         axo_con = np.zeros((Nu * Na,))
#         ayo_con = np.zeros((Nu * Na,))
#         axa_con[Nu * agent:Nu * (agent + 1)] = np.array([-za[3] * np.sin(za[2]) / np.cos(za[4])**2,
#                                                          np.cos(za[2]) - np.sin(za[2]) * np.tan(za[4])])
#         aya_con[Nu * agent:Nu * (agent + 1)] = np.array([za[3] * np.cos(za[2]) / np.cos(za[4])**2,
#                                                          np.sin(za[2]) + np.cos(za[2]) * np.tan(za[4])])
#         axo_con[Nu * idx:Nu * (idx + 1)] = np.array([-zz[3] * np.sin(zz[2]) / np.cos(zz[4])**2,
#                                                      np.cos(zz[2]) - np.sin(zz[2]) * np.tan(zz[4])])
#         ayo_con[Nu * idx:Nu * (idx + 1)] = np.array([zz[3] * np.cos(zz[2]) / np.cos(zz[4])**2,
#                                                      np.sin(zz[2]) + np.cos(zz[2]) * np.tan(zz[4])])
#         dax_con = axa_con - axo_con
#         day_con = aya_con - ayo_con
#
#         # x scale: designed to enforce larger safety distance in direction of travel
#         x_scale = 1  #0.2 -- < 1 for highway scenario
#         dx = x_scale * dx
#         dvx = x_scale * dvx
#         dax_unc = x_scale * dax_unc
#         dax_con = x_scale * dax_con
#
#         # Solve for minimizer (tau) of ||[dx dy] + [dvx dvy]t|| from 0 to T
#         T = 3.0
#         kh = 1000.0
#         epsilon = 1e-3
#         tau_star = -(dx * dvx + dy * dvy) / (dvx**2 + dvy**2 + epsilon)
#         tau = tau_star * ramp(tau_star, kh, 0.0) - (tau_star - T) * ramp(tau_star, kh, T)
#
#         # Derivatives of tau (controllable and uncontrollable)
#         tau_star_dot_unc = -(dax_unc * (2 * dvx * tau_star + dx) + day_unc * (2 * dvy * tau_star + dy) +
#                              (dvx**2 + dvy**2)) / (dvx**2 + dvy**2 + epsilon)
#         tau_star_dot_con = -(dax_con * (2 * dvx * tau_star + dx) + day_con * (2 * dvy * tau_star + dy)) / \
#                            (dvx**2 + dvy**2 + epsilon)
#         tau_dot_unc = tau_star_dot_unc * (ramp(tau_star, kh, 0.0) - ramp(tau_star, kh, T)) + \
#                       tau_star * tau_star_dot_unc * (dramp(tau_star, kh, 0.0) -
#                                                      dramp(tau_star, kh, T))
#         tau_dot_con = tau_star_dot_con * (ramp(tau_star, kh, 0.0) - ramp(tau_star, kh, T)) + \
#                       tau_star * tau_star_dot_con * (dramp(tau_star, kh, 0.0) -
#                                                      dramp(tau_star, kh, T))
#
#         # CBF Definitions
#         h0 = dx**2 + dy**2 - (2*R)**2
#         ht = h0 + tau**2 * (dvx**2 + dvy**2) + 2 * tau * (dx * dvx + dy * dvy) - discretization_error
#
#         # CBF Derivatives (Lie derivative notation -- Lfh = dh/dx * f(x))
#         Lfh0 = 2 * (dx * dvx + dy * dvy)
#         Lfht = Lfh0 + 2 * tau * (dvx**2 + dvy**2 + dx * dax_unc + dy * day_unc) + 2 * tau_dot_unc * \
#                (dx * dvx + dy * dvy + tau * (dvx**2 + dvy**2)) + 2 * tau**2 * (dvx * dax_unc + dvy * day_unc)
#         Lght = 2 * tau * tau_dot_con * (dvx**2 + dvy**2) + 2 * tau**2 * (dvx * dax_con + dvy * day_con) + \
#                2 * tau_dot_con * (dx * dvx + dy * dvy) + 2 * tau * (dx * dax_con + dy * day_con)
#
#         # CBF Condition: Lfht + Lght + l0 * ht >= 0 --> Au <= b
#         l0 = 20.0
#         Ai[ii, :] = -Lght
#         bi[ii] = Lfht + l0 * ht
#
#         if h0 < 0:
#             print("SAFETY VIOLATION: {:.2f}".format(-h0))
#
#     A = np.vstack([Au, Ai])
#     b = np.hstack([bu, bi])
#
#     return Q, p, A, b, None, None
