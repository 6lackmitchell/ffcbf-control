import numpy as np
import builtins
from typing import Callable, List, overload
from importlib import import_module
from nptyping import NDArray
from control import lqr
from scipy.linalg import block_diag, null_space
from .cbfs.cbf import Cbf
from core.controllers.cbf_qp_controller import CbfQpController
from core.controllers.controller import Controller
from core.solve_cvxopt import solve_qp_cvxopt

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
system_model = builtins.PROBLEM_CONFIG['system_model']
mod = vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    globals().update({'g': getattr(module, 'g')})
    globals().update({'sigma': getattr(module, 'sigma_{}'.format(system_model))})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e


class ConsolidatedCbfController(CbfQpController):

    P = None

    def __init__(self,
                 u_max: List,
                 nAgents: int,
                 objective_function: Callable,
                 nominal_controller: Controller,
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
        nCBF = len(self.cbf_vals)
        self.c_cbf = 100
        self.k_gains = 1.0 * np.ones((nCBF,))
        self.n_agents = 1

    def formulate_qp(self,
                     t: float,
                     ze: NDArray,
                     zr: NDArray,
                     u_nom: NDArray,
                     ego: int,
                     cascade: bool = False) -> (NDArray, NDArray, NDArray, NDArray, NDArray, NDArray, float):
        """Configures the Quadratic Program parameters (Q, p for objective function, A, b for inequality constraints,
        G, h for equality constraints).

        """
        # Parameters
        na = 1 + len(zr)
        ns = len(ze)
        self.safety = True
        discretization_error = 0.5

        # Only take ego nominal control
        # full_u_nom = u_nom
        # u_nom = u_nom.flatten()[self.nu * ego: (ego + 1) * self.nu]

        # Configure QP Matrices
        # Q, p: objective function
        # Au, bu: input constraints
        # if self.nv > 0:
        #     alpha_nom = 0.5
        #     Q, p = self.objective(np.append(u_nom, alpha_nom))
        #     Au = block_diag(*(1 + self.nv) * [self.au])[:-2, :-1]
        #     bu = np.append(np.array(1 * [self.bu]).flatten(), self.nv * [100, 0])
        # else:
        #     Q, p = self.objective(u_nom)
        #     Au = block_diag(*(1) * [self.au])
        #     bu = np.array(1 * [self.bu]).flatten()

        if self.nv > 0:
            # alpha_nom = 5.0
            # alpha_nom = 0.01
            alpha_nom = 1.0
            alpha_nom = 0.1
            Q, p = self.objective(np.append(u_nom.flatten(), alpha_nom))
            Au = block_diag(*(na + self.nv) * [self.au])[:-2, :-1]
            bu = np.append(np.array(na * [self.bu]).flatten(), self.nv * [1e6, 0])
        else:
            Q, p = self.objective(u_nom.flatten())
            Au = block_diag(*(na) * [self.au])
            bu = np.array(na * [self.bu]).flatten()

        # Initialize inequality constraints
        lci = len(self.cbfs_individual)
        h_array = np.zeros((len(self.cbf_vals,)))
        Lfh_array = np.zeros((len(self.cbf_vals,)))
        Lgh_array = np.zeros((len(self.cbf_vals), u_nom.flatten().shape[0]))

        # Iterate over individual CBF constraints
        for cc, cbf in enumerate(self.cbfs_individual):
            h0 = cbf.h0(ze)
            h_array[cc] = cbf.h(ze)
            dhdx = cbf.dhdx(ze)

            # Stochastic Term -- 0 for deterministic systems
            if np.trace(sigma(ze).T @ sigma(ze)) > 0 and self._stochastic:
                d2hdx2 = cbf.d2hdx2(ze)
                stoch = 0.5 * np.trace(sigma(ze).T @ d2hdx2 @ sigma(ze))
            else:
                stoch = 0.0

            # Get CBF Lie Derivatives
            Lfh_array[cc] = dhdx @ f(ze) + stoch - discretization_error
            Lgh_array[cc, self.nu * ego:(ego + 1) * self.nu] = dhdx @ g(ze)  # Only assign ego control
            if cascade:
                Lgh_array[cc, self.nu * ego] = 0.0

            self.cbf_vals[cc] = h_array[cc]
            if h0 < 0:
                self.safety = False

        # Iterate over pairwise CBF constraints
        for cc, cbf in enumerate(self.cbfs_pairwise):

            # Iterate over all other vehicles
            for ii, zo in enumerate(zr):
                other = ii + (ii >= ego)
                idx = lci + cc * zr.shape[0] + ii

                h0 = cbf.h0(ze, zo)
                h_array[idx] = cbf.h(ze, zo)
                dhdx = cbf.dhdx(ze,  zo)

                # Stochastic Term -- 0 for deterministic systems
                if np.trace(sigma(ze).T @ sigma(ze)) > 0 and self._stochastic:
                    d2hdx2 = cbf.d2hdx2(ze, zo)
                    stoch = 0.5 * (np.trace(sigma(ze).T @ d2hdx2[:ns, :ns] @ sigma(ze)) +
                                   np.trace(sigma(zo).T @ d2hdx2[ns:, ns:] @ sigma(zo)))
                else:
                    stoch = 0.0

                # Get CBF Lie Derivatives
                Lfh_array[idx] = dhdx[:ns] @ f(ze) + dhdx[ns:] @ f(zo) + stoch - discretization_error
                Lgh_array[idx, self.nu * ego:(ego + 1) * self.nu] = dhdx[:ns] @ g(ze)
                Lgh_array[idx, self.nu * other:(other + 1) * self.nu] = dhdx[ns:] @ g(zo)
                if cascade:
                    Lgh_array[idx, self.nu * ego] = 0.0

                if h0 < 0:
                    print("{} SAFETY VIOLATION: {:.2f}".format(str(self.__class__).split('.')[-1], -h0))
                    self.safety = False

                # print(np.linalg.norm(ze[0:2] - zo[0:2]))
                # print(h0)
                # if np.linalg.norm(ze[0:2] - zo[0:2]) < 2:
                #     uu = 1
                #     pass

                self.cbf_vals[idx] = h_array[idx]

        # Format inequality constraints
        Ai, bi = self.generate_consolidated_cbf_condition(ego, h_array, Lfh_array, Lgh_array)

        A = np.vstack([Au, Ai])
        b = np.hstack([bu, bi])

        return Q, p, A, b, None, None

    def generate_consolidated_cbf_condition(self,
                                            ego: int,
                                            h_array: NDArray,
                                            Lfh_array: NDArray,
                                            Lgh_array: NDArray) -> (NDArray, NDArray):
        """Generates the inequality constraint for the consolidated CBF.

        ARGUMENTS
            ego: ego vehicle id
            h: array of candidate CBFs
            Lfh: array of CBF drift terms
            Lgh: array of CBF control matrices

        RETURNS
            kdot: array of time-derivatives of k_gains
        """

        # Get C-CBF Value
        exp_term = np.exp(-self.k_gains * h_array)
        H = 1 - np.sum(exp_term)  # Get value of C-CBF
        self.c_cbf = H
        # print(H)

        # Non-centralized agents CBF dynamics become drifts
        # Lgh_uncontrolled = np.copy(Lgh_array[:, self.n_agents * self.nu:])
        Lgh_uncontrolled = np.copy(Lgh_array[:, :])
        Lgh_uncontrolled[:, ego * self.nu:(ego + 1) * self.nu] = 0
        Lgh_array[:, np.s_[:ego * self.nu]] = 0  # All indices before first ego index set to 0
        Lgh_array[:, np.s_[(ego + 1) * self.nu:]] = 0  # All indices after last ego index set to 0

        # Get time-derivatives of gains
        premultiplier_k = self.k_gains * exp_term
        premultiplier_h = h_array * exp_term
        k_dots = self.compute_k_dots(h_array, Lgh_array, premultiplier_k)
        # k_dots = np.zeros(k_dots.shape)  # Tuning nominal controller

        # Compute C-CBF Dynamics
        LfH = premultiplier_k @ Lfh_array + premultiplier_h @ k_dots
        LgH = premultiplier_k @ Lgh_array
        LgH_uncontrolled = premultiplier_k @ Lgh_uncontrolled

        # Tunable CBF Addition
        # kH = 0.1
        # kH = 0.01
        # kH = 0.5
        # kH = 0.75
        kH = 1.0
        phi = np.tile(-np.array(self.u_max), int(LgH_uncontrolled.shape[0] / len(self.u_max))) @ abs(LgH_uncontrolled) * np.exp(-kH * H)

        # Finish constructing CBF here
        a_mat = np.append(-LgH, -H)
        b_vec = np.array([LfH + phi])

        # Update k_dot
        self.k_gains = self.k_gains + k_dots * self._dt
        # print(self.k_gains)

        return a_mat[:, np.newaxis].T, b_vec

    def compute_k_dots(self,
                       h_array: NDArray,
                       Lgh_array: NDArray,
                       vec: NDArray) -> NDArray:
        """Computes the time-derivatives of the k_gains via QP based adaptation law.

        ARGUMENTS
            h_array: array of CBF values
            Lgh_array: 2D array of Lgh terms
            vec: vector that needs to stay out of the nullspace of matrix P

        RETURNS
            k_dot

        """
        threshold = 1e-4  # You must be at least this tall to ride the roller coaster (aka vec orthogonal minimum)
        gain = 10.0  # I don't know why I did this, but I did in my Matlab code
        Ao_basis = gain * null_space(Lgh_array.T)
        identity = np.eye(vec.shape[0])
        P_new = identity - (Ao_basis @ Ao_basis.T).T - Ao_basis @ Ao_basis.T + \
            (Ao_basis @ Ao_basis.T).T @ (Ao_basis @ Ao_basis.T)
        P_new = P_new / np.max(P_new)
        if self.P is not None:
            P_dot = (P_new - self.P) / self._dt
        else:
            P_dot = P_new / self._dt

        self.P = P_new  # Update P

        # Enforce CBF condition on the orthogonal component of vec
        ultra_conservative = False
        B = vec.T @ self.P @ vec - threshold**2
        LfB = vec.T @ P_dot @ vec
        LgB = 2 * vec.T @ P_new @ np.diag(np.exp(-self.k_gains * h_array) - vec * h_array.T)  # NEEDS DEBUGGING
        if ultra_conservative:
            eigs_P_dot, _ = np.linalg.eig(P_dot)
            LfB = np.min(eigs_P_dot) * np.linalg.norm(vec)**2

        # Constraints on k_gains
        k_min = 0.10
        As = -np.eye(self.k_gains.shape[0] + 1)
        As[-1, :] = np.append(-LgB, -B)
        bs = np.append(10 * (self.k_gains - k_min), LfB)

        # Get nominal k_dot
        k_dot_nom = self.k_dot_lqr(h_array)

        # Get QP params -- no constraints on alpha right now
        a_nom = 1.0
        lk = len(k_dot_nom) + 1
        Q = 1 / 2 * np.eye(lk)
        Q[-1, -1] = 10
        p = np.append(-k_dot_nom, -a_nom * Q[-1, -1])
        Au = block_diag(*lk * [np.array([[1, -1]]).T])
        bu = np.tile(1 / self._dt * np.ones((2,)), lk).flatten()
        bu[-2] = 1e6  # alpha < 1e6
        bu[-1] = 0    # alpha > 0
        A = np.concatenate([As, Au])
        b = np.concatenate([bs, bu])

        # Compute k_dot
        sol = solve_qp_cvxopt(Q, p, A, b, None, None)

        # Check solution
        if 'code' in sol.keys():
            code = sol['code']
            status = sol['status']

            if not code:
                k_dots = np.zeros((lk - 1,))
            else:
                alf = np.array(sol['x'])[-1]
                k_dots = np.array(sol['x']).flatten()[:-1]

        else:
            code = 0
            status = 'Divide by Zero'
            k_dots = np.zeros((lk - 1,))

        return k_dots

    def k_dot_lqr(self,
                  h_array: NDArray) -> NDArray:
        """Computes the nominal k_dot adaptation law based on LQR.

        Arguments
            h_array: array of CBF values

        RETURNS
            k_dot_nominal: nominal k_dot adaptation law

        """
        gain = 50.0
        gain = 1.0
        # k_star = gain * h_array / np.max([np.min(h_array), 0.5])  # Desired k values
        k_star = np.ones((len(h_array),))

        # Integrator dynamics
        Ad = np.zeros((self.k_gains.shape[0], self.k_gains.shape[0]))
        Bd = np.eye(self.k_gains.shape[0])

        # Compute LQR gain
        Q = 0.01 * Bd
        R = 1 * Bd
        K, _, _ = lqr(Ad, Bd, Q, R)

        k_dot_lqr = -K @ (self.k_gains - k_star)
        k_dot_bounds = 5.0
        # k_dot_bounds = 100.0

        return np.clip(k_dot_lqr, -k_dot_bounds, k_dot_bounds)
