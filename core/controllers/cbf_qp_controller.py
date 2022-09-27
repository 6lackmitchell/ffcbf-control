import builtins
import numpy as np
from sys import exit
from importlib import import_module
from nptyping import NDArray
from typing import Callable, List
from scipy.linalg import block_diag
from core.solve_cvxopt import solve_qp_cvxopt
from core.controllers.controller import Controller
from core.controllers.cbfs import Cbf

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

global integrated_error
integrated_error = np.zeros((25,))


class CbfQpController(Controller):

    _stochastic = False
    _generate_cbf_condition = None
    _dt = None

    def __init__(self,
                 u_max: List,
                 nAgents: int,
                 objective_function: Callable,
                 nominal_controller: Controller,
                 cbfs_individual: List,
                 cbfs_pairwise: List,
                 ignore: List = None):
        super().__init__()
        self.u_max = u_max
        self.na = nAgents
        self.objective = objective_function
        self.nominal_controller = nominal_controller
        self.cbfs_individual = cbfs_individual
        self.cbfs_pairwise = cbfs_pairwise
        self.ignored_agents = ignore
        self.code = 0
        self.status = "Initialized"
        self.nu = len(u_max)
        self.nv = 1  # Number of additional optimization variables
        self.cbf_vals = np.zeros((len(cbfs_individual) + (self.na - 1) * len(cbfs_pairwise)),)

        # Define individual input constraints
        self.au = block_diag(*self.nu * [np.array([[1, -1]]).T])
        self.bu = np.tile(np.array(self.u_max).reshape(self.nu, 1), self.nu).flatten()

    def _compute_control(self,
                         t: float,
                         z: NDArray,
                         cascaded: bool = False) -> (NDArray, NDArray, int, str, float):
        """Computes the vehicle's control input based on a cascaded approach: first, the CBF constraints attempt to
        filter out unsafe inputs on the first level. If no safe control exists, then all control inputs are eligible
        for safety filtering.

        INPUTS
        ------
        t: time (in sec)
        z: full state vector for all vehicles
        extras: anything else

        OUTPUTS
        ------
        u_act: actual control input used in the system
        u_nom: nominal input used if safety not considered
        code: error/success code
        status: more info on error/success

        """
        global integrated_error
        code = 0
        status = 'Incomplete'

        # Ignore agent if necessary (i.e. if comparing controllers for given initial conditions)
        ego = self.ego_id
        if self.ignored_agents is not None:
            self.ignored_agents.sort(reverse=True)
            for ignore in self.ignored_agents:
                z = np.delete(z, ignore, 0)
                if ego > ignore:
                    ego = ego - 1

        # Partition state into ego and other
        ze = z[ego, :]
        zo = np.vstack([z[:ego, :], z[ego + 1:, :]])

        # Compute nominal control input for ego only -- assume others are zero
        z_copy_nom = z.copy()
        z_copy_nom[self.ego_id] = z[ego]
        u_nom = np.zeros((len(z), 2))
        u_nom[ego, :], code_nom, status_nom = self.nominal_controller.compute_control(t, z_copy_nom)
        u_nom[ego, :] = u_nom[ego, :] + integrated_error[ego]
        self.u_nom = u_nom[ego, :]

        # print(integrated_error[ego])

        tuning_nominal = False
        if tuning_nominal:
            self.u = self.u_nom
            return self.u, 1, "Optimal"

        if not cascaded:
            # Get matrices and vectors for QP controller
            Q, p, A, b, G, h = self.formulate_qp(t, ze, zo, u_nom, ego)

            # Solve QP
            sol = solve_qp_cvxopt(Q, p, A, b, G, h)

            # Check solution
            if 'code' in sol.keys():
                code = sol['code']
                status = sol['status']
                self.assign_control(sol, ego)
                print("Actual Control: ({:.2f}, {:.2f})".format(self.u[0], self.u[1]))
                if abs(self.u[0]) > 1e-3:
                    pass
            else:
                status = 'Divide by Zero'
                self.u = np.zeros((self.nu,))

        else:
            pass

            # # Get matrices and vectors for QP controller
            # Q, p, A, b, G, h = self.formulate_qp(t, ze, zo, u_nom, ego, cascade=cascaded)
            #
            # # Solve QP
            # sol = solve_qp_cvxopt(Q, p, A, b, G, h)
            #
            # # Check solution
            # if 'code' in sol.keys():
            #     code = sol['code']
            #     status = sol['status']
            #
            #     if not code:
            #         if cascaded:
            #             Q, p, A, b, G, h = self.formulate_qp(t, ze, zo, u_nom, ego)
            #             sol = solve_qp_cvxopt(Q, p, A, b, G, h)
            #             if not sol['code']:
            #                 self.u = np.zeros((self.nu,))
            #             else:
            #                 self.assign_control(sol, ego)
            #         else:
            #             self.u = np.zeros((self.nu,))
            #     else:
            #         alf = np.array(sol['x'])[-1]
            #         self.assign_control(sol, ego)
            #
            # else:
            #     code = 0
            #     status = 'Divide by Zero'
            #     self.u = np.zeros((self.nu,))

        decay_const = 0.0  # needs to be < 1
        integrated_error[ego] = (np.linalg.norm(self.u - self.u_nom) + integrated_error[ego]) * decay_const

        return self.u, code, status

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

        # Configure QP Matrices
        # Q, p: objective function
        # Au, bu: input constraints
        if self.nv > 0:
            alpha_nom = 1.0
            # alpha_nom = 1000.0
            Q, p = self.objective(np.append(u_nom.flatten(), alpha_nom))
            Au = block_diag(*(na + self.nv) * [self.au])[:-2, :-1]
            bu = np.append(np.array(na * [self.bu]).flatten(), self.nv * [1e6, 0])
        else:
            Q, p = self.objective(u_nom.flatten())
            Au = block_diag(*(na) * [self.au])
            bu = np.array(na * [self.bu]).flatten()

        # Initialize inequality constraints
        lci = len(self.cbfs_individual)
        Ai = np.zeros((lci + len(zr), self.nu * na + self.nv))
        bi = np.zeros((lci + len(zr),))

        # Iterate over individual CBF constraints
        for cc, cbf in enumerate(self.cbfs_individual):
            h0 = cbf.h0(ze)
            h = cbf.h(ze)
            dhdx = cbf.dhdx(ze)

            # Stochastic Term -- 0 for deterministic systems
            if np.trace(sigma(ze).T @ sigma(ze)) > 0 and self._stochastic:
                d2hdx2 = cbf.d2hdx2(ze)
                stoch = 0.5 * np.trace(sigma(ze).T @ d2hdx2 @ sigma(ze))
            else:
                stoch = 0.0

            # Get CBF Lie Derivatives
            Lfh = dhdx @ f(ze) + stoch
            Lgh = np.zeros((self.nu * na,))
            Lgh[self.nu * ego:(ego + 1) * self.nu] = dhdx @ g(ze)  # Only assign ego control -- individual constraint
            if cascade:
                Lgh[self.nu * ego] = 0.0

            # Ai[cc, :], bi[cc] = self.generate_cbf_condition(cbf, h, Lfh, Lgh, cc, adaptive=True)
            Ai[cc, :], bi[cc] = self.generate_cbf_condition(cbf, h, Lfh, Lgh, cc, adaptive=True)
            self.cbf_vals[cc] = h
            if h0 < 0:
                self.safety = False

            if ego == 0:
                print("Individual CBF: {:.2f}".format(h))
                print("A Matrix: {}".format(Ai))
                print("b vector: {}".format(bi))

        # Iterate over pairwise CBF constraints
        for cc, cbf in enumerate(self.cbfs_pairwise):

            # Iterate over all other vehicles
            for ii, zo in enumerate(zr):
                idx = ii + (ii >= ego)

                h0 = cbf.h0(ze, zo)
                h = cbf.h(ze, zo)
                dhdx = cbf.dhdx(ze,  zo)

                # Stochastic Term -- 0 for deterministic systems
                if np.trace(sigma(ze).T @ sigma(ze)) > 0 and self._stochastic:
                    d2hdx2 = cbf.d2hdx2(ze, zo)
                    stoch = 0.5 * (np.trace(sigma(ze).T @ d2hdx2[:ns, :ns] @ sigma(ze)) +
                                   np.trace(sigma(zo).T @ d2hdx2[ns:, ns:] @ sigma(zo)))
                else:
                    stoch = 0.0

                # Get CBF Lie Derivatives
                Lfh = dhdx[:ns] @ f(ze) + dhdx[ns:] @ f(zo) + stoch
                Lgh = np.zeros((self.nu * na,))
                Lgh[self.nu * ego:(ego + 1) * self.nu] = dhdx[:ns] @ g(ze)
                if cascade:
                    Lgh[self.nu * ego] = 0.0
                # Lgh[self.nu * idx:(idx + 1) * self.nu] = dhdx[ns:] @ g(zo)  # Only allow ego to compensate for safety -- decentralized

                if h0 < 0:
                    print("{} SAFETY VIOLATION: {:.2f}".format(str(self.__class__).split('.')[-1], -h0))
                    self.safety = False

                update_idx = lci + cc * zr.shape[0] + ii
                Ai[update_idx, :], bi[update_idx] = self.generate_cbf_condition(cbf, h, Lfh, Lgh, update_idx, adaptive=True)
                self.cbf_vals[update_idx] = h

                if ego == 0:
                    print("Pairwise CBF: {:.2f}".format(h))
                    print("A Matrix: {}".format(Ai))
                    print("b vector: {}".format(bi))

        A = np.vstack([Au, Ai])
        b = np.hstack([bu, bi])

        return Q, p, A, b, None, None

    def generate_cbf_condition(self,
                               cbf: Cbf,
                               h: float,
                               Lfh: float,
                               Lgh: NDArray,
                               idx: int,
                               adaptive: bool = False) -> (NDArray, float):
        """Calls the child _generate_cbf_condition method. """
        if self._generate_cbf_condition is not None:
            return self._generate_cbf_condition(cbf, h, Lfh, Lgh, idx, adaptive)
        else:
            return cbf.generate_cbf_condition(h, Lfh, Lgh, adaptive)

    def assign_control(self,
                       solution: dict,
                       ego: int) -> None:
        """Assigns the control solution to the appropriate agent."""
        u = np.array(solution['x'][self.nu * ego: self.nu * (ego + 1)]).flatten()
        self.u = np.clip(u, -self.u_max, self.u_max)
        # Assign other agents' controls if this is a centralized node
        if hasattr(self, 'centralized_agents'):
            for agent in self.centralized_agents:
                agent.u = np.array(solution['x'][agent.nu * agent.id: self.nu * (agent.id + 1)]).flatten()


