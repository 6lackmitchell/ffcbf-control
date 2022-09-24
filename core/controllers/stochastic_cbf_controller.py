import numpy as np
import builtins
from typing import Callable, List
from importlib import import_module
from nptyping import NDArray
from scipy.stats import norm
from .cbfs.cbf import Cbf
from .cbf_qp_controller import CbfQpController

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


class StochasticCbfController(CbfQpController):

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

    def _generate_cbf_condition(self,
                                cbf: Cbf,
                                h: float,
                                Lfh: float,
                                Lgh: float,
                                idx: int,
                                adaptive: bool = False) -> (NDArray, float):
        """Generates the matrix A and vector b for the Risk-Bounded CBF constraint of the form Au <= b."""
        beta = 0.25
        k = 0.25
        B = np.exp(-k * h)
        LfB = -k * B * Lfh
        LgB = -k * B * Lgh

        return cbf.generate_stochastic_cbf_condition(B, LfB, LgB, beta, adaptive)

    def _generate_cvar_cbf_condition(self,
                                     cbf: Cbf,
                                     h: float,
                                     Lfh: float,
                                     Lgh: float,
                                     idx: int) -> (NDArray, float):
        """Generates the matrix A and vector b for the Stochastic CVaR CBF constraint of the form Au <= b."""
        beta = 0.01
        mean = 0.0
        stdev = 1.0  # Standard normal white noise (dw/dt)
        q_max = 0.95
        quantile = q_max
        # CVaR Parameters
        # q_max = 1 - (1e-15)
        # e_arg = np.nan_to_num((1 - B)**(4*B), nan=0.0, posinf=0.0, neginf=0.0)
        # quantile = np.min([q_max, np.exp(-e_arg)])
        cvar = mean + stdev * norm.pdf(norm.ppf(quantile, mean, stdev)) / (1 - quantile)
        # eta = np.sum(np.abs(dcbfdz(ze) @ sigma(ze) * cvar)) / dt
        eta = cvar / 0.01

        k = 0.25
        B = np.exp(-k * h)
        LfB[idx] = -k * B * Lfh
        LgB[idx, :] = -k * B * Lgh

        return cbf.generate_stochastic_cbf_condition(B, LfB, LgB, beta - eta)
