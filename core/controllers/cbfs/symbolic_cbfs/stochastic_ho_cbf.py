import builtins
import numpy as np
import symengine as se
from importlib import import_module
from nptyping import NDArray
from .collision_avoidance_2d import h_ca_symbolic, ss, sso
from simdycosys.core.controllers.cbfs.cbf_wrappers import symbolic_cbf_wrapper_multiagent, \
    symbolic_cbf_wrapper_singleagent

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
system_model = builtins.PROBLEM_CONFIG['system_model']
mod = 'simdycosys.' + vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    globals().update({'sigma': getattr(module, 'sigma_{}'.format(system_model))})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e


# High-Order CBF Parameters
ns = len(ss)
k0 = 0.25
k1 = k0
beta0 = 0.01
alpha0 = 0.1

# Necessary Symbols
f_ss = f(np.zeros((len(ss),)), True)
f_sso = f(np.zeros((len(ss),)), True)
sigma_ss = sigma(np.zeros((len(ss),)), True)
sigma_sso = sigma(np.zeros((len(ss),)), True)
for sego, sother in zip(ss, sso):
    f_sso = f_sso.replace(sego, sother)
    sigma_sso = sigma_sso.replace(sego, sother)
f_sym = se.DenseMatrix([f_ss] + [f_sso])
s_sym = se.DenseMatrix([sigma_ss] + [sigma_sso])

# Symbolic Stochastic CBF
b_symbolic = se.exp(-k0 * h_ca_symbolic)
dbdx_symbolic = se.DenseMatrix([b_symbolic]).jacobian(se.DenseMatrix(ss + sso))
d2bdx2_symbolic = dbdx_symbolic.jacobian(se.DenseMatrix(ss + sso))
d3bdx3_symbolic = d2bdx2_symbolic.jacobian(se.DenseMatrix(ss + sso))

# HO-Stochastic-CBF
AB_symbolic = dbdx_symbolic * f_sym + 0.5 * (s_sym.T * d2bdx2_symbolic * s_sym).trace()
c_symbolic = se.exp(-k1 * (beta0 - alpha0 * b_symbolic - AB_symbolic))
dcdx_symbolic = se.DenseMatrix([c_symbolic]).jacobian(se.DenseMatrix(ss + sso))
d2cdx2_symbolic = dcdx_symbolic.jacobian(se.DenseMatrix(ss + sso))

# Numerical HO-Stochastic-CBF
c = symbolic_cbf_wrapper_multiagent(c_symbolic, ss, sso)
dcdx = symbolic_cbf_wrapper_multiagent(dcdx_symbolic, ss, sso)
d2cdx2 = symbolic_cbf_wrapper_multiagent(d2cdx2_symbolic, ss, sso)


def C(ego, other) -> float:
    """High-Order Stochastic CBF."""
    return c(ego, other)


def dCdx(ego, other) -> NDArray:
    """First partial derivative of HO-S-CBF wrt x."""
    return np.squeeze(np.array(dcdx(ego, other)).astype(np.float64))


def d2Cdx2(ego, other) -> NDArray:
    """Second partial derivative of S-CBF wrt x."""
    return np.squeeze(np.array(d2cdx2(ego, other)).astype(np.float64))


