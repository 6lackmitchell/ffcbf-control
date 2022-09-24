import builtins
import numpy as np
import symengine as se
from importlib import import_module
from core.controllers.cbfs.cbf_wrappers import symbolic_cbf_wrapper_singleagent

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
mod = vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    globals().update({'ss': getattr(module, 'sym_state')})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e

# Defining Physical Params
l0 = 1.0
th = 15.0 * np.pi / 180.0
relax = 0.05
box_width = 4
R = 2 * box_width * np.sqrt(2) + box_width / 2
Cxy = np.array([2 * box_width + box_width / 2, 2 * box_width + box_width / 2])
LW = 3.0
tau = 1.0
vx = f(np.zeros((len(ss),)), True)[0]
vy = f(np.zeros((len(ss),)), True)[1]

# Circular Region CBF Symbolic
h_radial_symbolic = R**2 - (ss[0] - Cxy[0])**2 - (ss[1] - Cxy[1])**2
dhdx_radial_symbolic = (se.DenseMatrix([h_radial_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_radial_symbolic = dhdx_radial_symbolic.jacobian(se.DenseMatrix(ss))
f_radial_symbolic = f(np.zeros((len(ss),)), True)
dfdx_radial_symbolic = f_radial_symbolic.jacobian(se.DenseMatrix(ss))
h_radial_func = symbolic_cbf_wrapper_singleagent(h_radial_symbolic, ss)
dhdx_radial_func = symbolic_cbf_wrapper_singleagent(dhdx_radial_symbolic, ss)
d2hdx2_radial_func = symbolic_cbf_wrapper_singleagent(d2hdx2_radial_symbolic, ss)
dfdx_radial_func = symbolic_cbf_wrapper_singleagent(dfdx_radial_symbolic, ss)


def h_radial(ego):
    return float(np.array(dhdx_radial_func(ego)).T @ f(ego)) + l0 * h_radial_func(ego)


def dhdx_radial(ego):
    ret = f(ego).T @ d2hdx2_radial_func(ego) + dhdx_radial_func(ego).T @ dfdx_radial_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_radial(ego):
    ret = d2hdx2_radial_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


if __name__ == '__main__':
    # This is a unit test
    ms = 97.5
    ze = np.array([-ms * np.cos(th), -ms * np.sin(th), th, 15.0, 0.0])

    print(h_road(ze))
    print(dhdx_road(ze))
    print(d2hdx2_road(ze))
    print('stop')



