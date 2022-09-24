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
relax = 0.05
slope = 3
intercept = 0.0
R = 0.5
tau = 1.0
vx = f(np.zeros((len(ss),)), True)[0]
vy = f(np.zeros((len(ss),)), True)[1]

# Entryway Safety Nominal CBF
h_ew_symbolic = (ss[0] - R - (ss[1] - intercept) / slope) * (-ss[0] - R - (ss[1] - intercept) / slope)
dhdx_ew_symbolic = (se.DenseMatrix([h_ew_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_ew_symbolic = dhdx_ew_symbolic.jacobian(se.DenseMatrix(ss))
h_ew_func = symbolic_cbf_wrapper_singleagent(h_ew_symbolic, ss)
dhdx_ew_func = symbolic_cbf_wrapper_singleagent(dhdx_ew_symbolic, ss)
d2hdx2_ew_func = symbolic_cbf_wrapper_singleagent(d2hdx2_ew_symbolic, ss)

# Entryway Safety Predictive CBF
h_pew_symbolic = 10 * ((ss[0] + vx * tau) - R - ((ss[1] + vy * tau) - intercept) / slope) * (-(ss[0] + vx * tau) - R - ((ss[1] + vy * tau) - intercept) / slope)
dhdx_pew_symbolic = (se.DenseMatrix([h_pew_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_pew_symbolic = dhdx_pew_symbolic.jacobian(se.DenseMatrix(ss))
h_pew_func = symbolic_cbf_wrapper_singleagent(h_pew_symbolic, ss)
dhdx_pew_func = symbolic_cbf_wrapper_singleagent(dhdx_pew_symbolic, ss)
d2hdx2_pew_func = symbolic_cbf_wrapper_singleagent(d2hdx2_pew_symbolic, ss)

# Corridor Safety Nominal CBF
h_or_symbolic = (ss[0] + 1) * (1 - ss[0])
dhdx_or_symbolic = (se.DenseMatrix([h_or_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_or_symbolic = dhdx_or_symbolic.jacobian(se.DenseMatrix(ss))
h_or_func = symbolic_cbf_wrapper_singleagent(h_or_symbolic, ss)
dhdx_or_func = symbolic_cbf_wrapper_singleagent(dhdx_or_symbolic, ss)
d2hdx2_or_func = symbolic_cbf_wrapper_singleagent(d2hdx2_or_symbolic, ss)

# Corridor Safety Predictive CBF
h_por_symbolic = ((ss[0] + vx * tau) + 1) * (1 - (ss[0] + vx * tau))
dhdx_por_symbolic = (se.DenseMatrix([h_por_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_por_symbolic = dhdx_por_symbolic.jacobian(se.DenseMatrix(ss))
h_por_func = symbolic_cbf_wrapper_singleagent(h_por_symbolic, ss)
dhdx_por_func = symbolic_cbf_wrapper_singleagent(dhdx_por_symbolic, ss)
d2hdx2_por_func = symbolic_cbf_wrapper_singleagent(d2hdx2_por_symbolic, ss)


def h_ew(ego):
    return h_ew_func(ego)


def dhdx_ew(ego):
    ret = dhdx_ew_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_ew(ego):
    ret = d2hdx2_ew_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_pew(ego):
    return h_pew_func(ego)


def dhdx_pew(ego):
    ret = dhdx_pew_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_phs(ego):
    ret = d2hdx2_pew_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rpew(ego):
    return h_ew_func(ego) * relax + h_pew_func(ego)


def dhdx_rpew(ego):
    return dhdx_ew_func(ego) * relax + dhdx_pew_func(ego)


def d2hdx2_rpew(ego):
    return d2hdx2_ew_func(ego) * relax + d2hdx2_pew_func(ego)


def h_or(ego):
    return h_or_func(ego)


def dhdx_or(ego):
    ret = dhdx_or_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_or(ego):
    ret = d2hdx2_or_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_por(ego):
    return h_por_func(ego)


def dhdx_por(ego):
    ret = dhdx_por_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_por(ego):
    ret = d2hdx2_por_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rpor(ego):
    return h_or_func(ego) * relax + h_por_func(ego)


def dhdx_rpor(ego):
    return dhdx_or_func(ego) * relax + dhdx_por_func(ego)


def d2hdx2_rpor(ego):
    return d2hdx2_or_func(ego) * relax + d2hdx2_por_func(ego)

divider = 3


def h0_road(ego):
    if -divider < ego[1] < divider:
        return h_or(ego)
    else:
        return h_ew(ego)


def h_road(ego):
    if -divider < ego[1] < divider:
        return h_rpor(ego)
    else:
        return h_rpew(ego)


def dhdx_road(ego):
    if -divider < ego[1] < divider:
        ret = dhdx_rpor(ego)
    else:
        ret = dhdx_rpew(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_road(ego):
    if -divider < ego[1] < divider:
        ret = d2hdx2_rpor(ego)
    else:
        ret = d2hdx2_rpew(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


if __name__ == '__main__':
    # This is a unit test
    ms = 97.5
    ze = np.array([-ms * np.cos(th), -ms * np.sin(th), th, 15.0, 0.0])

    print(h_road(ze))
    print(dhdx_road(ze))
    print(d2hdx2_road(ze))
    print('stop')



