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
th = 15.0 * np.pi / 180.0
relax = 0.05
R = 1.0
LW = 3.0
tau = 1.0
vx = f(np.zeros((len(ss),)), True)[0]
vy = f(np.zeros((len(ss),)), True)[1]

# Highway Safety Nominal CBF
h_hs_symbolic = (3 / 2 * LW - ss[1]) * (ss[1] - (-LW / 2))
dhdx_hs_symbolic = (se.DenseMatrix([h_hs_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_hs_symbolic = dhdx_hs_symbolic.jacobian(se.DenseMatrix(ss))
h_hs_func = symbolic_cbf_wrapper_singleagent(h_hs_symbolic, ss)
dhdx_hs_func = symbolic_cbf_wrapper_singleagent(dhdx_hs_symbolic, ss)
d2hdx2_hs_func = symbolic_cbf_wrapper_singleagent(d2hdx2_hs_symbolic, ss)

# Highway Safety Predictive CBF
h_phs_symbolic = (3 / 2 * LW - (ss[1] + vy * tau)) * ((ss[1] + vy * tau) - (-LW / 2))
dhdx_phs_symbolic = (se.DenseMatrix([h_phs_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_phs_symbolic = dhdx_phs_symbolic.jacobian(se.DenseMatrix(ss))
h_phs_func = symbolic_cbf_wrapper_singleagent(h_hs_symbolic, ss)
dhdx_phs_func = symbolic_cbf_wrapper_singleagent(dhdx_hs_symbolic, ss)
d2hdx2_phs_func = symbolic_cbf_wrapper_singleagent(d2hdx2_hs_symbolic, ss)

# On-Ramp Safety Nominal CBF
h_or_symbolic = (ss[0] * np.tan(th) + LW / (2 * np.cos(th)) - ss[1]) * (ss[1] - (ss[0] * np.tan(th) - LW / (2 * np.cos(th))))
dhdx_or_symbolic = (se.DenseMatrix([h_or_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_or_symbolic = dhdx_or_symbolic.jacobian(se.DenseMatrix(ss))
h_or_func = symbolic_cbf_wrapper_singleagent(h_or_symbolic, ss)
dhdx_or_func = symbolic_cbf_wrapper_singleagent(dhdx_or_symbolic, ss)
d2hdx2_or_func = symbolic_cbf_wrapper_singleagent(d2hdx2_or_symbolic, ss)

# On-Ramp Safety Predictive CBF
h_por_symbolic = ((ss[0] + vx * tau) * np.tan(th) + LW / (2 * np.cos(th)) - (ss[1] + vy * tau)) * \
                 ((ss[1] + vy * tau) - ((ss[0] + vx * tau) * np.tan(th) - LW / (2 * np.cos(th))))
dhdx_por_symbolic = (se.DenseMatrix([h_por_symbolic]).jacobian(se.DenseMatrix(ss))).T
d2hdx2_por_symbolic = dhdx_por_symbolic.jacobian(se.DenseMatrix(ss))
h_por_func = symbolic_cbf_wrapper_singleagent(h_por_symbolic, ss)
dhdx_por_func = symbolic_cbf_wrapper_singleagent(dhdx_por_symbolic, ss)
d2hdx2_por_func = symbolic_cbf_wrapper_singleagent(d2hdx2_por_symbolic, ss)


def h_hs(ego):
    return h_hs_func(ego)


def dhdx_hs(ego):
    ret = dhdx_hs_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_hs(ego):
    ret = d2hdx2_hs_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_phs(ego):
    return h_phs_func(ego)


def dhdx_phs(ego):
    ret = dhdx_phs_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_phs(ego):
    ret = d2hdx2_phs_func(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rphs(ego):
    return h_hs_func(ego) * relax + h_phs_func(ego)


def dhdx_rphs(ego):
    return dhdx_hs_func(ego) * relax + dhdx_phs_func(ego)


def d2hdx2_rphs(ego):
    return d2hdx2_hs_func(ego) * relax + d2hdx2_phs_func(ego)


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


def h0_road(ego):
    if ego[1] < -LW / 2:
        return h_or(ego)
    else:
        return h_hs(ego)


def h_road(ego):
    if ego[1] < -LW / 2:
        return h_rpor(ego)
    else:
        return h_rphs(ego)


def dhdx_road(ego):
    if ego[1] < -LW / 2:
        ret = dhdx_rpor(ego)
    else:
        ret = dhdx_rphs(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


def d2hdx2_road(ego):
    if ego[1] < -LW / 2:
        ret = d2hdx2_rpor(ego)
    else:
        ret = d2hdx2_rphs(ego)

    return np.squeeze(np.array(ret).astype(np.float64))


if __name__ == '__main__':
    # This is a unit test
    ms = 97.5
    ze = np.array([-ms * np.cos(th), -ms * np.sin(th), th, 15.0, 0.0])

    print(h_road(ze))
    print(dhdx_road(ze))
    print(d2hdx2_road(ze))
    print('stop')



