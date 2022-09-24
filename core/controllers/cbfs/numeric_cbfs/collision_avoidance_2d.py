from sys import exit
import numpy as np
import sympy as sp
import builtins
from nptyping import NDArray
from importlib import import_module
from core.cbf_wrappers import symbolic_cbf_wrapper_multiagent, symbolic_cbf_wrapper_singleagent
from core.mathematics.analytical_functions import ramp, dramp, d2ramp

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
mod = vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    globals().update({'ss': getattr(module, 'sym_state')})
except ModuleNotFoundError:
    print('No module named \'{}\' -- exiting.'.format(mod))
    exit()




# Defining Physical Params
R = 1.0


def h_ca(ze: NDArray,
         zo: NDArray):
    """Nominal interagent distance CBF for collision avoidance (ca). """
    return (ze[0] - zo[0])**2 + (ze[1] - zo[1])**2 - (2 * R)**2


def dhdx_ca(ze: NDArray,
            zo: NDArray):
    """Nominal interagent distance dCBFdx for collision avoidance (ca). """
    return np.array([2 * ze[0],
                     2 * ze[1],
                     0,
                     0,
                     0,
                     -2 * ze[0],
                     -2 * ze[1],
                     0,
                     0,
                     0])


def d2hdx2_ca(ze: NDArray,
              zo: NDArray):
    """Nominal interagent distance dCBFdx for collision avoidance (ca). """
    return np.array([[2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 2, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, -2, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, -2, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


T = 3.0
kh = 1000.0
epsilon = 1e-3


def differentials(ze: NDArray,
                  zo: NDArray) -> (float, float, float, float):
    """Returns position and velocity differentials dx, dy, dvx, dvy between two agents ego (e) and other (o)."""
    dx = ze[0] - zo[0]
    dy = ze[1] - zo[1]
    dvx = f(ze)[0] - f(zo)[0]
    dvy = f(ze)[1] - f(zo)[1]

    return dx, dy, dvx, dvy

def tau_star(ze, zo):
    dx, dy, dvx, dvy = differentials(ze, zo)
    return -(dx * dvx + dy * dvy) / (dvx ** 2 + dvy ** 2 + epsilon)


def tau(ze: NDArray,
        zo: NDArray):
    """Estimate of time at which predicted future minimum interagent distance occurs."""
    ts = tau_star(ze, zo)
    return ts * ramp(ts, kh, 0.0) - (ts - T) * ramp(ts, kh, T)


def dtaustardx(ze: NDArray,
               zo: NDArray) -> NDArray:
    dx, dy, dvx, dvy = differentials(ze, zo)
    F = -(dx * dvx + dy * dvy)
    dFdx = -np.array([dvx,
                      dvy,
                      dy * f(ze)[0] - dx * f(ze)[1],
                      dx * (np.cos(ze[2]) - np.sin(ze[2]) * np.tan(ze[4])) +
                      dy * (np.sin(ze[2]) + np.cos(ze[2]) * np.tan(ze[4])),
                      (dy * ze[3] * np.cos(ze[2]) - dx * ze[3] * np.sin(ze[2])) / np.cos(ze[4])**2,
                      -dvx,
                      -dvy,
                      dx * f(zo)[1] - dy * f(zo)[0],
                      -dx * (np.cos(zo[2]) - np.sin(zo[2]) * np.tan(zo[4])) -
                      dy * (np.sin(zo[2]) + np.cos(zo[2]) * np.tan(zo[4])),
                      -(dy * zo[3] * np.cos(zo[2]) - dx * zo[3] * np.sin(zo[2])) / np.cos(zo[4]) ** 2])
    G = dvx**2 + dvy**2 + epsilon
    dGdx = 2 * dvx * np.array([0,
                               0,
                               -f(ze)[1],
                               np.cos(ze[2]) - np.sin(ze[2]) * np.tan(ze[4]),
                               -ze[3] * np.sin(ze[2]) / np.cos(ze[4])**2,
                               0,
                               0,
                               f(zo)[1],
                               -np.cos(zo[2]) + np.sin(zo[2]) * np.tan(zo[4]),
                               zo[3] * np.sin(zo[2]) / np.cos(zo[4]) ** 2]) + \
           2 * dvy * np.array([0,
                               0,
                               f(ze)[0],
                               np.sin(ze[2]) + np.cos(ze[2]) * np.tan(ze[4]),
                               ze[3] * np.cos(ze[2]) / np.cos(ze[4])**2,
                               0,
                               0,
                               -f(zo)[0],
                               -np.sin(zo[2]) - np.cos(zo[2]) * np.tan(zo[4]),
                               -zo[3] * np.cos(zo[2]) / np.cos(zo[4]) ** 2])

    return (dFdx * G - dGdx * F) / G**2


def dtaudtaustar(tau_star):
    """Partial derivative of tau with respect to tau_star."""
    return T * dramp(tau_star, kh, 0.0)


def d2taudtaustar2(tau_star):
    """Partial derivative of tau with respect to tau_star."""
    return T * d2ramp(tau_star, kh, 0.0)








def h_pca(ze: NDArray,
          zo: NDArray):
    """Predictive interagent distance CBF for collision avoidance (pca). """
    tau_val = tau(ze, zo)
    dx, dy, dvx, dvy = differentials(ze, zo)

    return (dx + tau_val * dvx)**2 + (dy + tau_val * dvy)**2 - (2 * R)**2


def dhdtau(ze, zo):
    tau_val = tau(ze, zo)
    dx, dy, dvx, dvy = differentials(ze, zo)

    return 2 * dvx * (dx + tau_val * dvx) + 2 * dvy * (dy + tau_val * dvy)


def d2hdtau2(ze, zo):
    dx, dy, dvx, dvy = differentials(ze, zo)
    return 2 * dvx**2 + 2 * dvy**2


def dhdx_pca(ze: NDArray,
             zo: NDArray):
    """Predictive interagent distance dCBFdx for collision avoidance (pca). """
    tau_val = tau(ze, zo)
    dx, dy, dvx, dvy = differentials(ze, zo)
    dhdx_0 = 2 * (dx + tau_val * dvx) * np.array([1,
                                                  0,
                                                  tau_val * -f(ze)[1],
                                                  tau_val * (np.cos(ze[2]) - np.sin(ze[2]) * np.tan(ze[4])),
                                                  -tau_val * ze[3] * np.sin(ze[2]) / np.cos(ze[4])**2,
                                                  -1,
                                                  -0,
                                                  -tau_val * -f(zo)[1],
                                                  -tau_val * (np.cos(zo[2]) - np.sin(zo[2]) * np.tan(zo[4])),
                                                  tau_val * zo[3] * np.sin(zo[2]) / np.cos(zo[4]) ** 2]) + \
             2 * (dy + tau_val * dvy) * np.array([0,
                                                  1,
                                                  tau_val * f(ze)[0],
                                                  tau_val * (np.sin(ze[2]) + np.cos(ze[2]) * np.tan(ze[4])),
                                                  tau_val * ze[3] * np.cos(ze[2]) / np.cos(ze[4])**2,
                                                  0,
                                                  -1,
                                                  -tau_val * f(zo)[0],
                                                  -tau_val * (np.sin(zo[2]) + np.cos(zo[2]) * np.tan(zo[4])),
                                                  -tau_val * zo[3] * np.cos(zo[2]) / np.cos(zo[4]) ** 2])

    return dhdx_0 + dhdtau(ze, zo) * dtaudtaustar(tau_star(ze, zo)) * dtaustardx(ze, zo)


def d2hdx2_pca(ze: NDArray,
               zo: NDArray):
    """Predictive interagent distance dCBFdx for collision avoidance (pca). """
    ts = tau_star(ze, zo)
    dtsdx = dtaustardx(ze, zo)
    outer = np.outer(dtsdx, dtsdx)
    d2hdx2_0 = np.array([0])
    dtdts = dtaudtaustar(ts)
    dhdt = dhdtau(ze, zo)



    return d2hdx2_0 + dtdts * d2hdtau2(ze, zo) * dtdts * outer + \
          dhdt * d2taudtaustar2(ts) * outer + dhdt * d2taustardx2(ze, zo) * dtdts


# Define new symbols -- necessary for pairwise interactions case
sso = sp.symbols(['{}o'.format(n) for n in ss], real=True)
dx = ss[0] - sso[0]
dy = ss[1] - sso[1]

# Collision Avoidance CBF
h_ca_symbolic = (ss[0] - sso[0])**2 + (ss[1] - sso[1])**2 - (2 * R)**2
dhdx_ca_symbolic = sp.Matrix([h_ca_symbolic], real=True).jacobian(np.concatenate([ss, sso]))
d2hdx2_ca_symbolic = dhdx_ca_symbolic.jacobian(np.concatenate([ss, sso]))
h_ca = symbolic_cbf_wrapper_multiagent(h_ca_symbolic, ss, sso)
dhdx_ca = symbolic_cbf_wrapper_multiagent(dhdx_ca_symbolic, ss, sso)
d2hdx2_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_ca_symbolic, ss, sso)

# Tau Formulation for PCA-CBF
vxe = f(np.zeros((len(ss),)), True)[0]
vye = f(np.zeros((len(ss),)), True)[1]
vxo = vxe
vyo = vye
for se, so in zip(ss, sso):
    vxo = vxo.replace(se, so)
    vyo = vyo.replace(se, so)
dvx = vxe - vxo
dvy = vye - vyo

tau_sym = sp.symbols('tau', real=True)

# tau* for computing tau
epsilon = 1e-3
tau_star_symbolic = -(dx * dvx + dy * dvy) / (dvx ** 2 + dvy ** 2 + epsilon)
dtaustardx_symbolic = sp.Matrix([tau_star_symbolic], real=True).jacobian(np.concatenate([ss, sso]))
d2taustardx2_symbolic = dtaustardx_symbolic.jacobian(np.concatenate([ss, sso]))
tau_star = symbolic_cbf_wrapper_multiagent(tau_star_symbolic, ss, sso)
dtaustardx = symbolic_cbf_wrapper_multiagent(dtaustardx_symbolic, ss, sso)
d2taustardx2 = symbolic_cbf_wrapper_multiagent(d2taustardx2_symbolic, ss, sso)

# tau for computing PCA-CBF
Tmax = 3.0
kh = 1000.0
tau_star_sym = sp.symbols('tau_star')
tau_symbolic = tau_star_sym * ramp(tau_star_sym, kh, 0.0) - (tau_star_sym - Tmax) * ramp(tau_star_sym, kh, Tmax)
dtaudtaustar_symbolic = sp.Matrix([tau_symbolic], real=True).jacobian(np.array([tau_star_sym]))
d2taudtaustar2_symbolic = dtaudtaustar_symbolic.jacobian(np.array([tau_star_sym]))
tau = symbolic_cbf_wrapper_singleagent(tau_symbolic, [tau_star_sym])
dtaudtaustar = symbolic_cbf_wrapper_singleagent(dtaudtaustar_symbolic, [tau_star_sym])
d2taudtaustar2 = symbolic_cbf_wrapper_singleagent(d2taudtaustar2_symbolic, [tau_star_sym])

# Predictive Collision Avoidance CBF
h_predictive_ca_symbolic = (dx + tau_sym * dvx)**2 + (dy + tau_sym * dvy)**2 - (2 * R)**2
dhdx_predictive_ca_symbolic = sp.Matrix([h_predictive_ca_symbolic], real=True).jacobian(np.concatenate([ss, sso]))
dhdtau_predictive_ca_symbolic = sp.Matrix([h_predictive_ca_symbolic], real=True).jacobian(np.array([tau_sym]))
d2hdx2_predictive_ca_symbolic = dhdx_predictive_ca_symbolic.jacobian(np.concatenate([ss, sso]))
d2hdtau2_predictive_ca_symbolic = dhdtau_predictive_ca_symbolic.jacobian(np.array([tau_sym]))
d2hdtaudx_predictive_ca_symbolic = dhdtau_predictive_ca_symbolic.jacobian(np.concatenate([ss, sso]))
h_predictive_ca = symbolic_cbf_wrapper_multiagent(h_predictive_ca_symbolic, ss, sso)
dhdx_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdx_predictive_ca_symbolic, ss, sso)
dhdtau_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdtau_predictive_ca_symbolic, ss, sso)
d2hdx2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtaudx_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtau2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdtau2_predictive_ca_symbolic, ss, sso)

# Relaxed Predictive Collision Avoidance
relaxation = 0.1


def h_pca(ego, other):
    return h_predictive_ca(ego, other).subs(tau_sym, tau([tau_star(ego, other)]))


def dhdx_pca(ego, other):
    ret = dhdx_predictive_ca(ego, other).subs(tau_sym, tau([tau_star(ego, other)])) + \
          dhdtau_predictive_ca(ego, other).subs(tau_sym, tau([tau_star(ego, other)])) * \
          dtaudtaustar([tau_star(ego, other)]) * dtaustardx(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_pca(ego, other):
    d2hdx2_eval = d2hdx2_predictive_ca(ego, other).subs(tau_sym, tau([tau_star(ego, other)]))
    dtaustardx_eval = dtaustardx(ego, other)
    dtaudtaustar_eval = float(dtaudtaustar([tau_star(ego, other)]))
    dhdtau_eval = dhdtau_predictive_ca(ego, other).subs(tau_sym, tau([tau_star(ego, other)]))[0]
    d2hdtau2_eval = float(d2hdtau2_predictive_ca(ego, other))
    d2taudtaustar2_eval = d2taudtaustar2([tau_star(ego, other)])
    d2taustardx2_eval = d2taustardx2(ego, other)
    outer = np.outer(dtaustardx_eval, dtaustardx_eval)

    ret = d2hdx2_eval + dtaudtaustar_eval * d2hdtau2_eval * dtaudtaustar_eval * outer + \
          dhdtau_eval * d2taudtaustar2_eval * outer + dhdtau_eval * d2taustardx2_eval * dtaudtaustar_eval

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rpca(ego, other):
    return relaxation * h_ca(ego, other) + h_pca(ego, other)


def dhdx_rpca(ego, other):
    return relaxation * dhdx_ca(ego, other) + dhdx_pca(ego, other)


# Necessary for stochastic systems
def d2hdx2_rpca(ego, other):
    return relaxation * d2hdx2_ca(ego, other) + d2hdx2_pca(ego, other)


if __name__ == '__main__':
    # This is a unit test
    ze = np.array([1, 2, 3, 4, 5])
    zo = np.array([5, 6, 7, 8, 9])
    print(h_collision_avoidance(ze, zo))
    print(h_predictive_collision_avoidance(ze, zo))
    print(dhdx_collision_avoidance(ze, zo))
    print(dhdx_predictive_collision_avoidance(ze, zo))
    print(d2hdx2_collision_avoidance(ze, zo))
    print(d2hdx2_predictive_collision_avoidance(ze, zo))
    print('stop')



