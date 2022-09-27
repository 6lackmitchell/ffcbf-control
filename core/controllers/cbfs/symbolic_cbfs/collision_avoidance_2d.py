import builtins
import numpy as np
import symengine as se
from importlib import import_module
from core.controllers.cbfs.cbf_wrappers import symbolic_cbf_wrapper_multiagent, \
    symbolic_cbf_wrapper_singleagent
from core.mathematics.symbolic_functions import ramp

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
mod = vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    # globals().update({'sigma': getattr(module, 'sigma')})
    globals().update({'ss': getattr(module, 'sym_state')})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e

# Defining Physical Params
R = 0.25

# Define new symbols -- necessary for pairwise interactions case
sso = se.symbols(['{}o'.format(n) for n in ss], real=True)
x_scale = 1.0
dx = (ss[0] - sso[0]) * x_scale
dy = ss[1] - sso[1]

# Collision Avoidance CBF
h_nominal_ca_symbolic = (ss[0] - sso[0])**2 + (ss[1] - sso[1])**2 - (2 * R)**2
dhdx_nominal_ca_symbolic = (se.DenseMatrix([h_nominal_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))).T
d2hdx2_nominal_ca_symbolic = dhdx_nominal_ca_symbolic.jacobian(se.DenseMatrix(ss + sso))
h_nominal_ca = symbolic_cbf_wrapper_multiagent(h_nominal_ca_symbolic, ss, sso)
dhdx_nominal_ca = symbolic_cbf_wrapper_multiagent(dhdx_nominal_ca_symbolic, ss, sso)
d2hdx2_nominal_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_nominal_ca_symbolic, ss, sso)

# Tau Formulation for PCA-CBF
vxe = f(np.zeros((len(ss),)), True)[0]
vye = f(np.zeros((len(ss),)), True)[1]
vxo = vxe
vyo = vye
for sego, sother in zip(ss, sso):
    vxo = vxo.replace(sego, sother)
    vyo = vyo.replace(sego, sother)
dvx = (vxe - vxo) * x_scale
dvy = vye - vyo

tau_sym = se.Symbol('tau', real=True)

# tau* for computing tau
epsilon = 1e-3
tau_star_symbolic = -(dx * dvx + dy * dvy) / (dvx ** 2 + dvy ** 2 + epsilon)
dtaustardx_symbolic = (se.DenseMatrix([tau_star_symbolic]).jacobian(se.DenseMatrix(ss + sso))).T
d2taustardx2_symbolic = dtaustardx_symbolic.jacobian(se.DenseMatrix(ss + sso))
tau_star = symbolic_cbf_wrapper_multiagent(tau_star_symbolic, ss, sso)
dtaustardx = symbolic_cbf_wrapper_multiagent(dtaustardx_symbolic, ss, sso)
d2taustardx2 = symbolic_cbf_wrapper_multiagent(d2taustardx2_symbolic, ss, sso)

# tau for computing PCA-CBF
Tmax = 10.0
kh = 1000.0
tau_star_sym = se.Symbol('tau_star', real=True)
tau_symbolic = tau_star_sym * ramp(tau_star_sym, kh, 0.0) - (tau_star_sym - Tmax) * ramp(tau_star_sym, kh, Tmax)
dtaudtaustar_symbolic = se.diff(tau_symbolic, tau_star_sym)
d2taudtaustar2_symbolic = se.diff(dtaudtaustar_symbolic, tau_star_sym)
tau = symbolic_cbf_wrapper_singleagent(tau_symbolic, [tau_star_sym])
dtaudtaustar = symbolic_cbf_wrapper_singleagent(dtaudtaustar_symbolic, [tau_star_sym])
d2taudtaustar2 = symbolic_cbf_wrapper_singleagent(d2taudtaustar2_symbolic, [tau_star_sym])

# Predictive Collision Avoidance CBF
h_predictive_ca_symbolic = (dx + tau_sym * dvx)**2 + (dy + tau_sym * dvy)**2 - (2 * R)**2
dhdx_predictive_ca_symbolic = (se.DenseMatrix([h_predictive_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))).T
dhdtau_predictive_ca_symbolic = se.diff(h_predictive_ca_symbolic, tau_sym)
d2hdx2_predictive_ca_symbolic = dhdx_predictive_ca_symbolic.jacobian(se.DenseMatrix(ss + sso))
d2hdtau2_predictive_ca_symbolic = se.diff(dhdtau_predictive_ca_symbolic, tau_sym)
d2hdtaudx_predictive_ca_symbolic = (se.DenseMatrix([dhdtau_predictive_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))).T
h_predictive_ca = symbolic_cbf_wrapper_multiagent(h_predictive_ca_symbolic, ss, sso)
dhdx_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdx_predictive_ca_symbolic, ss, sso)
dhdtau_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdtau_predictive_ca_symbolic, ss, sso)
d2hdx2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtaudx_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtau2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdtau2_predictive_ca_symbolic, ss, sso)

# Relaxed Predictive Collision Avoidance
# relaxation = 0.05
relaxation = 0.5


# CBF Callables
def h_ca(ego, other):
    return h_nominal_ca(ego, other)


def dhdx_ca(ego, other):
    ret = dhdx_nominal_ca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_ca(ego, other):
    ret = d2hdx2_nominal_ca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


def h_pca(ego, other):
    func = h_predictive_ca(ego, other)

    try:
        ret = func.subs({tau_sym: tau([tau_star(ego, other)])})
    except AttributeError:
        ret = func

    return ret


def dhdx_pca(ego, other):
    func1 = dhdx_predictive_ca(ego, other)
    func2 = dhdtau_predictive_ca(ego, other)

    try:
        ret1 = func1.subs({tau_sym: tau([tau_star(ego, other)])})
    except AttributeError:
        ret1 = func1

    try:
        ret2 = func2.subs({tau_sym: tau([tau_star(ego, other)])})
    except AttributeError:
        ret2 = func2

    ret = ret1 + ret2 * dtaudtaustar([tau_star(ego, other)]) * dtaustardx(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_pca(ego, other):
    func1 = d2hdx2_predictive_ca(ego, other)
    func2 = dhdtau_predictive_ca(ego, other)

    try:
        ret1 = func1.subs({tau_sym: tau([tau_star(ego, other)])})
    except AttributeError:
        ret1 = func1

    try:
        ret2 = func2.subs({tau_sym: tau([tau_star(ego, other)])})
    except AttributeError:
        ret2 = func2

    d2hdx2_eval = ret1
    dtaustardx_eval = dtaustardx(ego, other)
    dtaudtaustar_eval = dtaudtaustar([tau_star(ego, other)])
    dhdtau_eval = ret2
    d2hdtau2_eval = d2hdtau2_predictive_ca(ego, other)
    d2taudtaustar2_eval = d2taudtaustar2([tau_star(ego, other)])
    d2taustardx2_eval = d2taustardx2(ego, other)
    outer = np.outer(dtaustardx_eval, dtaustardx_eval)

    ret = d2hdx2_eval + dtaudtaustar_eval * d2hdtau2_eval * dtaudtaustar_eval * outer + \
          dhdtau_eval * d2taudtaustar2_eval * outer + dhdtau_eval * d2taustardx2_eval * dtaudtaustar_eval

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rpca(ego, other):
    return relaxation * h_ca(ego, other) + h_pca(ego, other)
    # return relaxation * h_ca(ego, other) ** 2 + h_pca(ego, other)


def dhdx_rpca(ego, other):
    try:
        ret = relaxation * dhdx_ca(ego, other) + dhdx_pca(ego, other)
    except Exception as e:
        print(e)
    # ret = 2 * h_ca(ego, other) * relaxation * dhdx_ca(ego, other) + dhdx_pca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_rpca(ego, other):
    ret = relaxation * d2hdx2_ca(ego, other) + d2hdx2_pca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


if __name__ == '__main__':
    # This is a unit test
    ze = np.array([0.0, 0.0, 0.0, 5.0, 0.0])
    zo = np.array([10.0, 0.0, -np.pi, 5.0, 0.0])

    print(h_ca(ze, zo))
    print(float(h_pca(ze, zo)))
    print(float(h_rpca(ze, zo)))
    print(dhdx_ca(ze, zo))
    print(dhdx_pca(ze, zo))
    print(dhdx_rpca(ze, zo))
    print(d2hdx2_ca(ze, zo))
    print(d2hdx2_pca(ze, zo))
    print(d2hdx2_rpca(ze, zo))
    print('stop')



