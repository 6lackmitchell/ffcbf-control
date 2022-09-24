import builtins
import numpy as np
import symengine as se
from importlib import import_module
from simdycosys.core.cbfs.cbf_wrappers import symbolic_cbf_wrapper_multiagent, symbolic_cbf_wrapper_singleagent
from simdycosys.core.mathematics.symbolic_functions import ramp

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
mod = 'simdycosys.' + vehicle + '.' + control_level + '.models'

# Programmatic import
try:
    module = import_module(mod)
    globals().update({'f': getattr(module, 'f')})
    globals().update({'ss': getattr(module, 'sym_state')})
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e

# Defining Physical Params
D = 2.5
th = 15.0 * np.pi / 180.0

# On-Ramp BF
h0_or_symbolic = D**2 - (ss[0] * np.cos(th) - ss[1] * np.sin(th))**2 + (ss[0] * np.sin(th) + ss[1] * np.cos(th))**2
dh0dx_or_symbolic = se.DenseMatrix([h0_or_symbolic]).jacobian(se.DenseMatrix(ss))
d2h0dx2_or_symbolic = dh0dx_or_symbolic.jacobian(se.DenseMatrix(ss))
d3h0dx3_or_symbolic = d2h0dx2_or_symbolic.jacobian(se.DenseMatrix(ss))
h0_or = symbolic_cbf_wrapper_singleagent(h0_or_symbolic, ss)
dh0dx_or = symbolic_cbf_wrapper_singleagent(dh0dx_or_symbolic, ss)
d2h0dx2_or = symbolic_cbf_wrapper_singleagent(d2h0dx2_or_symbolic, ss)
d3h0dx3_or = symbolic_cbf_wrapper_singleagent(d3h0dx3_or_symbolic, ss)

# On-Ramp High-Order Risk-Bounded CBF




# Collision Avoidance CBF
h_ca_symbolic = (ss[0] - sso[0])**2 + (ss[1] - sso[1])**2 - (2 * R)**2
dhdx_ca_symbolic = se.DenseMatrix([h_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))
d2hdx2_ca_symbolic = dhdx_ca_symbolic.jacobian(se.DenseMatrix(ss + sso))

# # Autowrap symbolic functions
# h_ca_autowrap = autowrap(h_ca_symbolic, backend="cython")

h_ca = symbolic_cbf_wrapper_multiagent(h_ca_symbolic, ss, sso)
dhdx_ca = symbolic_cbf_wrapper_multiagent(dhdx_ca_symbolic, ss, sso)
d2hdx2_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_ca_symbolic, ss, sso)

# Tau Formulation for PCA-CBF
vxe = f(np.zeros((len(ss),)), True)[0]
vye = f(np.zeros((len(ss),)), True)[1]
vxo = vxe
vyo = vye
for sego, sother in zip(ss, sso):
    vxo = vxo.replace(sego, sother)
    vyo = vyo.replace(sego, sother)
dvx = vxe - vxo
dvy = vye - vyo

tau_sym = se.Symbol('tau', real=True)

# tau* for computing tau
epsilon = 1e-3
tau_star_symbolic = -(dx * dvx + dy * dvy) / (dvx ** 2 + dvy ** 2 + epsilon)
dtaustardx_symbolic = se.DenseMatrix([tau_star_symbolic]).jacobian(se.DenseMatrix(ss + sso))
d2taustardx2_symbolic = dtaustardx_symbolic.jacobian(se.DenseMatrix(ss + sso))
tau_star = symbolic_cbf_wrapper_multiagent(tau_star_symbolic, ss, sso)
dtaustardx = symbolic_cbf_wrapper_multiagent(dtaustardx_symbolic, ss, sso)
d2taustardx2 = symbolic_cbf_wrapper_multiagent(d2taustardx2_symbolic, ss, sso)

# tau for computing PCA-CBF
Tmax = 3.0
kh = 1000.0
tau_star_sym = se.Symbol('tau_star', real=True)
tau_symbolic = tau_star_sym * ramp(tau_star_sym, kh, 0.0) - (tau_star_sym - Tmax) * ramp(tau_star_sym, kh, Tmax)
dtaudtaustar_symbolic = se.diff(tau_symbolic, tau_star_sym)
d2taudtaustar2_symbolic = se.diff(dtaudtaustar_symbolic, tau_star_sym)
tau = symbolic_cbf_wrapper_singleagent(tau_symbolic, [tau_star_sym])
dtaudtaustar = symbolic_cbf_wrapper_singleagent(dtaudtaustar_symbolic, [tau_star_sym])
d2taudtaustar2 = symbolic_cbf_wrapper_singleagent(d2taudtaustar2_symbolic, [tau_star_sym])

# Predictive Collision Avoidance CBF
h_ca_symbolic = (ss[0] - sso[0])**2 + (ss[1] - sso[1])**2 - (2 * R)**2
dhdx_ca_symbolic = se.DenseMatrix([h_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))
d2hdx2_ca_symbolic = dhdx_ca_symbolic.jacobian(se.DenseMatrix(ss + sso))

h_predictive_ca_symbolic = (dx + tau_sym * dvx)**2 + (dy + tau_sym * dvy)**2 - (2 * R)**2
dhdx_predictive_ca_symbolic = se.DenseMatrix([h_predictive_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))
dhdtau_predictive_ca_symbolic = se.diff(h_predictive_ca_symbolic, tau_sym)
d2hdx2_predictive_ca_symbolic = dhdx_predictive_ca_symbolic.jacobian(se.DenseMatrix(ss + sso))
d2hdtau2_predictive_ca_symbolic = se.diff(dhdtau_predictive_ca_symbolic, tau_sym)
d2hdtaudx_predictive_ca_symbolic = se.DenseMatrix([dhdtau_predictive_ca_symbolic]).jacobian(se.DenseMatrix(ss + sso))
h_predictive_ca = symbolic_cbf_wrapper_multiagent(h_predictive_ca_symbolic, ss, sso)
dhdx_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdx_predictive_ca_symbolic, ss, sso)
dhdtau_predictive_ca = symbolic_cbf_wrapper_multiagent(dhdtau_predictive_ca_symbolic, ss, sso)
d2hdx2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtaudx_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdx2_predictive_ca_symbolic, ss, sso)
d2hdtau2_predictive_ca = symbolic_cbf_wrapper_multiagent(d2hdtau2_predictive_ca_symbolic, ss, sso)

# Relaxed Predictive Collision Avoidance
# relaxation = 1.0  # produces wild avoid (rho = 0.25)
# relaxation = 1.0
relaxation = 0.1


def h_pca(ego, other):
    return h_predictive_ca(ego, other).subs({tau_sym: tau([tau_star(ego, other)])})


def dhdx_pca(ego, other):
    ret = dhdx_predictive_ca(ego, other).subs({tau_sym: tau([tau_star(ego, other)])}) + \
          dhdtau_predictive_ca(ego, other).subs({tau_sym: tau([tau_star(ego, other)])}) * \
          dtaudtaustar([tau_star(ego, other)]) * dtaustardx(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_pca(ego, other):
    d2hdx2_eval = d2hdx2_predictive_ca(ego, other).subs({tau_sym: tau([tau_star(ego, other)])})
    dtaustardx_eval = dtaustardx(ego, other)
    dtaudtaustar_eval = dtaudtaustar([tau_star(ego, other)])
    dhdtau_eval = dhdtau_predictive_ca(ego, other).subs({tau_sym: tau([tau_star(ego, other)])})
    d2hdtau2_eval = d2hdtau2_predictive_ca(ego, other)
    d2taudtaustar2_eval = d2taudtaustar2([tau_star(ego, other)])
    d2taustardx2_eval = d2taustardx2(ego, other)
    outer = np.outer(dtaustardx_eval, dtaustardx_eval)

    ret = d2hdx2_eval + dtaudtaustar_eval * d2hdtau2_eval * dtaudtaustar_eval * outer + \
          dhdtau_eval * d2taudtaustar2_eval * outer + dhdtau_eval * d2taustardx2_eval * dtaudtaustar_eval

    return np.squeeze(np.array(ret).astype(np.float64))


def h_rpca(ego, other):
    return relaxation * h_ca(ego, other) + h_pca(ego, other)


def dhdx_rpca(ego, other):
    ret = relaxation * dhdx_ca(ego, other) + dhdx_pca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))


# Necessary for stochastic systems
def d2hdx2_rpca(ego, other):
    ret = relaxation * d2hdx2_ca(ego, other) + d2hdx2_pca(ego, other)

    return np.squeeze(np.array(ret).astype(np.float64))

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



