import builtins
from importlib import import_module
from core.agent import Agent
from core.controllers.cbfs import cbfs_individual, cbfs_pairwise, cbf0
from core.controllers.cbf_qp_controller import CbfQpController
from core.controllers.consolidated_cbf_controller import ConsolidatedCbfController
from .timing_params import *
from .physical_params import u_max
from .models import f, g, nControls
from .objective_functions import objective_accel_and_steering
from .dasclab.nominal_controllers import LqrController

vehicle = builtins.PROBLEM_CONFIG['vehicle']
control_level = builtins.PROBLEM_CONFIG['control_level']
system_model = builtins.PROBLEM_CONFIG['system_model']
situation = builtins.PROBLEM_CONFIG['situation']
mod = vehicle + '.' + control_level + '.' + situation + '.initial_conditions'

# Programmatic version of 'from situation import *'
try:
    module = import_module(mod)
    globals().update(
        {n: getattr(module, n) for n in module.__all__} if hasattr(module, '__all__')
        else {k: v for (k, v) in module.__dict__.items() if not k.startswith('_')}
    )
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e

if system_model == 'stochastic':
    from .models import sigma_stochastic as sigma, \
        stochastic_dynamics as system_dynamics, stochastic_step as step_dynamics
else:
    from .models import sigma_deterministic as sigma, \
        deterministic_dynamics as system_dynamics, deterministic_step as step_dynamics

# Configure parameters
nAgents = len(z0)
time = [dt, tf]
save_path = '/home/dasc/Documents/MB/datastore/warehouse/test.pkl'


# Define controllers


def consolidated_cbf_controller(idx: int) -> ConsolidatedCbfController:
    return ConsolidatedCbfController(
        u_max,
        nAgents,
        objective_accel_and_steering,
        LqrController(idx),
        cbfs_individual,
        cbfs_pairwise
    )


# # Define CBF Controlled Agents
# cbf_controlled_agents = [
#     Agent(i, u0, cbf0, time, step_dynamics, consolidated_cbf_controller(i), save_path) for i in range(3)
# ]

# rover3 = Agent(0, u0, cbf0, time, step_dynamics, consolidated_cbf_controller(0), save_path)
# rover5 = Agent(1, u0, cbf0, time, step_dynamics, consolidated_cbf_controller(1), save_path)
# rover7 = Agent(2, u0, cbf0, time, step_dynamics, consolidated_cbf_controller(2), save_path)

rover3 = Agent(0, u0, cbf0, time, step_dynamics, LqrController(0), save_path)
rover5 = Agent(1, u0, cbf0, time, step_dynamics, LqrController(1), save_path)
rover7 = Agent(2, u0, cbf0, time, step_dynamics, LqrController(2), save_path)

decentralized_agents = [rover3 ,rover5, rover7]