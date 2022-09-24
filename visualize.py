# Determine which problem is to be simulated
import builtins
import importlib

vehicle = 'bicycle'
level = 'dynamic'
situation = 'warehouse'

# Make problem config available to other modules
builtins.PROBLEM_CONFIG = {'vehicle': vehicle,
                           'control_level': level,
                           'situation': situation,
                           'system_model': 'deterministic'}
mod = '{}.{}.{}.vis_paper'.format(vehicle, level, situation)

# Problem-specific import
try:
    module = importlib.import_module(mod)
    # module = importlib.import_module(config + '.vis_mc')
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e
