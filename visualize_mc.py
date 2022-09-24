# Determine which problem is to be simulated
import builtins
import importlib
from simdycosys.config import *

# Make problem config available to other modules
builtins.PROBLEM_CONFIG = {'vehicle': vehicle,
                           'control_level': control_level,
                           'situation': situation,
                           'system_model': system_model}
mod = 'simdycosys.{}.{}.{}.vis_mc'.format(vehicle, control_level, situation)

# Problem-specific import
try:
    module = importlib.import_module(mod)
except ModuleNotFoundError as e:
    print('No module named \'{}\' -- exiting.'.format(mod))
    raise e
