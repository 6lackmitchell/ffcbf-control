""" simulate.py: simulation entry-point for testing controllers for specific dynamical models. """

### Import Statements ###
# General imports
import numpy as np
import builtins
from pickle import dump
from copy import deepcopy
from traceback import print_exc
from simdycosys.config import *

# Make problem config available to other modules
builtins.PROBLEM_CONFIG = {'vehicle': vehicle,
                           'control_level': control_level,
                           'situation': situation,
                           'system_model': system_model}

if vehicle == 'bicycle':
    from simdycosys.bicycle import *
elif vehicle == 'integrator':
    from simdycosys.integrator import *
elif vehicle == 'quadrotor':
    from simdycosys.quadrotor import *

# Number of Trials
nTrials = 200

# Output file
filepath = save_path
filename = filepath + 'merging_monte_carlo.pkl'

# Logging variables
z = np.zeros((nTrials, nTimesteps, nAgents, nStates))
unsafe = np.zeros((nTrials, nAgents))
merged = np.zeros((nTrials, nAgents))

# Copy agents
agents = deepcopy(agents_list)

try:
    for nt in range(nTrials):
        z[nt, 0, :, :] = z0
        safety_violation = np.zeros((nAgents,))
        merge_status = np.zeros((nAgents,))

        for aa, agent in enumerate(agents):
            agent.reset(z[nt, 0, aa, :])

        for ii, tt in enumerate(np.linspace(ts, tf, nTimesteps - 1)):

            # Iterate over all agents in the system
            for aa, agent in enumerate(agents):
                code, status = agent.compute_control(z[nt, ii])

                # Step dynamics forward
                z[nt, ii + 1, aa, :] = agent.step_dynamics()

                # Check Safety
                if aa < 2:
                    safety_violation[aa] = int(not agent.controller.safety) + safety_violation[aa]
                    safety_violation[aa] = np.min([safety_violation[aa], 1])  # Keep unsafe as recording boolean yes/no values

                # Check Merging status -- only care about intersection vehicles
                merge_status[aa] = abs(z[nt, ii + 1, aa, 1]) < 0.25

        unsafe[nt, :] = safety_violation
        merged[nt, :] = merge_status

        if nt % int(nTrials / np.min([10, nTrials])) == 0:
            print("{} / {} Complete".format(nt + 1, nTrials))

except Exception as e:
    print_exc()

else:
    pass

finally:
    print("Merge Fraction: {:.2f}".format(np.sum(merged, 0)[0] / nTrials))
    print("Unsafe Fraction: {:.2f}".format(np.sum(unsafe, 0)[0] / nTrials))

    # Save all data
    # Write data to file
    data = {'x': z,
            'merged': merged,
            'unsafe': unsafe}
    with open(filename, 'wb') as f:
        dump(data, f)

    # for aa, agent in enumerate(agents):
    #     agent.save_data(aa, filename)

print('\a')

# class Model:
#     """An abstract representation of a dynamical system.
#     """
#
#     def __init__(self, model_type: ModelType):
#         self._type = model_type
#
# def simulate(model: Model)