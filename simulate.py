""" simulate.py: simulation entry-point for testing controllers for specific dynamical models. """

import numpy as np
import builtins


def simulate(tf: float,
             dt: float,
             vehicle: str,
             level: str,
             situation: str) -> bool:
    """Simulates the system specified by config.py for tf seconds at a frequency of dt.

    ARGUMENTS
        tf: final time (in sec)
        dt: timestep length (in sec)
        vehicle: the vehicle to be simulated
        level: the control level (i.e. kinematic, dynamic, etc.)
        situation: i.e. intersection_old, intersection, etc.

    RETURNS
        None

    """
    broken = False
    # Program-wide specifications
    builtins.PROBLEM_CONFIG = {'vehicle': vehicle,
                               'control_level': level,
                               'situation': situation,
                               'system_model': 'deterministic'}

    if vehicle == 'bicycle':
        from bicycle import nAgents, nStates, z0, centralized_agents, decentralized_agents

    nTimesteps = int((tf - 0.0) / dt) + 1

    # Simulation setup
    z = np.zeros((nTimesteps, nAgents, nStates))
    z[0, :, :] = z0
    complete = np.zeros((nAgents,))
    complete[3:] = 1
    # centralized_agents = deepcopy(centralized_agents_list)
    # decentralized_agents = deepcopy(decentralized_agents_list)

    # Simulate program
    for ii, tt in enumerate(np.linspace(0, tf, nTimesteps - 1)):
        code = 0
        if round(tt, 4) % 1 < dt:
            print("Time: {:.1f} sec".format(tt))

        if round(tt, 4) % 5 < dt and tt > 0:
            for aa, agent in enumerate(decentralized_agents):
                agent.save_data(aa)
            print("Time: {:.1f} sec: Intermediate Save".format(tt))

        # Compute inputs for centralized agents
        if centralized_agents is not None:
            centralized_agents.compute_control(tt, z[ii])

        # Iterate over all agents in the system
        for aa, agent in enumerate(decentralized_agents):
            code, status = agent.compute_control(z[ii])

            if not code:
                broken = True
                print('Error in Agent {}'.format(aa + 1))
                break
            if hasattr(agent, 'complete'):
                if agent.complete and not complete[aa]:
                    complete[aa] = True
                    print("Agent {} Completed!".format(aa))

            # Step dynamics forward
            z[ii + 1, aa, :] = agent.step_dynamics()

        if not code:
            broken = True
            break

        if np.sum(complete) == nAgents:
            break

    # Save data
    for aa, agent in enumerate(decentralized_agents):
        agent.save_data(aa)

    success = not broken

    return success
