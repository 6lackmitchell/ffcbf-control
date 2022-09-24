import numpy as np
from typing import Callable, List
from nptyping import NDArray
from pathlib import Path
from pickle import dump, load
from .controllers.controller import Controller


class Agent:

    t = 0.0

    @property
    def timestep(self):
        assert(type(self._timestep) == int)
        return self._timestep

    @timestep.setter
    def timestep(self, val):
        if type(val) != int:
            raise ValueError("Timestep must be an integer.")
        self._timestep = val

    @property
    def complete(self):
        return self.controller.nominal_controller.complete

    def __init__(self,
                 identifier: int,
                 u0: NDArray,
                 cbf0: NDArray,
                 timing: List,
                 dynamics: Callable,
                 controller: Controller,
                 save_file: str):
        """ Class initializer. """
        self.id = identifier
        self.u = u0
        self.u_nom = u0
        self.cbf = cbf0
        self.dynamics = dynamics
        self.controller = controller
        self.save_file = save_file

        # Give identifier to controller
        self.controller.ego_id = self.id

        # Extract timing data
        self.dt = timing[0]
        self.tf = timing[1]
        self.nTimesteps = int((self.tf - self.t) / self.dt) + 1
        self.controller._dt = self.dt

        # Additional class variables
        self.x = None
        self.data = None
        self.x_trajectory = None
        self.u_trajectory = None
        self.u0_trajectory = None
        self.cbf_trajectory = None
        self.consolidated_cbf_trajectory = None
        self.k_gains_trajectory = None
        self.safety = None
        self._timestep = None

        self.reset(np.zeros(5,))

    def reset(self,
              x0: NDArray) -> None:
        """Resets run variables for new trial."""
        self.x = x0
        self.t = 0.0
        self.timestep = 0
        self.x_trajectory = np.zeros((self.nTimesteps, x0.shape[0]))
        self.u_trajectory = np.zeros((self.nTimesteps, self.u_nom.shape[0]))
        self.u0_trajectory = np.zeros((self.nTimesteps, self.u_nom.shape[0]))
        self.safety = np.zeros((self.nTimesteps,))

        if hasattr(self.controller, 'cbf_vals'):
            self.cbf_trajectory = np.zeros((self.nTimesteps, len(self.controller.cbf_vals)))
            self.consolidated_cbf_trajectory = np.zeros((self.nTimesteps, ))
            self.k_gains_trajectory = np.zeros((self.nTimesteps, len(self.controller.cbf_vals)))
        else:
            self.cbf_trajectory = np.zeros((self.nTimesteps, ))
            self.consolidated_cbf_trajectory = np.zeros((self.nTimesteps,))
            self.k_gains_trajectory = np.zeros((self.nTimesteps,))

        # Save data object -- auto-updating since defined by reference
        self.data = {'x': self.x_trajectory,
                     'u': self.u_trajectory,
                     'u0': self.u0_trajectory,
                     'cbf': self.cbf_trajectory,
                     'ccbf': self.consolidated_cbf_trajectory,
                     'kgains': self.k_gains_trajectory,
                     'ii': self.t}

    def compute_control(self,
                        full_state: NDArray) -> (int, str):
        """ Computes the control input for the Agent.
        INPUTS
        ------
        full_state: full state vector for all agents
        OUTPUTS
        -------
        code: success / error flag
        status: more informative success / error information
        """
        misc = None

        self.u, code, status = self.controller.compute_control(self.t, full_state)

        # Update Control and CBF Trajectories
        self.u_trajectory[self.timestep, :] = self.controller.u
        self.u0_trajectory[self.timestep, :] = self.controller.u_nom
        self.data['ii'] = self.t
        if hasattr(self.controller, 'cbf_vals'):
            self.cbf_trajectory[self.timestep, :] = self.controller.cbf_vals
        if hasattr(self.controller, 'c_cbf'):
            self.consolidated_cbf_trajectory[self.timestep] = self.controller.c_cbf
        if hasattr(self.controller, 'k_gains'):
            self.k_gains_trajectory[self.timestep, :] = self.controller.k_gains

        if misc is not None:
            print(misc)

        return code, status

    def step_dynamics(self) -> NDArray:
        """ Advances the Agent's dynamics forward in time using the current state and control input.
        INPUTS
        ------
        None -- Requires only class variables
        OUTPUTS
        -------
        x_updated: new state vector
        x_updated: new state vector
        """
        x_updated = self.dynamics(self.t, self.x, self.u)
        self.update(x_updated)

        return x_updated

    def update(self,
               x_new: NDArray) -> None:
        """ Updates the class time and state vector.
        INPUTS
        ------
        x_new: new state vector
        OUTPUTS
        -------
        None
        """
        self.x = x_new
        self.x_trajectory[self.timestep, :] = self.x
        self.timestep = self.timestep + 1
        self.t = self.timestep * self.dt

    def save_data(self,
                  identity: int) -> None:
        """Saves the agent's individual simulation data out to a .pkl file.
        INPUTS
        ------
        identity: agent identifier
        filename: name of file to which data is saved
        OUTPUTS
        -------
        None
        """
        file = Path(self.save_file)
        if file.is_file():
            # Load data, then add to it
            with open(self.save_file, 'rb') as f:
                try:
                    data = load(f)
                    data[identity] = self.data
                except EOFError:
                    data = {identity: self.data}

        else:
            data = {identity: self.data}

        # Write data to file
        with open(self.save_file, 'wb') as f:
            dump(data, f)
