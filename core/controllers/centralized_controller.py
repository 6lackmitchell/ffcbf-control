from typing import List
from nptyping import NDArray
from .controller import Controller


class CentralizedController(Controller):

    def __init__(self,
                 agents: List,
                 u_max):
        super().__init__()
        self.agents = agents
        self.nu = len(u_max)

    def compute_control(self,
                        t: float,
                        z: NDArray,
                        cascade: bool = True) -> (int, str):
        """Computes the control input for the vehicle in question.

        INPUTS
        ------
        t: time (in sec)
        z: full state vector
        """
        u_array, code, status = self.agents[0].controller._compute_control(t, z)

        for aa, agent in enumerate(self.agents):
            agent.controller.u = u_array[self.nu * aa:self.nu * (aa + 1)]
            agent.u = agent.controller.u

        return code, status

