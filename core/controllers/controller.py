from nptyping import NDArray


class Controller:

    ego_id = None
    nu = None
    u = None
    u_nom = None
    u_max = None
    nominal_controller = None
    _compute_control = None
    _compute_cascaded_control = None

    def __init__(self):
        self.safety = True

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
        return self._compute_control(t, z)

