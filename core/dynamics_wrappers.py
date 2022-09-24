import sympy as sp
import numpy as np
from nptyping import NDArray

# global vars for stochastic systems
global DW, T_LAST
DW = None
T_LAST = None


def dyn_wrapper(d_sym: sp.Matrix,
                state_sym: sp.Matrix):
    """ Wrapper for generic symbolic and numerical component to system dynamics. """

    def ret(z: NDArray,
            is_symbolic: bool = False) -> sp.Matrix:
        """ Drift dynamics for bicycle model:

        INPUTS
        ------
        z: state vector
        is_symbolic: boolean specifying whether to return symbolic Matrix (default is False)

        OUTPUTS
        -------
        f(z): sp.Matrix

        """
        if is_symbolic:
            return d_sym

        else:
            # return np.squeeze(np.array(d_sym.subs([(sym, zz) for sym, zz in zip(state_sym, z)])).astype(np.float64))
            return np.squeeze(np.array(d_sym.subs({sym:zz for sym, zz in zip(state_sym, z)})).astype(np.float32))

    return ret


def control_affine_system_deterministic(f, g):
    """Wrapper for full system dynamics. Use to define individual system dynamics as follows:

    system_dynamics = control_affine_system_deterministic(f, g)

    Then evaluate zdot using: zdot = system_dynamics(t, z, u).

    """

    def system(t: float,
               z: NDArray,
               u: NDArray,
               **kwargs: dict) -> NDArray:
        """ Dynamical model for deterministic control-affine system of the form

        zdot = f(z) + g(z)u

        INPUTS
        ------
        t: time (in sec)
        z: state vector
        u: control input vector

        OPTIONAL INPUT KEYS
        ---------------
        theta: true affine system parameters influencing dynamics

        OUTPUTS
        -------
        zdot: np.ndarray

        """

        zdot = f(z) + g(z) @ u

        if 'theta' in kwargs.keys():
            # Regressor not yet defined -- cross that bridge if/when we come to it
            zdot = zdot + regressor(t,z) @ kwargs['theta']

        return zdot

    return system


def control_affine_system_stochastic(f, g, sigma, dt):
    """Wrapper for full system dynamics. Use to define individual system dynamics as follows:

    system_dynamics = control_affine_system_stochastic(f, g, sigma, dt)

    Then evaluate zdot using: zdot = system_dynamics(t, z, u).

    """

    def system(t: float,
               z: NDArray,
               u: NDArray,
               **kwargs: dict) -> NDArray:
        """ Dynamical model for deterministic control-affine system of the form

        dz = (f(z) + g(z)u)dt + sigma(z)dw

        INPUTS
        ------
        t: time (in sec)
        z: state vector
        u: control input vector

        OPTIONAL INPUT KEYS
        ---------------
        theta: true affine system parameters influencing dynamics

        OUTPUTS
        -------
        zdot: np.ndarray

        """
        global DW, T_LAST
        if DW is None:
            DW = np.zeros(f(z).shape[0])
        if T_LAST is None:
            T_LAST = 0.0
        else:
            DW = np.array([np.random.normal() for nn in range(f(z).shape[0])])
            T_LAST = t

        # if t != T_LAST:
        #     DW = np.array([np.random.normal() for nn in range(f(z).shape[0])])
        #     T_LAST = t

        zdot = f(z) + g(z) @ u + sigma(z) @ DW / dt

        if 'theta' in kwargs.keys():
            zdot = zdot + regressor(t,z) @ kwargs['theta']

        return zdot

    return system


def first_order_forward_euler(system_dynamics, dt):
    """Uses the current time, state, and control action to advance the state forward in time according
    to the first-order forward Euler discretization. """

    def step(t: float,
             x: NDArray,
             u: NDArray) -> NDArray:
        """

        INPUTS
        ------
        t: time (in sec)
        x: state vector
        u: control input vector

        OUTPUTS
        -------
        updated state vector

        """
        return x + dt * system_dynamics(t, x, u)

    return step
