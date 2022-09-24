import numpy as np


def ramp(x: float,
         k: float,
         d: float) -> float:
    """ Approximation to the unit ramp function.

    INPUTS
    ------
    x: independent variable
    k: scale factor
    d: offset factor

    OUTPUTS
    -------
    y = 1/2 + 1/2 * tanh(k * (x - d))

    """
    return 0.5 * (1 + np.tanh(k * (x - d)))


def dramp(x: float,
          k: float,
          d: float) -> float:
    """ Derivative of approximation to the unit ramp function.

    INPUTS
    ------
    x: independent variable
    k: scale factor
    d: offset factor

    OUTPUTS
    -------
    dy = k/2 * (sech(k * (x - d))) ** 2

    """
    return k / (2 * (np.cosh(k * (x - d)))**2)


def d2ramp(x: float,
           k: float,
           d: float) -> float:
    """ 2nd Derivative of approximation to the unit ramp function.

    INPUTS
    ------
    x: independent variable
    k: scale factor
    d: offset factor

    OUTPUTS
    -------
    dy = k/2 * (sech(k * (x - d))) ** 2

    """
    arg = k * (d - x)
    return k**2 * np.tanh(arg) / np.cosh(arg)**2
