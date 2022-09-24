import math
import numpy as np
from typing import Tuple


def solve_quadratic(a: float,
                    b: float,
                    c: float) -> Tuple[float, float]:
    """Returns the real solutions to quadratic equation of the form

    a * x ** 2 + b * x + c == 0

    INPUTS
    ------
    a: quadratic coefficient
    b: linear coefficient
    c: constant coefficient

    OUTPUTS
    -------
    roots: real roots of quadratic equation

    """
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return 0, 0
    else:
        root1 = (-b + np.sqrt(discriminant)) / (2 * a)
        root2 = (-b - np.sqrt(discriminant)) / (2 * a)

        return root1, root2
