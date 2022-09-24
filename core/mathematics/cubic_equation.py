import math
from typing import Tuple

# The contents of this file were taken from https://gist.github.com/cppio/4ca0324d00240d11ecaa1b1fcc8e7c80
# with slight modifications


def solve_cubic(a: float,
                b: float,
                c: float,
                d: float) -> Tuple[float, ...]:
    """Returns the real solutions to cubic equation of the form

    a * x ** 3 + b * x ** 2 + c * x + d == 0

    INPUTS
    ------
    a: cubic coefficient
    b: quadratic coefficient
    c: linear coefficient
    d: constant coefficient

    OUTPUTS
    -------
    roots: real roots of cubic equation

    """
    return solve_monic_cubic(b / a, c / a, d / a)


def solve_monic_cubic(b: float,
                      c: float,
                      d: float) -> Tuple[float, ...]:
    """Returns the real solutions to monic cubic equation of the form

    x ** 3 + b * x ** 2 + c * x + d == 0

    INPUTS
    ------
    b: quadratic coefficient
    c: linear coefficient
    d: constant coefficient

    OUTPUTS
    -------
    roots: real roots of cubic equation

    """

    p = c - b ** 2 / 3
    q = d - b * c / 3 + b ** 3 * 2 / 27

    return tuple(t - b / 3 for t in solve_depressed_cubic(p, q))


def cube_root(x: float) -> float:
    """Computes the signed cube root of the argument x. """
    return math.copysign(abs(x) ** (1 / 3), x)


def solve_depressed_cubic(p: float,
                          q: float) -> Tuple[float, ...]:
    """Returns the real solutions to x ** 3 + p * x + q == 0"""

    if p == 0:
        return (-cube_root(q), )

    d = p ** 3 / 27 + q ** 2 / 4

    if d > 0:
        r = -q / 2
        s = math.sqrt(d)

        return (cube_root(r + s) + cube_root(r - s), )

    r = 3 * q / p

    if d == 0:
        return r, -r / 2

    s = math.acos(r / 2 * math.sqrt(-3 / p)) / 3
    t = 2 * math.sqrt(-p / 3)
    u = 2 * math.pi / 3

    return tuple(t * math.cos(s - u * k) for k in range(3))
