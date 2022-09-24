import numpy as np
from nptyping import NDArray


def get_circle(xy: NDArray,
               r: float,
               d_points: int) -> (NDArray, NDArray):
    """ Takes in an xy-position and generates a circle of radius R around that point.

    INPUTS
    ------
    xy: x and y coordinates
    R: radius
    dpoints: number of data points

    OUTPUTS
    -------
    xdata: x-coordinates for circle
    ydata: y-coordinates for circle
    """
    # s = np.linspace(-np.pi, np.pi, d_points)
    s = np.linspace(0, 2 * np.pi, d_points)
    xdata = xy[0] + (r * 1) * np.cos(s)
    ydata = xy[1] + (r * 1) * np.sin(s)

    return xdata, ydata


def get_ex(xy: NDArray,
           r: float,
           d_points: int) -> (NDArray, NDArray):
    """ Takes in an xy-position and generates an X of radius R around that point.

    INPUTS
    ------
    xy: x and y coordinates
    R: radius
    dpoints: number of data points

    OUTPUTS
    -------
    xdata: x-coordinates for circle
    ydata: y-coordinates for circle
    """
    s = np.linspace(0, r / 2, int(d_points / 4))
    xdata = np.vstack([xy[0] + s, xy[0] - s, xy[0] - s, xy[0] + s])
    ydata = np.vstack([xy[1] + s, xy[1] + s, xy[1] - s, xy[1] - s])

    return xdata, ydata
