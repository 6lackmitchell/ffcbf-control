import symengine as se


def ramp(x: float,
         k: float,
         d: float) -> se.Function:
    """Returns a function for evaluating the sigmoid function of the form
    1/2 * (1 + tanh(k*(x - d)))

    INPUTS
    ------
    x: argument to tanh function
    k: scale factor
    d: offset factor

    OUTPUTS
    -------
    function

    """
    return 0.5 * (1 + se.tanh(k * (x - d)))
