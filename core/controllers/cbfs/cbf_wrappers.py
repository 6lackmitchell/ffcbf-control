import sympy as sp
import symengine as se
import numpy as np
from nptyping import NDArray


# These all need to be wrappers as well, so that the function call is just the evaluation/substitution
def symbolic_cbf_wrapper_singleagent(cbf_symbolic, *args):
    """Wrapper for symbolic CBFs and derivatives. """

    def cbf(z: NDArray,
            is_symbolic: bool = False):
        """Single agent CBF formulation -- can be CBF or Nth partial derivatives. """
        if is_symbolic:
            return cbf_symbolic

        else:
            ss = np.concatenate(args)
            ret = cbf_symbolic.subs({s:zz for s, zz in zip(ss, z)})

            if type(ret) is not se.Add and type(ret) is not se.MutableDenseMatrix:
                if type(ret) is se.RealDouble or type(ret) is se.Mul:
                    return float(ret)
                elif len(ret) == 1:
                    return float(ret)
                else:
                    return np.squeeze(np.array(ret).astype(np.float32))
            else:
                return ret

    return cbf


def symbolic_cbf_wrapper_multiagent(cbf_symbolic, *args):
    """Wrapper for symbolic CBFs and derivatives. """

    def cbf(ze: NDArray,
            zo: NDArray,
            is_symbolic: bool = False):
        """Multi-agent CBF formulation -- can be CBF or Nth partial derivatives. """
        if is_symbolic:
            return cbf_symbolic

        else:
            ss = np.concatenate(args)
            zz = np.concatenate([ze, zo])
            ret = cbf_symbolic.subs({s:z for s, z in zip(ss, zz)})

            if type(ret) is not se.Add and type(ret) is not se.MutableDenseMatrix:
                if type(ret) is se.RealDouble:
                    return float(ret)
                elif len(ret) == 1:
                    return float(ret)
                else:
                    return np.squeeze(np.array(ret).astype(np.float32))
            else:
                return ret

    return cbf
