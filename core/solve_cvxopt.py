import numpy as np
from cvxopt import matrix, solvers

# Silence the solver output
solvers.options['show_progress'] = False


def solve_qp_cvxopt(Q0,
                    p0,
                    A0=None,
                    b0=None,
                    G0=None,
                    h0=None,
                    level=0) -> dict:
    """Solves a quadratic program using the cvxopt library.

    min 1/2 x^T Q x  +  p^T x
    subject to
    Ax <= b
    Gx = h (optional)

    INPUTS
    ------
    Q: (np.array) nxn array (objective)
    p: (np.array) nx1 array (objective)
    A: (np.array) mxn array (inequalities, optional)
    b: (np.array) mx1 array (inequalities, optional)
    G: (np.array) oxn array (equalities, optional)
    h: (np.array) ox1 array (equalities, optional)

    OUTPUTS
    -------
    data: (dict) object containing:
        sol (np.array) if exists
        status (str)
    """
    # Initialize data
    data = dict()

    # Format objective function using cvxopt matrix form
    n = p0.shape[0]
    Q = matrix(np.array(Q0, dtype=float))
    p = matrix(np.array(p0, dtype=float))

    # Format inequality constraints (if they exist)
    if A0 is not None:
        m = b0.shape[0]
        A = matrix(np.array(A0, dtype=float))
        b = matrix(np.array(b0, dtype=float))
        # if m == 1:
        #     A = matrix(A[:, np.newaxis].T)
        # else:
        #     A = matrix(A)
    else:
        A = None
        b = None

    # Format equality constraints (if they exist)
    if G0 is not None:
        o = h0.shape[0]
        G = matrix(np.array(G0, dtype=float))
        h = matrix(np.array(h0, dtype=float))
        # if o == 1:
        #     G = matrix(G[:, np.newaxis].T)
        # else:
        #     G = matrix(G)
    else:
        G = None
        h = None

    try:
        sol = solvers.qp(Q, p, A, b, G, h)
        data['x'] = sol['x']
        data['code'] = 1
        data['status'] = sol['status']

        if sol['status'] != 'optimal':
            check_x = np.array(sol['x']).flatten()
            check_A = np.array(A)
            check_b = np.array(b)
            if np.sum((check_A @ check_x) <= check_b) < len(check_b):
                data['code'] = 0
                data['status'] = 'violates_constraints'

    except ValueError as e:  # Catch infeasibility
        if level == 0:
            return solve_qp_cvxopt(Q0, p0, A0 * 1e3, b0 * 1e3, G0, h0, level + 1)

        print(e)
        data['x'] = np.zeros((n, 1))
        data['code'] = 0
        data['status'] = 'infeasible_or_unbounded'
    except ZeroDivisionError as e:
        data['x'] = np.zeros((n, 1))
        data['code'] = 0
        data['status'] = 'divide_by_zero'
    except Exception as e:
        print(e)
        data['x'] = np.zeros((n, 1))
        data['code'] = 0
        data['status'] = 'some_other_error'
    finally:
        return data


def solve_convex_nonlinear_problem(F, G0=None, h0=None, A0=None, b0=None) -> dict:
    """Solves a convex nonlinear programming problem (using cvxopt) of the form

    min f0(x)
    subject to
    fk(x) <= 0
    Gx <= h (optional)
    Ax == b (optional)

    INPUTS
    ------
    F: function with specific requirements to manage f0, fk: (http://cvxopt.org/userguide/solvers.html)
    G0: (np.array) mxn array (inequalities, optional)
    h0: (np.array) mx1 array (inequalities, optional)
    A0: (np.array) oxn array (equalities, optional)
    b0: (np.array) ox1 array (equalities, optional)

    OUTPUTS
    -------
    data: (dict) object containing:
        sol (np.array) if exists
        status (str)
        code (int)
    """
    # Initialize data
    data = dict()

    # Format inequality constraints (if they exist)
    if G0 is not None:
        m = h0.shape[0]
        G = matrix(np.array(G0, dtype=float))
        h = matrix(np.array(h0, dtype=float))
    else:
        G = None
        h = None

    # Format equality constraints (if they exist)
    if A0 is not None:
        o = b0.shape[0]
        A = matrix(np.array(A0, dtype=float))
        b = matrix(np.array(b0, dtype=float))
    else:
        A = None
        b = None

    try:
        sol = solvers.cp(F, G=G, h=h, A=A, b=b)
        data['x'] = sol['x']
        data['code'] = 1
        data['status'] = sol['status']
    except ValueError as e:  # Catch infeasibility
        print(e)
        data['x'] = np.zeros((n, 1))
        data['code'] = 0
        data['status'] = 'infeasible_or_unbounded'
    except Exception as e:
        print(e)
    finally:
        return data


if __name__ == "__main__":
    # # QP Example
    # Q = 2 * np.array([[2, 1], [1, 1]])
    # p = np.array([1, 1])
    #
    # # Inequality constraints
    # A = np.array([[-1.0, 0.0], [0.0, 1.0]])
    # b = np.array([1.0, -2.0])
    #
    # # # Conflicting inequality constraints
    # # A = np.array([[-1.0, 0.0], [1.0, 0.0]])
    # # b = np.array([1.0, -2.0])
    #
    # # Equality constraints
    # G = np.array([1.0, 1.0])
    # h = np.array([1.0])
    #
    # sol = solve_qp_cvxopt(Q, p, A, b, G, h)
    # if sol['code']:
    #     print(sol['x'])
    # else:
    #     print(sol['status'])

    from cvxopt import solvers, matrix, spdiag, log


    def acent(A, b):
        m, n = A.size

        def F(x=None, z=None):
            if x is None:
                return 0, matrix(1.0, (n, 1))

            if min(x) <= 0.0:
                return None

            f = -sum(log(x))
            Df = -(x ** -1).T

            if z is None:
                return f, Df

            H = spdiag(z[0] * x ** -2)

            return f, Df, H

        return solvers.cp(F, A=A, b=b)['x']

    def qcqp(Q, p, G, h):
        m, n = G.size

        def F(x=None, z=None):

            if x is None:
                return 1, matrix(1.0, (n, 1))

            f0 = 1/2 * x.T * Q * x + p.T * x
            Df0 = x.T * Q + p.T

            def cbf(tt):
                print("tt: {}".format(tt))
                print("Sum: {:.2f}".format(sum(tt**2 + 2*tt) + 3))
                return sum(tt**2 + 2*tt) + 3

            def dcbf(tt):
                print("dcbf: {}".format((2 * tt + 2).T))
                return (2 * tt + 2).T

            x0 = matrix(np.array([1.0, 0.0])).T * x
            x1 = matrix(np.array([0.0, 1.0])).T * x
            f1 = cbf(x)
            Df1 = matrix(dcbf(x))
            # Df1 = matrix(np.array([1.0, 1.0]))

            f = matrix([f0, f1])
            Df = matrix([Df0, Df1])

            if z is None:
                return f, Df

            H = spdiag(matrix(np.diag(z[0] * Q)))

            return f, Df, H

        return solvers.cp(F, G=G, h=h)

    Q = matrix(np.eye(2))
    p = matrix(np.array([1.0, 2.0]))
    G = matrix(np.eye(2))
    h = matrix(np.array([2.0, 1.0]))

    print(qcqp(Q, p, G, h))

