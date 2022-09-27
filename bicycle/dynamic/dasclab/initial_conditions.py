import numpy as np


rover2_goal = [0.71, -1.77]
rover3_goal = [-2.32, -1.76]
rover5_goal = [2.14, 0.53]
rover6_goal = [-1.51, 1.29]
rover7_goal = [1.01, 3.30]

xg = np.array([rover2_goal[0], rover3_goal[0], rover5_goal[0], rover6_goal[0], rover7_goal[0]])
yg = np.array([rover2_goal[1], rover3_goal[1], rover5_goal[1], rover6_goal[1], rover7_goal[1]])

experimenting = True
if experimenting:
    z0 = np.zeros((5,))
    u0 = np.zeros((2,))
    nAgents = len(xg)
    nStates = z0.shape[0]

else:
    x0 = -xg
    y0 = -yg
    phi0 = np.arctan2(yg, xg)
    v0 = np.zeros((len(xg),))
    beta0 = np.zeros((len(xg),))

    z0 = np.array([[x0[ii], y0[ii], phi0[ii], v0[ii], beta0[ii]] for ii in range(len(xg))])
    u0 = np.zeros((2,))
    nAgents = len(xg)
    nStates = 5

    print("Distances")
    for zz1 in z0:
        for zz2 in z0:
            print("Distance: {}".format(np.linalg.norm(zz1[0:2] - zz2[0:2])))

    print("ok?")
