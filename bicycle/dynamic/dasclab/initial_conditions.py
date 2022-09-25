import numpy as np


rover1_goal = [-2.00, 0.00]
rover2_goal = [-1.00, -1.00]
rover3_goal = [1.76, 0.86]
rover5_goal = [-1.708, 0.885]
rover7_goal = [2.33, -1.22]

xg = np.array([rover1_goal[0], rover2_goal[0], rover3_goal[0], rover5_goal[0], rover7_goal[0]])
yg = np.array([rover1_goal[1], rover2_goal[1], rover3_goal[1], rover5_goal[1], rover7_goal[1]])

experimenting = False
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
