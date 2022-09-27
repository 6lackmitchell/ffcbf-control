import numpy as np

# Lab Params
n_goal_end = [-0.08,  3.14]
s_goal_end = [-0.87, -1.88]
e_goal_end = [ 2.47, -0.18]
w_goal_end = [-3.34,  0.45]
n_goal_beg = [-0.80,  0.39]
s_goal_beg = [-0.13, -0.11]
e_goal_beg = [-0.18,  0.53]
w_goal_beg = [-0.66, -0.18]

xg = np.array([e_goal_end[0], s_goal_end[0], n_goal_end[0], w_goal_end[0], w_goal_end[0] + 1.5])
yg = np.array([e_goal_end[1], s_goal_end[1], n_goal_end[1], w_goal_end[1], w_goal_end[1]])




# rover2_goal = [0.71, -1.77]
# rover3_goal = [-2.32, -1.76]
# rover5_goal = [2.14, 0.53]
# rover6_goal = [-1.51, 1.29]
# rover7_goal = [1.01, 3.30]

# xg = np.array([rover2_goal[0], rover3_goal[0], rover5_goal[0], rover6_goal[0], rover7_goal[0]])
# yg = np.array([rover2_goal[1], rover3_goal[1], rover5_goal[1], rover6_goal[1], rover7_goal[1]])

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
