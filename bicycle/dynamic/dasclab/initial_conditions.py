import numpy as np

rover3_goal = [1.76, 0.86]
rover5_goal = [-1.708, 0.885]
rover7_goal = [2.33, -1.22]

xg = np.array([rover3_goal[0], rover5_goal[0], rover7_goal[0]])
yg = np.array([rover3_goal[1], rover5_goal[1], rover7_goal[1]])

z0 = np.zeros((5,))
u0 = np.zeros((2,))
nAgents = 3
nStates = z0.shape[0]
