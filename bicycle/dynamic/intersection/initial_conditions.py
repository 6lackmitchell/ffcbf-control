import numpy as np


nAgents = 25
x_dist = 1.0
y_dist = 1.0
nRows = int(np.sqrt(nAgents))
box_width = 4.0

xi0 = np.zeros((nAgents,))
yi0 = np.zeros((nAgents,))

for ww, bw in enumerate(range(nRows)):
    for ll, bl in enumerate(range(nRows)):
        xi0[ll * nRows + ww] = ww * box_width + box_width / 2
        yi0[ll * nRows + ww] = ll * box_width + box_width / 2

# Set Goal locations
xi0r = np.copy(xi0)
yi0r = np.copy(yi0)
new_order = np.arange(0, len(xi0r))
np.random.shuffle(new_order)
xg = np.array([xx for xx in xi0r[new_order]])
yg = np.array([yy for yy in yi0r[new_order]])

xi = np.array([xi0[ii] + np.random.uniform(low=-x_dist, high=x_dist) for ii in range(nAgents)])
yi = np.array([yi0[ii] + np.random.uniform(low=-y_dist, high=y_dist) for ii in range(nAgents)])
psii = np.array([np.arctan2(yg[ii] - yi[ii], xg[ii] - xi[ii]) + np.random.uniform(low=-np.pi / 8, high=np.pi / 8) for ii in range(nAgents)])
vi = np.array([0.0 for ii in range(nAgents)])
z0 = np.array([np.array([xi[aa], yi[aa], psii[aa], vi[aa], 0.0]) for aa in range(nAgents)])



u0 = np.array([0.0, 0.0])

nAgents, nStates = z0.shape
