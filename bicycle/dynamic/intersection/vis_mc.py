import matplotlib
matplotlib.use("Qt5Agg")

import os
import glob
import pickle
import traceback
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
from simdycosys.bicycle.dynamic import nAgents
from simdycosys.bicycle.dynamic.physical_params import LW
from simdycosys.bicycle.dynamic.timing_params import dt, tf
from simdycosys.bicycle.dynamic.merging import *
from simdycosys.visualizing.helpers import get_circle, get_ex

matplotlib.rcParams.update({'figure.autolayout': True})

N = 2 * nAgents
plt.style.use(['Solarize_Light2'])
plt.rcParams["axes.prop_cycle"] = plt.cycler("color", plt.cm.viridis(np.linspace(0, 1, N)))
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
colors[0] = colors[1]
colors.reverse()

filepath = '/home/mblack/Documents/datastore/bicycle/dynamic/intersection/'


# ### Define Recording Variables ###
t = np.linspace(dt, tf, int(tf/dt))

import builtins
if hasattr(builtins, "VIS_FILE"):
    filename = filepath + builtins.VIS_FILE + '.pkl'
else:
    ftype = r'*.pkl'
    files = glob.glob(filepath + ftype)
    files.sort(key=os.path.getmtime)
    filename = files[-1]

with open(filename, 'rb') as f:
    try:
        data = pickle.load(f)

        x = data['x']
        x = np.swapaxes(x[2], 0, 1)
        unsafe = data['unsafe']
        success = data['merged']

        print("RB-CBF Merged: {:.3f}".format(np.sum(success, 0)[0] / success.shape[0]))
        print("S-CBF Merged:  {:.3f}".format(np.sum(success, 0)[1] / success.shape[0]))
        print("RB-CBF Unsafe: {:.3f}".format(np.sum(unsafe, 0)[0] / unsafe.shape[0]))
        print("S-CBF Unsafe:  {:.3f}".format(np.sum(unsafe, 0)[1] / unsafe.shape[0]))
        # (array([7, 8, 18, 32, 38, 51, 61, 64, 70, 85, 87, 90, 93,
        #         103, 117, 136, 139, 152, 154, 155, 156, 159, 164, 174, 180, 187]),)
    except:
        traceback.print_exc()

lwidth = 2
dash = [3, 2]
color_idx = np.array(range(0, 2*nAgents)).reshape(nAgents, 2)

ii = int(tf / dt)
# ii = np.min([int(5.418/dt),ii])

def set_edges_black(ax):
    ax.spines['bottom'].set_color('#000000')
    ax.spines['top'].set_color('#000000')
    ax.spines['right'].set_color('#000000')
    ax.spines['left'].set_color('#000000')

plt.close('all')


############################################
### State Trajectories ###
plt.style.use(['dark_background'])
fig_map = plt.figure(figsize=(13, 2.5))
ax_pos = fig_map.add_subplot(111)
set_edges_black(ax_pos)

# Set Up Road
d_points = 20
start_p = -100.0
end_p = 1000.0
s_th = np.sin(15.0 * np.pi / 180.0)
end_1 = -LW / (s_th)
start_2 = 0.0
ax_pos.plot(np.linspace(start_p, end_1, d_points), -(LW / 2) * np.ones((d_points,)), linewidth=lwidth+1, color='w')
ax_pos.plot(np.linspace(start_2, end_p, d_points), -(LW / 2) * np.ones((d_points,)), linewidth=lwidth+1, color='w')
ax_pos.plot(np.linspace(start_p, end_p, d_points), LW + LW / 2 * np.ones((d_points,)), linewidth=lwidth+1, color='w')
ax_pos.plot(np.linspace(start_p, 0, d_points), np.linspace(start_p, 0, d_points) * s_th - LW / 2, linewidth=lwidth+1, color='w')
ax_pos.plot(np.linspace(start_p, -LW / (s_th), d_points), np.linspace(start_p, -LW / (s_th), d_points) * s_th + LW / 2, linewidth=lwidth+1, color='w')

px = np.tile(np.arange(start_p, end_p, 10), 2)
# py = np.repeat(np.array([-LW / 2, LW / 2]), int(len(px)) / 2)
py = np.repeat(np.array([LW / 2]), int(len(px)) / 2)

# Add rectangles
width = 3
height = 0.25
for ppx, ppy in zip(px, py):
    ax_pos.add_patch(Rectangle(
        xy=(ppx - width / 2, ppy - height / 2), width=width, height=height,
        linewidth=1, color='white', fill=True))

# plt.show()

# Create variable reference to plot
map_vid = []
for aa in range(2 * nAgents):
    if (aa + 10) % 2 == 0:
        map_vid.append(ax_pos.plot([], [], linewidth=lwidth)[0])
    else:
        map_vid.append(ax_pos.plot([], [], linewidth=lwidth, dashes=dash)[0])

# Add text annotation and create variable reference
txt = ax_pos.text(40.0, 6.2, '', ha='right', va='top', fontsize=24)

ax_pos.set(ylim=[-5.0, 6.0],
           xlim=[-2.0, 50.0])
ax_pos.set(ylim=[-50.0, 6.0],
           xlim=[-125.0, 50.0])

# Plot Settings
for item in ([ax_pos.title, ax_pos.xaxis.label, ax_pos.yaxis.label] +
             ax_pos.get_xticklabels() + ax_pos.get_yticklabels()):
    item.set_fontsize(25)
# Hide X and Y axes label marks
ax_pos.xaxis.set_tick_params(labelbottom=False)
ax_pos.yaxis.set_tick_params(labelleft=False)
# Hide X and Y axes tick marks
ax_pos.set_xticks([])
ax_pos.set_yticks([])
ax_pos.legend(fancybox=True)
ax_pos.grid(False)


# Animation function -- Full view
def animate(jj):
    last_1_sec = 20
    right_vehicle = np.argmax(x[:, jj, 0])
    left_edges = np.array([x[np.argmin(x[:, np.max([0, idx-10]):idx+1, 0], 0)[-1], idx, 0] for idx in range(jj+1)]) - 2.0 - (jj / 30)
    zero_point = left_edges[-1]
    # zero_point = -125.0  # left_edges[-1]
    zero_point_y = np.min(x[:, jj, 1])
    end_point = np.max([np.max(x[right_vehicle, jj, 0]) + 2.0, zero_point + 150.0])

    for aa in range(0, 2 * nAgents, 2):
        idx = int(aa / 2)
        if idx < 2:
            x_circ, y_circ = get_ex(x[idx, jj], 1.0, d_points)
        else:
            x_circ, y_circ = get_circle(x[idx, jj], 1.0, d_points)
        x_hist, y_hist = x[idx, np.max([0, jj+1 - last_1_sec]):jj+1, 0:2].T
        map_vid[aa].set_data(x_circ, y_circ)
        # map_vid[aa + 1].set_data(x_hist - left_edges + zero_point, y_hist)
        map_vid[aa + 1].set_data(x_hist, y_hist)
        if idx <= 2:
            map_vid[aa].set_color(colors[color_idx[idx, 1]])
            map_vid[aa + 1].set_color(colors[color_idx[idx, 1]])
        else:
            map_vid[aa].set_color('r')
            map_vid[aa + 1].set_color('r')

    ax_pos.set_xlim([zero_point, end_point])
    ax_pos.set_ylim([zero_point_y - 1.0, 6.0])
    txt.set_text('{:.1f} sec'.format(jj * dt))
    txt.set_position((zero_point + 40.0, 6.2))

    return map_vid,


# Animation function -- Full view
def animate_ego(jj):
    last_1_sec = 20
    ego_pos = x[0, jj, 0:2]
    for aa in range(0, 2 * nAgents, 2):

        idx = int(aa / 2)
        if np.linalg.norm(x[idx, jj, 0:2] - ego_pos) > 50:
            continue
        if idx < 3:
            x_circ, y_circ = get_ex(x[idx, jj], 1.0, d_points)
        else:
            x_circ, y_circ = get_circle(x[idx, jj], 1.0, d_points)
        x_hist, y_hist = x[idx, np.max([0, jj+1 - last_1_sec]):jj+1, 0:2].T
        map_vid[aa].set_data(x_circ, y_circ)
        map_vid[aa + 1].set_data(x_hist, y_hist)
        if idx == 0:
            map_vid[aa].set_color('y')
            map_vid[aa + 1].set_color('y')
        elif idx == 1:
            map_vid[aa].set_color('b')
            map_vid[aa + 1].set_color('b')
        elif idx == 2:
            map_vid[aa].set_color('c')
            map_vid[aa + 1].set_color('c')
        elif idx == 3:
            map_vid[aa].set_color('g')
            map_vid[aa + 1].set_color('g')
        else:
            map_vid[aa].set_color('r')
            map_vid[aa + 1].set_color('r')

    ax_pos.set_xlim([x[0, jj, 0] - 45, x[0, jj, 0] + 15])
    ax_pos.set_ylim([x[0, jj, 1] - 6, x[0, jj, 1] + 6])
    txt.set_text('{:.1f} sec'.format(jj * dt))
    txt.set_position((x[0, jj, 0], x[0, jj, 1] + 7))


# Create animation
ani = animation.FuncAnimation(fig=fig_map, func=animate_ego, frames=int(ii/1), interval=10, repeat=False)
writer = animation.writers['ffmpeg']
ani.save(filename[:-4] + '.mp4', writer=writer(fps=15))

plt.tight_layout(pad=2.0)
plt.show()
