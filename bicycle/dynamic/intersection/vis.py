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
from bicycle.dynamic import nAgents
from bicycle.dynamic.physical_params import LW
from bicycle.dynamic.timing_params import dt, tf
from bicycle.dynamic.intersection.initial_conditions import xg, yg, box_width
from visualizing.helpers import get_circle, get_ex

matplotlib.rcParams.update({'figure.autolayout': True})

N = 2 * nAgents
plt.style.use(['Solarize_Light2'])
plt.rcParams["axes.prop_cycle"] = plt.cycler("color", plt.cm.viridis(np.linspace(0, 1, N)))
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
colors[0] = colors[1]
colors.reverse()

filepath = '/home/dasc/Documents/MB/datastore/swarm/'
# filepath = '/Users/mblack/Documents/datastore/swarm/'


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

        x = np.array([data[a]['x'] for a in data.keys()])
        u = np.array([data[a]['u'] for a in data.keys()])
        u0 = np.array([data[a]['u0'] for a in data.keys()])
        ii = int(data[0]['ii'] / dt)
        # cbf = np.array([data[a]['cbf'] for a in data.keys()])
    except:
        traceback.print_exc()

lwidth = 2
dash = [3, 2]
color_idx = np.array(range(0, 2*nAgents)).reshape(nAgents, 2)

# ii = int(tf / dt)
# ii = np.min([int(5.418/dt),ii])

def set_edges_black(ax):
    ax.spines['bottom'].set_color('#000000')
    ax.spines['top'].set_color('#000000')
    ax.spines['right'].set_color('#000000')
    ax.spines['left'].set_color('#000000')

plt.close('all')


############################################
### Control Trajectories ###
fig_control = plt.figure(figsize=(8, 8))
ax_cont_a = fig_control.add_subplot(211)
ax_cont_b = fig_control.add_subplot(212)
set_edges_black(ax_cont_a)
set_edges_black(ax_cont_b)

# Angular Control Inputs
ax_cont_a.plot(t[1:ii], 2 * np.pi * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
ax_cont_a.plot(t[1:ii], -2 * np.pi * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
# ax_cont_a.plot(t[1:ii], 2 * np.pi * np.ones(t[1:ii].shape), label=r'$\pm\omega_{max}$', linewidth=lwidth+1, color='k')
# ax_cont_a.plot(t[1:ii], -2 * np.pi * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
for aa in range(nAgents):
    ax_cont_a.plot(t[:ii], u[aa, :ii, 0], label='w_{}'.format(aa), linewidth=lwidth,
                   color=colors[color_idx[aa, 0]])
    # ax_cont_a.plot(t[:ii], u0[aa, :ii, 0], label='w_{}^0'.format(aa), linewidth=lwidth,
    #                color=colors[color_idx[aa, 1]], dashes=dash)
ax_cont_a.set(ylabel='w',#ylabel=r'$\omega$',
              ylim=[np.min(u[:ii, :, 0]) - 0.1, np.max(u[:ii, :, 0]) + 0.1],
              title='Control Inputs')

# Acceleration Inputs
# ax_cont_b.plot(t[1:ii], 9.81 * np.ones(t[1:ii].shape), label=r'$\pm a_{max}$', linewidth=lwidth+1, color='k')
# ax_cont_b.plot(t[1:ii], -9.81 * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
ax_cont_b.plot(t[1:ii], 9.81 * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
ax_cont_b.plot(t[1:ii], -9.81 * np.ones(t[1:ii].shape), linewidth=lwidth+1, color='k')
for aa in range(nAgents):
    ax_cont_b.plot(t[:ii], u[aa, :ii, 1], label='a_{}'.format(aa), linewidth=lwidth,
                   color=colors[color_idx[aa, 0]])
    # ax_cont_b.plot(t[:ii], u0[aa, :ii, 1], label='a_{}^0'.format(aa), linewidth=lwidth,
    #                color=colors[color_idx[aa, 1]], dashes=dash)
ax_cont_b.set(ylabel='a',#ylabel=r'$a_r$',
              ylim=[np.min(u[:ii, :, 1]) - 0.5, np.max(u[:ii, :, 1]) + 0.5])

# Plot Settings
for item in ([ax_cont_a.title, ax_cont_a.xaxis.label, ax_cont_a.yaxis.label] +
             ax_cont_a.get_xticklabels() + ax_cont_a.get_yticklabels()):
    item.set_fontsize(25)
# ax_cont_a.legend(fancybox=True)
ax_cont_a.grid(True, linestyle='dotted', color='white')

for item in ([ax_cont_b.title, ax_cont_b.xaxis.label, ax_cont_b.yaxis.label] +
             ax_cont_b.get_xticklabels() + ax_cont_b.get_yticklabels()):
    item.set_fontsize(25)
# ax_cont_b.legend(fancybox=True)
ax_cont_b.grid(True, linestyle='dotted', color='white')

plt.tight_layout(pad=2.0)


# ############################################
# ### CBF Trajectories ###
# fig_cbfs = plt.figure(figsize=(8, 8))
# ax_cbfs = fig_cbfs.add_subplot(111)
# set_edges_black(ax_cbfs)
#
# # NN-CBF Values
# ax_cbfs.plot(t[1:ii], np.zeros(t[1:ii].shape), linewidth=lwidth+1, color='k')
# for aa in range(cbf.shape[0]):
#     ax_cbfs.plot(t[:ii], cbf[aa, :ii, 0], label='h_{}'.format(aa), linewidth=lwidth,
#                    color=colors[color_idx[aa, 0]])
#     # ax_cbfs.plot(t[:ii], cbf[aa, :ii, 1], label='h_{}^0'.format(aa), linewidth=lwidth,
#     #                color=colors[color_idx[aa, 1]], dashes=dash)
# ax_cbfs.set(ylabel='h',
#             ylim=[-0.1, 250],
#             title='CBF Trajectories')
#
# # Plot Settings
# for item in ([ax_cbfs.title, ax_cbfs.xaxis.label, ax_cbfs.yaxis.label] +
#              ax_cbfs.get_xticklabels() + ax_cbfs.get_yticklabels()):
#     item.set_fontsize(25)
# ax_cbfs.legend(fancybox=True)
# ax_cbfs.grid(True, linestyle='dotted', color='white')
#
# plt.tight_layout(pad=2.0)



############################################
### State Trajectories ###
# plt.style.use(['dark_background'])
fig_map = plt.figure(figsize=(10, 10))
ax_pos = fig_map.add_subplot(111)
set_edges_black(ax_pos)

# # Set Up Road
d_points = 30
# start_p = -100.0
# end_p = 1000.0
# s_th = np.sin(15 * np.pi / 180)
# ax_pos.plot(np.linspace(start_p, end_p, d_points), -(LW / 2) * np.ones((d_points,)), linewidth=lwidth+1, color='w')
# ax_pos.plot(np.linspace(start_p, end_p, d_points), LW + LW / 2 * np.ones((d_points,)), linewidth=lwidth+1, color='w')
# ax_pos.plot(np.linspace(start_p, 0, d_points), np.linspace(start_p, 0, d_points) * s_th - LW / 2, linewidth=lwidth+1, color='w')
# ax_pos.plot(np.linspace(start_p, -LW / (s_th), d_points), np.linspace(start_p, -LW / (s_th), d_points) * s_th + LW / 2, linewidth=lwidth+1, color='w')

# px = np.tile(np.arange(start_p, end_p, 10), 2)
# # py = np.repeat(np.array([-LW / 2, LW / 2]), int(len(px)) / 2)
# py = np.repeat(np.array([LW / 2]), int(len(px)) / 2)
#
# # Add rectangles
# width = 3
# height = 0.25
# for ppx, ppy in zip(px, py):
#     ax_pos.add_patch(Rectangle(
#         xy=(ppx - width / 2, ppy - height / 2), width=width, height=height,
#         linewidth=1, color='white', fill=True))

# plt.show()

center = 2 * box_width + box_width / 2
x_c, y_c = get_circle(np.array([center, center]), 2 * box_width * np.sqrt(2) + box_width / 2, 100)
ax_pos.plot(x_c, y_c)

for aaa in range(nAgents):
    # x_c, y_c = get_ex(np.array([xg[aaa], yg[aaa]]), 0.25, d_points)
    ax_pos.plot(xg[aaa], yg[aaa], '*', markersize=10)

# x_circ, y_circ = get_circle()
# ax_pos.plot(x_circ, y_circ, 'k')


# Create variable reference to plot
map_vid = []
for aa in range(2 * nAgents):
    if (aa + 10) % 2 == 0:
        map_vid.append(ax_pos.plot([], [], linewidth=lwidth)[0])
    else:
        map_vid.append(ax_pos.plot([], [], linewidth=lwidth, dashes=dash)[0])

# Add text annotation and create variable reference
txt = ax_pos.text(0.0, 0.0, '', ha='right', va='top', fontsize=24)
txt_list = [ax_pos.text(x[aa, 0, 0], x[aa, 0, 1], '{}'.format(aa + 1), ha='right', va='top', fontsize=24) for aa in range(nAgents)]

ax_pos.set(ylim=[-1.0, 25.0],
           xlim=[-1.0, 25.0])

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
        if idx == 0:
            x_circ, y_circ = get_ex(x[idx, jj], 0.5, d_points)
        else:
            x_circ, y_circ = get_circle(x[idx, jj], 0.5, d_points)
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

    # ax_pos.set_xlim([zero_point, end_point])
    # ax_pos.set_ylim([zero_point_y - 1.0, 6.0])
    ax_pos.set(ylim=[-1.0, 25.0],
               xlim=[-1.0, 25.0])
    txt.set_text('{:.1f} sec'.format(jj * dt))
    txt.set_position((zero_point + 40.0, 6.2))

    return map_vid,


# Animation function -- Full view
def animate_ego(jj):
    last_1_sec = 40
    ego_pos = x[0, jj, 0:2]
    for aa in range(0, 2 * nAgents, 2):

        idx = int(aa / 2)
        if np.linalg.norm(x[idx, jj, 0:2] - ego_pos) > 50:
            continue
        if idx == -1:
            x_circ, y_circ = get_ex(x[idx, jj], 0.5, d_points)
        else:
            x_circ, y_circ = get_circle(x[idx, jj], 0.5, d_points)
        x_hist, y_hist = x[idx, np.max([0, jj+1 - last_1_sec]):jj+1, 0:2].T
        map_vid[aa].set_data(x_circ, y_circ)
        map_vid[aa + 1].set_data(x_hist, y_hist)
        if False:#idx <= 2:
            map_vid[aa].set_color(colors[color_idx[idx, 1]])
            map_vid[aa + 1].set_color(colors[color_idx[idx, 1]])
        else:
            map_vid[aa].set_color('r')
            map_vid[aa + 1].set_color('r')

    # ax_pos.set_xlim([x[0, jj, 0] - 45, x[0, jj, 0] + 15])
    # ax_pos.set_ylim([x[0, jj, 1] - 6, x[0, jj, 1] + 6])
    ax_pos.set(ylim=[-1.0, 25.0],
               xlim=[-1.0, 25.0])
    txt.set_text('{:.1f} sec'.format(jj * dt))
    for ee, agent_txt in enumerate(txt_list):
        agent_txt.set_position((x[ee, jj, 0], x[ee, jj, 1]))
    # txt_list = [ax_pos.text(x[aa, 0, 0], x[aa, 0, 1], '', ha='right', va='top', fontsize=24) for aa in range(nAgents)]
    # txt.set_position((x[0, jj, 0], x[0, jj, 1] + 7))


# Create animation
ani = animation.FuncAnimation(fig=fig_map, func=animate_ego, frames=int(ii/1), interval=50, repeat=False)
writer = animation.writers['ffmpeg']
# ani.save(filename[:-4] + '.mp4', writer=writer(fps=15))

plt.tight_layout(pad=2.0)
plt.show()
