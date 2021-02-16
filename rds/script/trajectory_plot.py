import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap

import capsule_distance

capsule = capsule_distance.Capsule(0.18, -0.5, 0.45)
capsule_larger = capsule_distance.Capsule(0.18, -0.5, 0.5)

def cmap_map(function, cmap):
    """ Applies function (which should operate on vectors of shape 3: [r, g, b]), on colormap cmap.
    This routine will break any discontinuous points in a colormap.
    """
    cdict = cmap._segmentdata
    step_dict = {}
    # Firt get the list of points where the segments start or end
    for key in ('red', 'green', 'blue'):
        step_dict[key] = list(map(lambda x: x[0], cdict[key]))
    step_list = sum(step_dict.values(), [])
    step_list = np.array(list(set(step_list)))
    # Then compute the LUT, and apply the function to the LUT
    reduced_cmap = lambda step : np.array(cmap(step)[0:3])
    old_LUT = np.array(list(map(reduced_cmap, step_list)))
    new_LUT = np.array(list(map(function, old_LUT)))
    # Now try to make a minimal segment definition of the new LUT
    cdict = {}
    for i, key in enumerate(['red','green','blue']):
        this_cdict = {}
        for j, step in enumerate(step_list):
            if step in step_dict[key]:
                this_cdict[step] = new_LUT[j, i]
            elif new_LUT[j,i] != old_LUT[j, i]:
                this_cdict[step] = new_LUT[j, i]
        colorvector = list(map(lambda x: x + (x[1], ), this_cdict.items()))
        colorvector.sort()
        cdict[key] = colorvector

    return LinearSegmentedColormap('colormap',cdict,1024)

def add_circles(X, Y, r, ax):
	for i in range(X.shape[0]):
		circle = plt.Circle((X[i], Y[i]), r, fill=False, color="k")#[0.8,0.8,0.0])
		ax.add_artist(circle)

def plot_center_index_range(xy_robot, orientation_robot, xy_reference, X_crowd, Y_crowd, X_crowd_ref, Y_crowd_ref,
	index_range_bounds, distance_cutoff, ax, cm_past_future, baseline_method=False):
	sub_range = np.arange(index_range_bounds[0], index_range_bounds[1], 1)
	xy_robot = xy_robot[sub_range, :]
	orientation_robot = orientation_robot[sub_range]
	xy_reference = xy_reference[sub_range, :]
	X_crowd = X_crowd[sub_range, :]
	Y_crowd = Y_crowd[sub_range, :]
	X_crowd_ref = X_crowd_ref[sub_range, :]
	Y_crowd_ref = Y_crowd_ref[sub_range, :]
	if True: #distance cutoff
		x_max = np.max(xy_robot[:, 0]) + distance_cutoff
		x_min = np.min(xy_robot[:, 0]) - distance_cutoff
		y_max = np.max(xy_robot[:, 1]) + distance_cutoff
		y_min = np.min(xy_robot[:, 1]) - distance_cutoff
		ax.plot([x_max, x_max, x_min, x_min, x_max], [y_max, y_min, y_min, y_max, y_max], color=[0.8,0.8,0.8], linestyle='--')
		indices_close = np.logical_and(
			np.logical_and((x_max > X_crowd), (x_min < X_crowd)),
			np.logical_and((y_max > Y_crowd), (y_min < Y_crowd)))
		X_crowd[np.logical_not(indices_close)] = np.nan
		Y_crowd[np.logical_not(indices_close)] = np.nan
		indices_close = np.logical_and(
			np.logical_and((x_max > X_crowd_ref), (x_min < X_crowd_ref)),
			np.logical_and((y_max > Y_crowd_ref), (y_min < Y_crowd_ref)))
		X_crowd_ref[np.logical_not(indices_close)] = np.nan
		Y_crowd_ref[np.logical_not(indices_close)] = np.nan
	t_normalized = np.linspace(0.0, 1.0, xy_robot.shape[0])
	ax.scatter(X_crowd.flatten(), Y_crowd.flatten(), c=np.repeat(t_normalized, X_crowd.shape[1]),
		cmap = cm_past_future)
	lime_color = [0.25,1.0,0.4]
	#ax.scatter(xy_robot[:, 0], xy_robot[:, 1], marker='o', edgecolors="k", facecolors="None", lw=2)
	ax.scatter(xy_robot[:, 0], xy_robot[:, 1], c=t_normalized, cmap = cm_past_future)
	
	#ax.scatter(xy_reference[:, 0], xy_reference[:, 1], c=t_normalized, cmap = cm_past_future, marker='s', edgecolors='k')

	sub_sampler_ref = np.arange(0, xy_reference.shape[0], 4)
	if True: #show robot references
		ax.scatter(xy_reference[sub_sampler_ref, 0], xy_reference[sub_sampler_ref, 1],
			c=t_normalized[sub_sampler_ref], cmap=cm_past_future, marker="s")
				#facecolors="None", edgecolors=cm_past_future(t_normalized[sub_sampler_ref]), marker="x", lw=1)
		ax.scatter(xy_reference[sub_sampler_ref, 0], xy_reference[sub_sampler_ref, 1],
			marker='s', edgecolors='k', facecolors="None")

	if False: #show pedestrian references
		ax.scatter(X_crowd_ref[sub_sampler_ref, :].flatten(), Y_crowd_ref[sub_sampler_ref, :].flatten(),
			c=np.repeat(t_normalized[sub_sampler_ref], X_crowd.shape[1]),
			cmap = cm_past_future, marker="s")

	index_middle = xy_robot.shape[0]/2
	x_robot_middle = xy_robot[index_middle, 0]
	y_robot_middle = xy_robot[index_middle, 1]
	orientation_robot_middle = orientation_robot[index_middle]
	capsule.plot_at_pose(x_robot_middle, y_robot_middle, orientation_robot_middle, ax, orca=baseline_method, color="g")

	add_circles(np.transpose(X_crowd[index_middle, :]), np.transpose(Y_crowd[index_middle, :]), 0.3, ax)


def plot_robot_trajectory(xy_robot, xy_reference, ax):
	ax.plot(xy_robot[:, 0], xy_robot[:, 1], color="g", zorder=0)#, "k"), zorder=0)
	ax.plot(xy_reference[:, 0], xy_reference[:, 1], color="g", linestyle="--")#, "k--"), zorder=0)

trajectories_rds = np.genfromtxt('../trajectories_rds.csv', delimiter=';')
trajectories_baseline = np.genfromtxt('../trajectories_baseline.csv', delimiter=';')

trajectories_both_cases = [trajectories_rds, trajectories_baseline]

fig, axes = plt.subplots(2, 1, sharex=True, subplot_kw={"adjustable":'box-forced'})
fig.subplots_adjust(hspace=0.025)

for k in [0, 1]:
	ax = axes[k]
	trajectories = trajectories_both_cases[k]

	sub_sampler = np.arange(0, trajectories.shape[0], 1)
	trajectories = trajectories[sub_sampler, :]

	indices_x = np.arange(0, trajectories.shape[1] - 5, 4) + 5
	indices_y = indices_x + 1
	indices_x_ref = indices_x + 2
	indices_y_ref = indices_x + 3
	X = trajectories[:, indices_x]
	Y = trajectories[:, indices_y]
	X_ref = trajectories[:, indices_x_ref]
	Y_ref = trajectories[:, indices_y_ref]

	ax.set_xlim([-3.2, 9])
	ax.set_ylim([-4.5, 1])

	plot_robot_trajectory(trajectories[:, 0:2], trajectories[:, 3:5], ax)

	index_ranges = [
		[trajectories.shape[0]*5/40, trajectories.shape[0]*9/40],
		[150, 190]]#,
		#[trajectories.shape[0]*13/20 - 25, trajectories.shape[0]*15/20 - 25]]

	colormaps = [
		#cmap_map(lambda x: 1-x, cm.seismic),
		cmap_map(lambda x: 1-x, cm.BrBG),
		cmap_map(lambda x: 1-x, cm.seismic)]

	for i in range(len(index_ranges)):
		plot_center_index_range(trajectories[:, 0:2], trajectories[:, 2], trajectories[:, 3:5],
			X, Y, X_ref, Y_ref, index_ranges[i], 1.5, ax, colormaps[i], baseline_method=(k==1))

	if k == 0:
		shift_x = -9.5
		shift_y = -4.75
		ax.text(2.9+shift_x, 2.6+shift_y, "1 m")
		ax.plot([2.75+shift_x,3.75+shift_x], [2.5+shift_y, 2.5+shift_y],'k', linewidth=1)
		ax.plot([2.75+shift_x,2.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)
		ax.plot([3.75+shift_x,3.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)

	ax.set_aspect("equal")
	ax.xaxis.set_ticks([])
	ax.xaxis.set_ticklabels([])
	ax.yaxis.set_ticks([])
	ax.yaxis.set_ticklabels([])
plt.show()

quit()

def plot_index_range(xy_robot, xy_reference, X_crowd, Y_crowd, index_range_bounds, distance_cutoff, ax, x_lim, y_lim):
	ax.set_aspect("equal")
	ax.set_xlim(x_lim)
	ax.set_ylim(y_lim)
	M = ax.transData.get_matrix()
	markersize_arg_pedestrians = (0.3*M[0,0])**2

	sub_range = np.arange(index_range_bounds[0], index_range_bounds[1], 1)
	xy_robot = xy_robot[sub_range, :]
	xy_reference = xy_reference[sub_range, :]
	X_crowd = X_crowd[sub_range, :]
	Y_crowd = Y_crowd[sub_range, :]
	x_max = np.max(xy_robot[:, 0]) + distance_cutoff
	x_min = np.min(xy_robot[:, 0]) - distance_cutoff
	y_max = np.max(xy_robot[:, 1]) + distance_cutoff
	y_min = np.min(xy_robot[:, 1]) - distance_cutoff
	indices_close = np.logical_and(
		np.logical_and((x_max > X_crowd), (x_min < X_crowd)),
		np.logical_and((y_max > Y_crowd), (y_min < Y_crowd)))
	X_crowd[np.logical_not(indices_close)] = np.nan
	Y_crowd[np.logical_not(indices_close)] = np.nan
	t_normalized = np.linspace(0.0, 1.0, xy_robot.shape[0])
	ax.scatter(X_crowd.flatten(), Y_crowd.flatten(), c=np.repeat(t_normalized, X_crowd.shape[1]),
		cmap = cm.hot_r, s=markersize_arg_pedestrians)
	lime_color = [0.25,1.0,0.4]
	#ax.scatter(xy_robot[:, 0], xy_robot[:, 1], s=300, facecolors="None", edgecolors=lime_color, lw=1)
	ax.scatter(xy_robot[:, 0], xy_robot[:, 1], c=t_normalized, cmap = cm.hot_r, s=markersize_arg_pedestrians)
	ax.scatter(xy_robot[:, 0], xy_robot[:, 1], facecolors="None", edgecolors=lime_color, lw=0.1, s=markersize_arg_pedestrians)
	sub_sampler_ref = np.arange(0, xy_reference.shape[0], 4)
	ax.scatter(xy_reference[sub_sampler_ref, 0], xy_reference[sub_sampler_ref, 1], c=t_normalized[sub_sampler_ref], cmap = cm.hot_r, marker="s", edgecolors=lime_color)
	ax.plot([x_max, x_max, x_min, x_min, x_max], [y_max, y_min, y_min, y_max, y_max], 'b--')

trajectories_rds = np.genfromtxt('../trajectories_rds.csv', delimiter=';')
#trajectories_baseline = np.genfromtxt('../trajectories_baseline.csv', delimiter=';')
sub_sampler = np.arange(0, trajectories_rds.shape[0], 1)
trajectories_rds = trajectories_rds[sub_sampler, :]

indices_x = np.arange(0, trajectories_rds.shape[1] - 5, 4) + 5
indices_y = indices_x + 1
X = trajectories_rds[:, indices_x]
Y = trajectories_rds[:, indices_y]

fig, ax = plt.subplots(1, 1)
plot_index_range(trajectories_rds[:, 0:2], trajectories_rds[:, 3:5], X, Y,
	[0, trajectories_rds.shape[0]/5], 1.5, ax, [3, 11], [-8, -1])

ax.set_aspect("equal")
plt.show()

quit()


sub_sampler = np.arange(0, trajectories_rds.shape[0]/5, 1)
trajectories_rds = trajectories_rds[sub_sampler, :]

t_normalized = np.linspace(0.0, 1.0, trajectories_rds.shape[0])
#plt.scatter(trajectories_rds[:, 3], trajectories_rds[:, 4], facecolors="None", edgecolors=cm.hot_r(t_normalized), lw=1)

if True:

	indices_x = np.arange(0, trajectories_rds.shape[1] - 3, 4) + 3
	indices_y = indices_x + 1
	X = trajectories_rds[:, indices_x]
	Y = trajectories_rds[:, indices_y]
	plt.scatter(X.flatten(), Y.flatten(), c = np.repeat(t_normalized, indices_x.shape[0]), cmap = cm.hot_r)#, edgecolors='k')
	ax = plt.gca()
	ax.set_xlim([-10, 10])
	ax.set_ylim([-10, 10])
	ax.set_aspect("equal")

if True:
	lime_color = [0.25,1.0,0.4]
	plt.scatter(trajectories_rds[:, 3], trajectories_rds[:, 4], c = t_normalized, cmap = cm.hot_r, marker="s", edgecolors=lime_color)#, lw=1)
	plt.scatter(trajectories_rds[:, 0], trajectories_rds[:, 1], c = t_normalized, cmap = cm.hot_r, edgecolors=lime_color)#, lw=1)


plt.show()
