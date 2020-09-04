import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import LinearSegmentedColormap

import capsule_distance

capsule = capsule_distance.Capsule(0.18, -0.5, 0.45)
k_robot = 0.25
k_pedestrian = 0.5
dt = 0.05

def integrate_reference_motion(xy_zero, gain_k, xy_ref, dt):
	# [dx/dt = d(x_ref)/dt + k*(x_ref - x)]
	# x_ref_new = x_ref + c*exp(-k*t)
	# dx/dt = d(x_ref_new)/dt + k*(x_ref_new - x) = 
	#       = d(x_ref)/dt - k*c*exp(-k*t) + k*(x_ref + c*exp(-k*t) - x) =
	#       = d(x_ref)/dt + k*(x_ref - x)
	# The motion law is invariant under addition of c*exp(-k*t) to the reference trajectory
	# x_ref_new(0) = x_ref(0) + c
	# x_ref_new(0) - x(0) = x_ref(0) + c - x(0)
	# Setting c = x(0) - x_ref(0) yields x_ref_new(0) - x(0) = 0, i.e. the new ref. traj.
	# passes through the initial position. The reference motion which the robot would follow
	# from its initial position is therefore equal to x_ref(t) + (x(0) - x_ref(0))*exp(-k*t)

	t = np.arange(0, xy_ref.shape[0])*dt
	return xy_ref + (np.reshape(xy_zero, (1,2)) - np.reshape(xy_ref[0, :], (1,2)))*np.reshape(np.exp(-gain_k*t), (xy_ref.shape[0],1))

def integrate_crowd_reference_motion(X_zero, Y_zero, gain_k, X_ref, Y_ref, dt):
	t = np.arange(0, X_ref.shape[0])*dt
	X_int = X_ref + np.reshape(X_zero - X_ref[0, :], (1, X_ref.shape[1]))*np.reshape(np.exp(-gain_k*t), (X_ref.shape[0], 1))
	Y_int = Y_ref + np.reshape(Y_zero - Y_ref[0, :], (1, X_ref.shape[1]))*np.reshape(np.exp(-gain_k*t), (X_ref.shape[0], 1))
	return (X_int, Y_int)

def add_circles(X, Y, r, ax):
	for i in range(X.shape[0]):
		circle = plt.Circle((X[i], Y[i]), r, fill=False, color="k")#[0.8,0.8,0.0])
		ax.add_artist(circle)

def plot_index_range(ax, index_range_bounds, baseline_method,
		xy_robot, orientation_robot, xy_robot_ref, X_crowd, Y_crowd, X_crowd_ref, Y_crowd_ref):
	# consider sub-range
	sub_range = np.arange(index_range_bounds[0], index_range_bounds[1], 1)
	xy_robot = xy_robot[sub_range, :]
	orientation_robot = orientation_robot[sub_range]
	xy_robot_ref = xy_robot_ref[sub_range, :]
	X_crowd = X_crowd[sub_range, :]
	Y_crowd = Y_crowd[sub_range, :]
	X_crowd_ref = X_crowd_ref[sub_range, :]
	Y_crowd_ref = Y_crowd_ref[sub_range, :]
	# shift the data to center the robot
	x_robot_zero = xy_robot[0, 0]
	y_robot_zero = xy_robot[0, 1]
	orientation_robot_zero = orientation_robot[0]
	shift = np.array([[x_robot_zero, y_robot_zero]])
	xy_robot -= shift
	xy_robot_ref -= shift
	X_crowd -= shift[0, 0]
	Y_crowd -= shift[0, 1]
	X_crowd_ref -= shift[0, 0]
	Y_crowd_ref -= shift[0, 1]
	x_robot_zero = 0.0
	y_robot_zero = 0.0
	# plot the future trajectories for the pedestrians and the robot
	t_normalized = np.linspace(0.0, 1.0, xy_robot.shape[0])
	ax.scatter(X_crowd.flatten(), Y_crowd.flatten(),
		c=np.repeat(t_normalized, X_crowd.shape[1]), cmap = cm.hot_r, s=30)
	ax.scatter(xy_robot[:, 0], xy_robot[:, 1], c=t_normalized, cmap = cm.hot_r, s=30)
	# plot their footprints at the current state
	capsule.plot_at_pose(x_robot_zero, y_robot_zero, orientation_robot_zero, ax, orca=baseline_method, color="g")
	add_circles(np.transpose(X_crowd[0, :]), np.transpose(Y_crowd[0, :]), 0.3, ax)
	# plot their reference trajectories
	xy_robot_ref_new = integrate_reference_motion(xy_robot[0, :], k_robot, xy_robot_ref, dt)
	#ORIGINAL TRAJECTORY: ax.plot(xy_robot_ref[:, 0], xy_robot_ref[:, 1], "g:")
	ax.plot(xy_robot_ref_new[:, 0], xy_robot_ref_new[:, 1], "g--")
	X_crowd_ref_new, Y_crowd_ref_new = integrate_crowd_reference_motion(X_crowd[0,:], Y_crowd[0,:], k_pedestrian, X_crowd_ref, Y_crowd_ref, dt)
	ax.plot(X_crowd_ref_new, Y_crowd_ref_new, "k--")

trajectories_rds = np.genfromtxt('../trajectories_rds.csv', delimiter=';')
trajectories_baseline = np.genfromtxt('../trajectories_baseline.csv', delimiter=';')

trajectories_both_cases = [trajectories_rds, trajectories_baseline]

window_width = 5.0*1.2#*2.0/3.0
window_height = 3.0*1.25
m = 3
n = 2
fig, axs = plt.subplots(m, n, sharex=True, sharey=True, figsize=(10,10*(window_height*m)/(window_width*n)))
fig.subplots_adjust(wspace=0.0, hspace=0.0)

#index_windows = [
#	[[3, 80], [95, 150], [260, 310]],#[170, 220]],
#	[[3, 90], [90, 145], [260, 310]]
#] # pass through/avoid + consequences + pass through smoothly/requiring a lot of space 
index_windows = [
	[[3, 80], [3, 90]],
	[[95, 150], [90, 145]],
	[[260, 310], [260, 310]]
]

for i in range(m):
	for j in range(n):
		ax = axs[i, j]
		ax.set_xlim([-window_width*2/3, window_width/3])
		ax.set_ylim([-window_height*3/8, window_height*5/8])

		trajectories = trajectories_both_cases[j]
		sub_sampler = np.arange(0, trajectories.shape[0], 1)
		trajectories = trajectories[sub_sampler, :]

		indices_x = np.arange(0, trajectories.shape[1] - 5, 4) + 5
		indices_y = indices_x + 1
		indices_x_ref = indices_x + 2
		indices_y_ref = indices_x + 3
		X_crowd = trajectories[:, indices_x]
		Y_crowd = trajectories[:, indices_y]
		X_crowd_ref = trajectories[:, indices_x_ref]
		Y_crowd_ref = trajectories[:, indices_y_ref]
		xy_robot = trajectories[:, 0:2]
		orientation_robot = trajectories[:, 2]
		xy_robot_ref = trajectories[:, 3:5]

		index_range = index_windows[i][j]
		baseline_method = (j == 1)
		plot_index_range(ax, index_range, baseline_method,
			xy_robot, orientation_robot, xy_robot_ref, X_crowd, Y_crowd, X_crowd_ref, Y_crowd_ref)

		ax.set_aspect("equal")
		ax.xaxis.set_ticks([])
		ax.xaxis.set_ticklabels([])
		ax.yaxis.set_ticks([])
		ax.yaxis.set_ticklabels([])

plt.savefig('crowd_trajectory_plot.png', bbox_inches='tight', dpi=299)
plt.show()
