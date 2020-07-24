import numpy as np
import scipy.io as sio
from scipy import interpolate
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import capsule_distance

capsule = capsule_distance.Capsule(0.18, -0.5, 0.45)
y_reference_point = 0.18
tau = 1.5

def rotate_pose_mat(pose_mat, angle):
	rotation_matrix = np.array([
		[np.cos(angle), -np.sin(angle)],
		[np.sin(angle),  np.cos(angle)]])
	
	pose_mat[:, 1:3] = np.matmul(pose_mat[:, 1:3], np.transpose(rotation_matrix))

	pose_mat[:, 3] = pose_mat[:, 3] + angle

def rotate_t_vector_stack(stack_mat, angle):
	rotation_matrix = np.array([
		[np.cos(angle), -np.sin(angle)],
		[np.sin(angle),  np.cos(angle)]])
	
	stack_mat[:, 1:3] = np.matmul(stack_mat[:, 1:3], np.transpose(rotation_matrix))

def rotate_t_multi_vector_stack(stack_mat, angle):
	rotation_matrix = np.array([
		[np.cos(angle), -np.sin(angle)],
		[np.sin(angle),  np.cos(angle)]])

	n_vectors = (stack_mat.shape[1] - 1)/2
	rot_diag = np.kron(np.eye(n_vectors, dtype=float), rotation_matrix)

	nan_ind = np.isnan(stack_mat)
	stack_mat[nan_ind] = 0.0
	stack_mat[:, 1:] = np.matmul(stack_mat[:, 1:], np.transpose(rot_diag))
	stack_mat[nan_ind] = np.nan

def plot_and_export(name, t_start, t_snapshot_1, t_snapshot_2, show_big_circle, x_lim, y_lim, remove_ground, rotation_angle):

	folder_path = "./" + name + "/"
	picture_export_prefix = "./pictures/a_" + name + "_"

	pose_file_name = folder_path + "pose.mat"
	xy_p_ref_file_name = folder_path + "t_xy_p_ref.mat"
	v_nom_file_name = folder_path + "t_v_cartesian_nominal_p_ref.mat"
	v_corr_file_name = folder_path + "t_v_cartesian_corrected_p_ref.mat"
	d_tracks_file_name = folder_path + "t_shortest_distance_track_all.mat"
	tracks_file_name = folder_path + "t_track_ob_global.mat"
	command_file_name = folder_path + "command.mat"
	lrf_file_name = folder_path + "t_lrf_ob_global.mat"
	d_lrf_file_name = folder_path + "t_shortest_distance_lrf_all.mat"

	pose_mat = sio.loadmat(pose_file_name) # to be rotated
	xy_p_ref_mat = sio.loadmat(xy_p_ref_file_name) # to be rotated
	v_nom_mat = sio.loadmat(v_nom_file_name) # to be rotated
	v_corr_mat = sio.loadmat(v_corr_file_name) # to be rotated
	d_tracks_mat = sio.loadmat(d_tracks_file_name)
	tracks_mat = sio.loadmat(tracks_file_name) # to be rotated
	command_mat = sio.loadmat(command_file_name)
	lrf_mat = sio.loadmat(lrf_file_name) # to be rotated
	d_lrf_mat = sio.loadmat(d_lrf_file_name)

	rotate_pose_mat(pose_mat['data'], rotation_angle)
	rotate_t_vector_stack(xy_p_ref_mat['data'], rotation_angle)
	rotate_t_vector_stack(v_nom_mat['data'], rotation_angle)
	rotate_t_vector_stack(v_corr_mat['data'], rotation_angle)
	rotate_t_multi_vector_stack(tracks_mat['data'], rotation_angle)
	rotate_t_multi_vector_stack(lrf_mat['data'], rotation_angle)

	pose_without_nan_rows = pose_mat['data'][~np.isnan(pose_mat['data']).any(axis=1)]

	t_pose = pose_without_nan_rows[:, 0]
	x_pose = pose_without_nan_rows[:, 1]
	y_pose = pose_without_nan_rows[:, 2]
	phi_pose = pose_without_nan_rows[:, 3]

	x_of_t = interpolate.interp1d(t_pose, x_pose, bounds_error=False)
	y_of_t = interpolate.interp1d(t_pose, y_pose, bounds_error=False)
	phi_of_t = interpolate.interp1d(t_pose, phi_pose, bounds_error=False)

	t = xy_p_ref_mat['data'][:, 0]
	xy_p_ref = xy_p_ref_mat['data'][:, 1:3]
	v_cartesian_nominal_p_ref = v_nom_mat['data'][:, 1:3]
	v_cartesian_corrected_p_ref = v_corr_mat['data'][:, 1:3]
	d = d_tracks_mat['data'][:, 1]
	tracks = tracks_mat['data'][:, 1:]
	lrf = lrf_mat['data'][:, 1:]
	d_lrf = d_lrf_mat['data'][:, 1]

	d_all = np.minimum(d_lrf, d)
	d = d_all

	d_min = 10000.0
	i_d_min = 0
	t_start += t[1]
	i_start = None
	t_d_min_1 = t[1] + t_snapshot_1
	t_d_min_2 = t[1] + t_snapshot_2
	i_d_min_1 = None
	i_d_min_2 = None
	for i in range(d.shape[0]):
		if d_min > d[i]:
			i_d_min = i
			d_min = d[i]
		if (t[i] > t_d_min_1) and (i_d_min_1 == None):
			i_d_min_1 = i
		if (t[i] > t_d_min_2) and (i_d_min_2 == None):
			i_d_min_2 = i
		if (t[i] > t_start) and (i_start == None):
			i_start = i

	i_snapshots = [i_d_min_1, i_d_min_2]
	t_snapshot_colors = [[0.0,1.0, 1.0], [0.0,1.0, 0.0]]
	plot_lrf = True
	#lrf_sampler = np.arange(0, t.shape[0]/5, 20)
	#lrf_points_sampled = lrf[lrf_sampler, :]
	if plot_lrf:
		even = np.arange(0, lrf.shape[1], 2)
		odd = np.arange(1, lrf.shape[1], 2)
		#for k in range(lrf_points_sampled.shape[0]):
		#	plt.scatter(lrf_points_sampled[k,even].flatten(), lrf_points_sampled[k, odd].flatten())
		#plt.scatter(lrf[0, even].flatten(), lrf[0, odd].flatten())
		if not remove_ground:
			plt.scatter(lrf[i_d_min_1, even].flatten(), lrf[i_d_min_1, odd].flatten(), color=t_snapshot_colors[0])
			plt.scatter(lrf[i_d_min_2, even].flatten(), lrf[i_d_min_2, odd].flatten(), color=t_snapshot_colors[1])
		else:
			r_cutoff_squared = 16.0
			for i_snap in range(2):
				i_now = i_snapshots[i_snap]
				X_lrf_now = lrf[i_now, even].flatten() 
				Y_lrf_now = lrf[i_now, odd].flatten()
				R_now_squared = np.power(X_lrf_now - xy_p_ref[i_now, 0], 2) + np.power(Y_lrf_now - xy_p_ref[i_now, 1], 2)
				i_cutoff = R_now_squared < r_cutoff_squared
				plt.scatter(X_lrf_now[i_cutoff], Y_lrf_now[i_cutoff], color=t_snapshot_colors[i_snap])

	plt.scatter(xy_p_ref[i_start:i_d_min_2, 0], xy_p_ref[i_start:i_d_min_2, 1], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker='o', edgecolors='k')
	plt.scatter(xy_p_ref[i_start:i_d_min_2, 0], xy_p_ref[i_start:i_d_min_2, 1], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker='o')#, edgecolors='k')
	for j in range(tracks.shape[1]/4):
		plt.scatter(tracks[i_start:i_d_min_2,j*4], tracks[i_start:i_d_min_2,j*4+1], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker="^", edgecolors='k')
		plt.scatter(tracks[i_start:i_d_min_2,j*4], tracks[i_start:i_d_min_2,j*4+1], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker="^")

	#plt.plot(xy_p_ref[i_d_min, 0], xy_p_ref[i_d_min, 1], 'kx')
	plt.plot(xy_p_ref[i_d_min_1, 0], xy_p_ref[i_d_min_1, 1], 'kx')
	plt.plot(xy_p_ref[i_d_min_2, 0], xy_p_ref[i_d_min_2, 1], 'kx')
	ax = plt.gca()
	capsule.plot_at_pose(x_of_t(t_d_min_1), y_of_t(t_d_min_1), phi_of_t(t_d_min_1), ax, orca=show_big_circle)
	capsule.plot_at_pose(x_of_t(t_d_min_2), y_of_t(t_d_min_2), phi_of_t(t_d_min_2), ax, orca=show_big_circle)

	for j in range(tracks.shape[1]/4):
		if np.isnan(tracks[i_d_min_1, j*4]):
			break
		circle_d_min_1 = plt.Circle((tracks[i_d_min_1, j*4], tracks[i_d_min_1, j*4+1]), 0.3, color=t_snapshot_colors[0], fill=False)
		ax.add_artist(circle_d_min_1)
	for j in range(tracks.shape[1]/4):
		if np.isnan(tracks[i_d_min_2, j*4]):
			break
		circle_d_min_2 = plt.Circle((tracks[i_d_min_2, j*4], tracks[i_d_min_2, j*4+1]), 0.3, color=t_snapshot_colors[1], fill=False)
		ax.add_artist(circle_d_min_2)
#v_sampler = v_sampler = np.arange(0, t.shape[0], 10)#[i_d_min_1, i_d_min_2]
#plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_nominal_p_ref[v_sampler, 0], v_cartesian_nominal_p_ref[v_sampler, 1], color=(0,0,1,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005)
#plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_corrected_p_ref[v_sampler, 0], v_cartesian_corrected_p_ref[v_sampler, 1], color=(0,1,0,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005)

	ax.set_aspect('equal')
	ax.set_ylabel('y [m]')
	ax.set_xlabel('x [m]')
	#ax.set_title('Trajectories')
	if not x_lim == None:
		ax.set_xlim(x_lim)
	if not y_lim == None:
		ax.set_ylim(y_lim)
	plt.savefig(picture_export_prefix + 'trajectories.png', bbox_inches='tight', dpi=199)
	plt.show()
	#np.savetxt("debug_t_vector", t[i_start:i_d_min_2]-t[i_start])
	plt.scatter(np.zeros(np.shape(t[i_start:i_d_min_2])[0]), t[i_start:i_d_min_2]-t[i_start], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker='s')
	plt.scatter([0.0], [t[i_d_min_1]-t[i_start]], color=t_snapshot_colors[0])
	plt.scatter([0.0], [t[i_d_min_2]-t[i_start]], color=t_snapshot_colors[1])
	plt.gca().set_ylabel('t [s]')
	plt.gca().set_aspect('equal')
	#ax.set_title('Trajectories')
	plt.gca().set_xlim([(t[i_d_min_2]-t[i_start])/2/20.0, -(t[i_d_min_2]-t[i_start])/2/20.0])
	plt.gca().set_xticks([], [])
	plt.savefig(picture_export_prefix + 'color_map.png', bbox_inches='tight', dpi=199)
	plt.show()

	commands = command_mat['data'][:, :]
	t_commands = commands[i_start:i_d_min_2, 0]
	v_nominal = commands[i_start:i_d_min_2, 1]
	w_nominal = commands[i_start:i_d_min_2, 2]
	v_corrected = commands[i_start:i_d_min_2, 3]
	w_corrected = commands[i_start:i_d_min_2, 4]

	ax1 = plt.subplot(211)
	ax2 = plt.subplot(212, sharex=ax1)
	ax1.plot(t_commands-t[i_start], v_nominal, color='k', label='nominal')
	ax1.plot(t_commands-t[i_start], v_corrected, color='r', label='corrected')
	ax2.plot(t_commands-t[i_start], w_nominal, color='k', label='nominal')
	ax2.plot(t_commands-t[i_start], w_corrected, color='r', label='corrected')
	ax1.set_ylabel('v [m/s]')
	ax2.set_ylabel('omega [rad/s]')
	ax2.set_xlabel('t [s]')
	ax1.legend()
	ax2.legend()
	plt.savefig(picture_export_prefix + 'commands.png', bbox_inches='tight', dpi=199)
	plt.show()


tests_args = [
	["jul_16_door_rds_3", 0.0, 0.0, 8.5, False, [-1.5, 4.5], [-2, 2], False, 0.0]
	,["jul_16_door_orca_2", 0.0, 0.0, 8.5, True, [-1.5, 2.5], [-2, 2], False, 0.0]
	,["jul_17_row_overtaking_rds_o1", 0.0, 2.0, 12.5, False, [0, 9.5], [-2, 3], True, 0.0]
	,["jul_17_row_overtaking_orca", 0.0, 2.0, 9, True, [5, 11], [-4, 1], True, 0.0]
	,
	["jul_17_crowd_overtaking_rds", 4.0, 6.0, 11, False, [11.8, 18], [5.75,10]# [11.5,19], [-8.2,-1]
	, True, np.pi/4.0]
	,["jul_17_crowd_overtaking_orca", 6.0, 8.0, 17.7, True, [14,20.25], [3.5, 7.6]# [13.7, 19.7], [-8.5, -2.5]
	, True, np.pi/4.0-0.15]
]

for args in tests_args:
	plot_and_export(args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], args[8])