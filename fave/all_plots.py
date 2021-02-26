import numpy as np
import scipy.io as sio
from scipy import interpolate
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import capsule_distance_new

capsule = capsule_distance_new.Capsule(0.18, -0.5, 0.45)
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

def extract_samples_centers(t_seq, f_seq):
	threshold = (np.max(f_seq.shape[0]) - np.min(f_seq.shape[0]))/10000.0
	centers = [[f_seq[0], t_seq[0]]]
	for i in range(1, f_seq.shape[0]):
		if np.abs(centers[-1][0] - f_seq[i]) > threshold:
			centers[-1][1] = (t_seq[i-1] + centers[-1][1])/2.0
			centers.append([f_seq[i], t_seq[i]])
	centers[-1][1] = (t_seq[-1] + centers[-1][1])/2.0
	tf_c = np.zeros([len(centers), 2])
	for i in range(len(centers)):
		tf_c[i, :] = np.array([centers[i][1], centers[i][0]])
	return tf_c

def compute_metrics(x_spline, y_spline, t_seq):
	path_length = 0.0
	v_seq = np.zeros([t_seq.shape[0]-1])
	for i in range(1, t_seq.shape[0]):
		dx = x_spline(t_seq[i]) - x_spline(t_seq[i-1])
		dy = y_spline(t_seq[i]) - y_spline(t_seq[i-1])
		ds = np.sqrt(dx**2 + dy**2)
		path_length += ds
		dt = t_seq[i] - t_seq[i-1]
		v_seq[i-1] = ds/dt
	v_mean = np.mean(v_seq)
	v_var = np.var(v_seq)
	return (path_length, v_mean, v_var, v_seq)

def evaluate_metrics(t_pose, x_pose, y_pose, name):
	tx_c = extract_samples_centers(t_pose, x_pose)
	ty_c = extract_samples_centers(t_pose, y_pose)
	x_cubic_spline = interpolate.interp1d(tx_c[:, 0], tx_c[:, 1], kind="cubic", bounds_error=False)
	y_cubic_spline = interpolate.interp1d(ty_c[:, 0], ty_c[:, 1], kind="cubic", bounds_error=False)
	#plt.plot(t_pose, x_cubic_spline(t_pose), "k")
	#plt.plot(t_pose, x_pose, "ko")
	#plt.plot(t_pose, y_cubic_spline(t_pose), "r")
	#plt.plot(t_pose, y_pose, "ro")
	#plt.show()

	t_regular = np.linspace(np.max([tx_c[0,0], ty_c[0,0]]), np.min([tx_c[-1,0], ty_c[-1,0]]), 100)
	path_length, v_mean, v_var, v_seq = compute_metrics(x_cubic_spline, y_cubic_spline, t_regular)
	print name
	print "path_length=%f, v_mean=%f, v_std=%f" % (path_length, v_mean, np.sqrt(v_var))
	
	fig, axs = plt.subplots(3, 1)
	axs[0].plot(t_pose, x_pose, t_regular, x_cubic_spline(t_regular))
	axs[1].plot(t_pose, y_pose, t_regular, y_cubic_spline(t_regular))
	axs[2].plot(t_regular[:-1], v_seq)
	plt.show()

def plot_and_export(name, t_start, t_snapshot_1, t_snapshot_2, show_big_circle,
	x_lim, y_lim, remove_ground, rotation_angle, horizontal_t_scale, shift_meter_scale):

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

	only_path_length_and_average_velocity = True
	if only_path_length_and_average_velocity:
		#fig, axs = plt.subplots(3,1)
		#axs[0].plot(t_pose)
		#axs[1].plot(x_pose)
		#axs[2].plot(y_pose)
		#plt.show()

		#plt.plot(x_pose, y_pose,'ko')
		#plt.plot(x_pose[0], y_pose[0],'ro')
		#plt.gca().set_aspect("equal")
		#plt.show()
		t_new = t_pose[1:]
		x_new = x_pose[1:]
		y_new = y_pose[1:]
		evaluate_metrics(t_new, x_new, y_new, name)
		return

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
	t_snapshot_colors = [[0.0,1.0, 1.0], [0.0,0.0, 1.0]]
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
	capsule.plot_at_pose(x_of_t(t_d_min_1), y_of_t(t_d_min_1), phi_of_t(t_d_min_1), ax, orca=show_big_circle, color="g")
	capsule.plot_at_pose(x_of_t(t_d_min_2), y_of_t(t_d_min_2), phi_of_t(t_d_min_2), ax, orca=show_big_circle, color="g")

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
	if False: #show scale
		shift_x = -2.0 + x_of_t(t_d_min_1) + shift_meter_scale[0]
		shift_y = -2.0 + y_of_t(t_d_min_1) + shift_meter_scale[1]
		ax.text(2.9+shift_x, 2.6+shift_y, "1 m")
		ax.plot([2.75+shift_x,3.75+shift_x], [2.5+shift_y, 2.5+shift_y],'k', linewidth=1)
		ax.plot([2.75+shift_x,2.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)
		ax.plot([3.75+shift_x,3.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)

	ax.set_aspect('equal')
	ax.xaxis.set_ticks([])
	ax.xaxis.set_ticklabels([])
	ax.yaxis.set_ticks([])
	ax.yaxis.set_ticklabels([])
	#ax.set_title('Trajectories')
	if not x_lim == None:
		ax.set_xlim(x_lim)
	if not y_lim == None:
		ax.set_ylim(y_lim)
	plt.savefig(picture_export_prefix + 'trajectories.png', bbox_inches='tight', dpi=199)
	plt.show()
	#np.savetxt("debug_t_vector", t[i_start:i_d_min_2]-t[i_start])
	
	if not horizontal_t_scale:
		plt.scatter(np.zeros(np.shape(t[i_start:i_d_min_2])[0]), t[i_start:i_d_min_2]-t[i_start], c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker='s')
		plt.scatter([0.0], [t[i_d_min_1]-t[i_start]], color=t_snapshot_colors[0])
		plt.scatter([0.0], [t[i_d_min_2]-t[i_start]], color=t_snapshot_colors[1])
		plt.gca().set_ylabel('t [s]')
		plt.gca().set_aspect('equal')
		#ax.set_title('Trajectories')
		plt.gca().set_xlim([(t[i_d_min_2]-t[i_start])/2/20.0, -(t[i_d_min_2]-t[i_start])/2/20.0])
		plt.gca().set_xticks([], [])
	else:
		plt.scatter(t[i_start:i_d_min_2]-t[i_start], np.zeros(np.shape(t[i_start:i_d_min_2])[0]), c=(t[i_start:i_d_min_2]-t[i_start])/(t[i_d_min_2]-t[i_start]), cmap=cm.hot_r, marker='s')
		plt.scatter([t[i_d_min_1]-t[i_start]], [0.0], color=t_snapshot_colors[0])
		plt.scatter([t[i_d_min_2]-t[i_start]], [0.0], color=t_snapshot_colors[1])
		plt.gca().set_xlabel('t [s]')
		plt.gca().set_aspect('equal')
		#ax.set_title('Trajectories')
		plt.gca().set_ylim([(t[i_d_min_2]-t[i_start])/2/20.0, -(t[i_d_min_2]-t[i_start])/2/20.0])
		plt.gca().set_yticks([], [])
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
	["jul_16_door_rds_3", 0.0, 0.0, 8.5, False, [-1.5, 4.5], [-2, 2], False, 0.0, True, np.array([-1.0, -1.5])]
	,["jul_16_door_orca_2", 0.0, 0.0, 8.5, True, [-1.5, 2.5], [-2, 2], False, 0.0, True, np.array([-1.5, -1.75])]
	,["jul_17_row_overtaking_rds_o1", 0.0, 2.0, 12.5, False, [0, 9.5], [-2, 3], True, 0.0, True, np.array([-1.0, -1.5])]
	,["jul_17_row_overtaking_orca", 0.0, 2.0, 9, True, [5, 11], [-4, 1], True, 0.0, False, np.array([-1.0, -2.0])]
	,
	["jul_17_crowd_overtaking_rds", 4.0, 6.0, 11, False, [11.8, 18], [5.75,10]# [11.5,19], [-8.2,-1]
	, True, np.pi/4.0, False, np.array([-1.0, -1.5])]
	,["jul_17_crowd_overtaking_orca", 6.0, 8.0, 17.7, True, [14,20.25], [3.5, 7.6]# [13.7, 19.7], [-8.5, -2.5]
	, True, np.pi/4.0-0.15, False, np.array([-1.0, -2.0])]
]

for args in tests_args:
	plot_and_export(args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], args[8], args[9], args[10])