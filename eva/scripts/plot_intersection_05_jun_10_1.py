import numpy as np
import scipy.io as sio
from scipy import interpolate
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import capsule_distance

capsule = capsule_distance.Capsule(0.051, -0.5, 0.45)
y_reference_point = 0.18
tau = 1.5

folder_path = "jun_10_1/rdsntersection_05/"
picture_export_prefix = "../pictures/intersection_05_jun_10_1_"

pose_file_name = folder_path + "pose.mat"
xy_p_ref_file_name = folder_path + "t_xy_p_ref.mat"
v_nom_file_name = folder_path + "t_v_cartesian_nominal_p_ref.mat"
v_corr_file_name = folder_path + "t_v_cartesian_corrected_p_ref.mat"
d_tracks_file_name = folder_path + "t_shortest_distance_track_all.mat"
tracks_file_name = folder_path + "t_track_ob_global.mat"
command_file_name = folder_path + "command.mat"

pose_mat = sio.loadmat(pose_file_name)
xy_p_ref_mat = sio.loadmat(xy_p_ref_file_name)
v_nom_mat = sio.loadmat(v_nom_file_name)
v_corr_mat = sio.loadmat(v_corr_file_name)
d_tracks_mat = sio.loadmat(d_tracks_file_name)
tracks_mat = sio.loadmat(tracks_file_name)
command_mat = sio.loadmat(command_file_name)

t_pose = pose_mat['data'][:, 0]
x = pose_mat['data'][:, 1]
y = pose_mat['data'][:, 2]
phi = pose_mat['data'][:, 3]

x_of_t = interpolate.interp1d(t_pose, x, bounds_error=False)
y_of_t = interpolate.interp1d(t_pose, y, bounds_error=False)
phi_of_t = interpolate.interp1d(t_pose, phi, bounds_error=False)

t = xy_p_ref_mat['data'][:, 0]
xy_p_ref = xy_p_ref_mat['data'][:, 1:3]
v_cartesian_nominal_p_ref = v_nom_mat['data'][:, 1:3]
v_cartesian_corrected_p_ref = v_corr_mat['data'][:, 1:3]
d = d_tracks_mat['data'][:, 1]
tracks = tracks_mat['data'][:, 1:]

i_command_start = None
for i in range(command_mat['data'].shape[0]):
	if (command_mat['data'][i, 0] >= t[0]) and (i_command_start == None):
		i_command_start = i
	if command_mat['data'][i, 0] >= t[-1]:
		i_command_end = i
		break
commands = command_mat['data'][i_command_start:(i_command_end + 1), :]

d_min = 10000.0
i_d_min = 0
t_d_min_1 = t[0] + 12.5
t_d_min_2 = t_d_min_1 + 4.0
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

plt.scatter(xy_p_ref[:, 0], xy_p_ref[:, 1], c=t/(t[-1]-t[0]), cmap=cm.hot_r, marker='o', edgecolors='k')
plt.scatter(xy_p_ref[:, 0], xy_p_ref[:, 1], c=t/(t[-1]-t[0]), cmap=cm.hot_r, marker='o')#, edgecolors='k')
for j in range(4, -1, -1):
	plt.scatter(tracks[:,j*2], tracks[:,j*2+1], c=t/(t[-1]-t[0]), cmap=cm.hot_r, marker="^", edgecolors='k')
	plt.scatter(tracks[:,j*2], tracks[:,j*2+1], c=t/(t[-1]-t[0]), cmap=cm.hot_r, marker="^")

#plt.plot(xy_p_ref[i_d_min, 0], xy_p_ref[i_d_min, 1], 'kx')
plt.plot(xy_p_ref[i_d_min_1, 0], xy_p_ref[i_d_min_1, 1], 'kx')
plt.plot(xy_p_ref[i_d_min_2, 0], xy_p_ref[i_d_min_2, 1], 'kx')
ax = plt.gca()
capsule.plot_at_pose(x_of_t(t_d_min_1), y_of_t(t_d_min_1), phi_of_t(t_d_min_1)+np.pi/2.0, ax)
capsule.plot_at_pose(x_of_t(t_d_min_2), y_of_t(t_d_min_2), phi_of_t(t_d_min_2)+np.pi/2.0, ax)
for j in range(tracks.shape[1]/4):
	if np.isnan(tracks[i_d_min_1, j*4]):
		break
	circle_d_min_1 = plt.Circle((tracks[i_d_min_1, j*4], tracks[i_d_min_1, j*4+1]), 0.3, color='g', fill=False)
	ax.add_artist(circle_d_min_1)
for j in range(tracks.shape[1]/4):
	if np.isnan(tracks[i_d_min_2, j*4]):
		break
	circle_d_min_2 = plt.Circle((tracks[i_d_min_2, j*4], tracks[i_d_min_2, j*4+1]), 0.3, color='g', fill=False)
	ax.add_artist(circle_d_min_2)
#v_sampler = v_sampler = np.arange(0, t.shape[0], 10)#[i_d_min_1, i_d_min_2]
#plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_nominal_p_ref[v_sampler, 0], v_cartesian_nominal_p_ref[v_sampler, 1], color=(0,0,1,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005)
#plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_corrected_p_ref[v_sampler, 0], v_cartesian_corrected_p_ref[v_sampler, 1], color=(0,1,0,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005)

ax.set_aspect('equal')
ax.set_ylabel('y [m]')
ax.set_xlabel('x [m]')
#ax.set_title('Trajectories')
ax.set_xlim([-7.5, 0])
ax.set_ylim([-4, 4])
plt.savefig(picture_export_prefix + 'trajectories.png', bbox_inches='tight', dpi=199)
plt.show()

plt.plot(t-t[0], d-0.3, 'ko')
plt.plot(t-t[0], np.ones(t.shape)*0.05, 'r')
plt.plot(t-t[0], np.zeros(t.shape), 'k')
ax = plt.gca()
ax.set_ylabel('distance [m]')
ax.set_xlabel('t [s]')
#ax.set_title('Shortest distance')
plt.savefig(picture_export_prefix + 'distance.png', bbox_inches='tight', dpi=199)
plt.show()

plt.scatter(np.zeros(t.shape), t-t[0], c=t/(t[-1]-t[0]), cmap=cm.hot_r, marker='s')
plt.gca().set_ylabel('t [s]')
plt.gca().set_aspect('equal')
#ax.set_title('Trajectories')
plt.gca().set_xlim([(t[-1]-t[0])/2/20.0, -(t[-1]-t[0])/2/20.0])
plt.gca().set_xticks([], [])
plt.savefig(picture_export_prefix + 'color_map.png', bbox_inches='tight', dpi=199)
plt.show()

t_commands = commands[:, 0]
v_nominal = commands[:, 1]
w_nominal = commands[:, 2]
v_corrected = commands[:, 3]
w_corrected = commands[:, 4]

ax1 = plt.subplot(211)
ax2 = plt.subplot(212, sharex=ax1)
ax1.plot(t_commands-t[0], v_nominal, color='k', label='nominal')
ax1.plot(t_commands-t[0], v_corrected, color='r', label='corrected')
ax2.plot(t_commands-t[0], w_nominal, color='k', label='nominal')
ax2.plot(t_commands-t[0], w_corrected, color='r', label='corrected')
ax1.set_ylabel('v [m/s]')
ax2.set_ylabel('omega [rad/s]')
ax2.set_xlabel('t [s]')
ax1.legend()
ax2.legend()
plt.savefig(picture_export_prefix + 'commands.png', bbox_inches='tight', dpi=199)
plt.show()

##### compute and print metrics #####

risk = 0.0
severity = 0.0
for j in range(1):
	for i in range(t.shape[0]):
		if np.isnan(tracks[i,j*2]) or np.isnan(t[i]):
			continue
		ped_x = tracks[i,j*2]
		ped_y = tracks[i,j*2+1]
		ped_v_x = (tracks[i,j*2+2] - tracks[i,j*2])/tau
		ped_v_y = (tracks[i,j*2+3] - tracks[i,j*2+1])/tau
		rob_x = x_of_t(t[i]) #xy_p_ref[i, 0]
		rob_y = y_of_t(t[i])#xy_p_ref[i, 1]
		rob_v_x = -np.sin(phi_of_t(t_d_min_1)+np.pi/2.0)*v_corrected[i]#v_cartesian_corrected_p_ref[i, 0]
		rob_v_y = np.cos(phi_of_t(t_d_min_1)+np.pi/2.0)*v_corrected[i]#v_cartesian_corrected_p_ref[i, 1]
		diff_x = ped_x - rob_x
		diff_y = ped_y - rob_y
		v_rel_x = rob_v_x - ped_v_x
		v_rel_y = rob_v_y - ped_v_y
		r = 0.45 + 0.3
		v_rel_diff = v_rel_x*diff_x + v_rel_y*diff_y
		v_rel_v_rel = v_rel_x*v_rel_x + v_rel_y*v_rel_y
		diff_diff = diff_x*diff_x + diff_y*diff_y
		discriminant = 4.0*(v_rel_diff*v_rel_diff - v_rel_v_rel*(diff_diff - r*r))
		if discriminant <= 0.0:
			continue
		ttca = (2.0*v_rel_diff - np.sqrt(discriminant))/2.0/v_rel_v_rel
		if ttca < 0.001:
			ttca = 0.001
		risk += 1.0/ttca
		severity += 1.0/ttca*v_rel_v_rel

risk /= t.shape[0]
severity /= t.shape[0]

print ("Risk: ", risk)
print ("Severity: ", severity)
print ("Minimum distance: ", d_min-0.3)

