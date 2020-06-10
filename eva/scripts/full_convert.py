import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import sys

from scipy import interpolate

import matplotlib.cm as cm

import capsule_distance

capsule = capsule_distance.Capsule(0.051, -0.5, 0.45)
y_reference_point = 0.2

command_file_name = 'command.mat'
tracker_file_name = 'tracker_object.mat'
lrf_file_name = 'lrf_object.mat'
pose_file_name = 'pose.mat'

command_mat = sio.loadmat(command_file_name)
print ('Command mat: ', np.shape(command_mat['data']))

tracker_mat = sio.loadmat(tracker_file_name)
print ('Tracker mat: ', np.shape(tracker_mat['data']))

lrf_mat = sio.loadmat(lrf_file_name)
print ('LRF mat: ', np.shape(lrf_mat['data']))

pose_mat = sio.loadmat(pose_file_name)
print ('Pose mat: ', np.shape(pose_mat['data']))

###################################################################

t_pose = pose_mat['data'][:, 0]
x = pose_mat['data'][:, 1]
y = pose_mat['data'][:, 2]
phi = pose_mat['data'][:, 3]

x_of_t = interpolate.interp1d(t_pose, x, bounds_error=False)
y_of_t = interpolate.interp1d(t_pose, y, bounds_error=False)
phi_of_t = interpolate.interp1d(t_pose, phi, bounds_error=False)

t_lower = t_pose[0]
t_upper = t_pose[-1]

print ('t_lower=', t_lower, 't_upper=', t_upper)

t_evaluation = np.linspace(t_lower, t_upper, 5000)

ax1 = plt.subplot(311)
ax2 = plt.subplot(312, sharex=ax1)
ax3 = plt.subplot(313, sharex=ax1)
ax1.plot(t_evaluation, x_of_t(t_evaluation), label='x')
ax2.plot(t_evaluation, y_of_t(t_evaluation), label='y')
ax3.plot(t_evaluation, phi_of_t(t_evaluation), label='phi')
ax1.set_ylabel('x [m]')
ax2.set_ylabel('y [m]')
ax3.set_ylabel('phi [rad]')
ax3.set_xlabel('t [s]')
ax1.set_title('Pose')
plt.show()

###################################################################

# for jun_02
# for bag 17:
sub_range = np.arange(150, 300, 1)
plot_lrf = True

# for bag 01:
sub_range = np.arange(370, 500, 1)
plot_lrf = False

# for bag 04:
sub_range = np.arange(260, 450, 1)
plot_lrf = False

# for bag 06:
sub_range = np.arange(200, 600, 1)
plot_lrf = False

# for bag 09:
sub_range = np.arange(500, 900, 1)
plot_lrf = False

# for jun_10
# for overcoming_03
sub_range = np.arange(700, 1000, 1)
plot_lrf = False

# for backwards_04
#sub_range = np.arange(0, 300, 1)
#plot_lrf = False


###################################################################

t = command_mat['data'][sub_range, 0]
v_nominal = command_mat['data'][sub_range, 1]
w_nominal = command_mat['data'][sub_range, 2]
v_corrected = command_mat['data'][sub_range, 3]
w_corrected = command_mat['data'][sub_range, 4]

ax1 = plt.subplot(211)
ax2 = plt.subplot(212, sharex=ax1)
ax1.plot(t, v_nominal, color='k', label='nominal')
ax1.plot(t, v_corrected, color='r', label='corrected')
ax2.plot(t, w_nominal, color='k', label='nominal')
ax2.plot(t, w_corrected, color='r', label='corrected')
ax1.set_ylabel('v [m/s]')
ax2.set_ylabel('omega [rad/s]')
ax2.set_xlabel('t [s]')
ax1.legend()
ax2.legend()
plt.show()

###################################################################

track_ob = tracker_mat['data'][sub_range, :]
lrf_ob = lrf_mat['data'][sub_range, :]

track_ob_global = np.empty(track_ob.shape)
track_ob_global[:] = np.nan
lrf_ob_global = np.empty(lrf_ob.shape)
lrf_ob_global[:] = np.nan

shortest_distance_first_track = np.empty([track_ob.shape[0], 1])
shortest_distance_first_track[:] = np.nan

shortest_distance_track_all = np.empty([track_ob.shape[0], 1])
shortest_distance_track_all[:] = np.nan

shortest_distance_lrf_all = np.empty([track_ob.shape[0], 1])
shortest_distance_lrf_all[:] = np.nan

recent_track_value = 1234567.89

xy_p_ref = np.empty([t.shape[0], 2])
xy_p_ref[:] = np.nan
v_cartesian_nominal_p_ref = np.empty([t.shape[0], 2])
v_cartesian_nominal_p_ref[:] = np.nan
v_cartesian_corrected_p_ref = np.empty([t.shape[0], 2])
v_cartesian_corrected_p_ref[:] = np.nan

for i in range(track_ob.shape[0]):
	if t[i] < t_lower:
		continue
	if t[i] > t_upper:
		break
	phi_rds = phi_of_t(t[i])+np.pi/2.0

	rotation_matrix = np.array([
		[np.cos(phi_rds), -np.sin(phi_rds)],
		[np.sin(phi_rds),  np.cos(phi_rds)]])
	translation_vector = np.array([x_of_t(t[i]), y_of_t(t[i])])

	xy_p_ref[i, :] = np.matmul(rotation_matrix, np.array([0.0, y_reference_point])) + translation_vector
	v_cartesian_nominal_p_ref_local = np.array([-w_nominal[i]*y_reference_point, v_nominal[i]])
	v_cartesian_nominal_p_ref[i, :] = np.matmul(rotation_matrix, v_cartesian_nominal_p_ref_local)
	v_cartesian_corrected_p_ref_local = np.array([-w_corrected[i]*y_reference_point, v_corrected[i]])
	v_cartesian_corrected_p_ref[i, :] = np.matmul(rotation_matrix, v_cartesian_corrected_p_ref_local)

	for j in range(lrf_ob.shape[1]/2):
		if np.isnan(lrf_ob[i, j*2]):
			break
		lrf_ob_global[i, j*2:(j + 1)*2] = np.matmul(rotation_matrix, lrf_ob[i, j*2:(j + 1)*2]) + translation_vector
		
		min_dist = capsule.distance(lrf_ob[i, j*2], lrf_ob[i, j*2 + 1])
		if np.isnan(shortest_distance_lrf_all[i, 0]) or (shortest_distance_lrf_all[i, 0] > min_dist):
			shortest_distance_lrf_all[i, 0] = min_dist

	if recent_track_value == track_ob[i, 0]:
		continue
	else:
		recent_track_value = track_ob[i, 0]

	for j in range(track_ob.shape[1]/4):
		if np.isnan(track_ob[i, j*4]):
			break
		track_ob_global[i, j*4:j*4+2] = np.matmul(rotation_matrix, track_ob[i, j*4:j*4+2]) + translation_vector
		track_ob_global[i, j*4+2:(j + 1)*4] = np.matmul(rotation_matrix, track_ob[i, j*4+2:(j + 1)*4]) + translation_vector
		min_dist = capsule.distance(track_ob[i, j*4], track_ob[i, j*4+1])
		if np.isnan(shortest_distance_track_all[i, 0]) or (shortest_distance_track_all[i, 0] > min_dist):
			shortest_distance_track_all[i, 0] = min_dist
		if j == 0:
			shortest_distance_first_track[i, 0] = min_dist

v_scale = 1.0
v_sampler = np.arange(0, t.shape[0], 20)
plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_nominal_p_ref[v_sampler, 0], v_cartesian_nominal_p_ref[v_sampler, 1], color='b', scale=v_scale, angles='xy', scale_units='xy', width=0.005)
plt.quiver(xy_p_ref[v_sampler, 0], xy_p_ref[v_sampler, 1], v_cartesian_corrected_p_ref[v_sampler, 0], v_cartesian_corrected_p_ref[v_sampler, 1], color=(0,1,0,0.5), scale=v_scale, angles='xy', scale_units='xy', width=0.005)
plt.scatter(xy_p_ref[:, 0], xy_p_ref[:, 1], c=t/(t_upper-t_lower), cmap=cm.hot, marker='o', edgecolors='k')
plt.scatter(track_ob_global[:,0], track_ob_global[:,1], c=t/(t_upper-t_lower), cmap=cm.hot, marker='s', edgecolors='g')
plt.scatter(track_ob_global[:,2], track_ob_global[:,3], c=t/(t_upper-t_lower), cmap=cm.hot, marker='x')

if plot_lrf:
	even = np.arange(0, lrf_ob_global.shape[1], 2)
	odd = np.arange(1, lrf_ob_global.shape[1], 2)
	plt.scatter(lrf_ob_global[:, even].flatten(), lrf_ob_global[:, odd].flatten())

ax = plt.gca()
ax.set_aspect('equal')
ax.set_ylabel('y [m]')
ax.set_xlabel('x [m]')
ax.set_title('Trajectories')
plt.show()

plt.plot(t, shortest_distance_first_track[:, 0]-0.3, 'ko')
plt.plot(t, np.ones(t.shape)*0.05, 'r')
plt.plot(t, np.zeros(t.shape), 'k')
ax = plt.gca()
ax.set_ylabel('distance [m]')
ax.set_xlabel('t [s]')
ax.set_title('Shortest distance')
plt.show()

#################################################################################################

# write out

sio.savemat('xy_p_ref.mat', {'data' : xy_p_ref})
sio.savemat('v_cartesian_nominal_p_ref.mat', {'data' : v_cartesian_nominal_p_ref})
sio.savemat('v_cartesian_corrected_p_ref.mat', {'data' : v_cartesian_corrected_p_ref})
sio.savemat('shortest_distance_lrf_all.mat', {'data' : shortest_distance_lrf_all})
sio.savemat('shortest_distance_first_track.mat', {'data' : shortest_distance_first_track})
sio.savemat('shortest_distance_track_all.mat', {'data' : shortest_distance_track_all})
sio.savemat('track_ob_global.mat', {'data' : track_ob_global})
sio.savemat('lrf_ob_global.mat', {'data' : lrf_ob_global})