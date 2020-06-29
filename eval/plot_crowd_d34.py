import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import matplotlib

VIDEO = False
COMMANDS = False
SNAPSHOTS = False
DISTANCE = False
RISK = True

y_c_front = 0.056
y_c_back = -0.517
r_front = 0.45
r_back = 0.35

def min_distance(point_x, point_y):
	d_front = np.sqrt(point_x*point_x + (point_y-y_c_front)*(point_y-y_c_front))
	d_front -= r_front
	d_back = np.sqrt(point_x*point_x + (point_y-y_c_back)*(point_y-y_c_back))
	d_back -= r_back
	return np.min(np.array([d_front, d_back]))

class Selection:
	def __init__(self, export, t_start, t_stop, t_snap = 0.0):
		self.export = export
		self.t_start = t_start
		self.t_stop = t_stop
		self.t_snap = t_start + t_snap

points_mat_name = '2019-11-29-09-00-22.bag_collision_points_log.mat'
command_mat_name = '2019-11-29-09-00-22.bag_commands_log.mat'

points_mat = sio.loadmat(points_mat_name)
points = points_mat['collision_points']

command_mat = sio.loadmat(command_mat_name)
commands = command_mat['commands']

t_commands = commands[0, :]
v_nominal = commands[1, :]
w_nominal = commands[2, :]
v_corrected = commands[3, :]
w_corrected = commands[4, :]

scenes = [
	Selection('./pictures/no_crowd_', 202.0, 216.0),
	Selection('./pictures/unidir_1d_', 262.0, 282.0, 4.0),
	Selection('./pictures/bidir_1d_', 382.0, 402.0, 12.0)
]

distance_plot_data = []

for s in scenes:
	i_start = None
	i_snap = None
	for i in range(t_commands.shape[0]):
		if (t_commands[i] >= s.t_start) and (i_start == None):
			i_start = i
		if (t_commands[i] >= s.t_snap) and (i_snap == None):
			i_snap = i
		if (t_commands[i] >= s.t_stop):
			i_end = i
			break
	sub_range = np.arange(i_start, i_end + 1, 1)

	t_s = t_commands[sub_range]
	v_n_s = v_nominal[sub_range]
	w_n_s = w_nominal[sub_range]
	v_c_s = v_corrected[sub_range]
	w_c_s = w_corrected[sub_range]

	if COMMANDS:
		ax1 = plt.subplot(211)
		ax2 = plt.subplot(212, sharex=ax1)
		ax1.plot(t_s-t_s[0], v_n_s, color='k', label='nominal')
		ax1.plot(t_s-t_s[0], v_c_s, color='r', label='corrected')
		ax2.plot(t_s-t_s[0], w_n_s, color='k', label='nominal')
		ax2.plot(t_s-t_s[0], w_c_s, color='r', label='corrected')
		ax1.set_ylabel('v [m/s]')
		ax2.set_ylabel('omega [rad/s]')
		ax2.set_xlabel('t [s]')
		ax1.legend(loc='lower right')
		ax2.legend()
		plt.savefig(s.export + 'commands.png', bbox_inches='tight', dpi=199)
		plt.show(block=False)
		plt.pause(0.1)
		plt.close()

	if VIDEO:
		plt.ion()
		for i in range(i_start, i_end+1):
			plt.cla()
			plt.scatter(points[0, i][0,:], points[0, i][1,:])
			ax = plt.gca()
			ax.set_xlabel('[m]')
			ax.set_ylabel('[m]')
			ax.set_xlim([-3, 3])
			ax.set_ylim([-3, 3])
			ax.set_aspect('equal')
			ax.set_title('t = %f s'%(t_commands[i]-t_commands[i_start]))
			plt.draw()
			plt.pause(0.02)

	if SNAPSHOTS:
		for i in range(3):
			i_snap_now = i_snap + i*2
			pts = plt.scatter(points[0, i_snap_now][0,:], points[0, i_snap_now][1,:], color='g', edgecolors='g', label='scanpoints')

			y_p_ref = 0.4
			v_nominal_here = commands[1, i_snap_now]
			w_nominal_here = commands[2, i_snap_now]
			v_corrected_here = commands[3, i_snap_now]
			w_corrected_here = commands[4, i_snap_now]
			v_n_x_p_ref = -w_nominal_here*y_p_ref
			v_n_y_p_ref = v_nominal_here
			v_c_x_p_ref = -w_corrected_here*y_p_ref
			v_c_y_p_ref = v_corrected_here
			circle_front = plt.Circle((0.0, y_c_front), r_front, color='b', fill=False)	
			circle_back = plt.Circle((0.0, y_c_back), r_back, color='b', fill=False)

			ax = plt.gca()
			arrow_n = ax.quiver([0.0], [y_p_ref], [v_n_x_p_ref], [v_n_y_p_ref], color=(0,0,1,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005, label='nominal V')
			arrow_c = ax.quiver([0.0], [y_p_ref], [v_c_x_p_ref], [v_c_y_p_ref], color=(0,1,0,0.5), scale=1.0, angles='xy', scale_units='xy', width=0.005, label='corrected V')

			ax.add_artist(circle_front)
			ax.add_artist(circle_back)
			props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
			# place a text box in upper left in axes coords
			textstr = 't = %.2f s'%(t_commands[i_snap_now] - t_commands[i_start])
			ax.text(0.6, 0.1, textstr, transform=ax.transAxes, fontsize=14,
		        verticalalignment='top', bbox=props)
			ax.set_xlabel('[m]')
			ax.set_ylabel('[m]')
			ax.set_xlim([-2.5, 2.5])
			ax.set_ylim([-2.5, 2.5])
			circle_front.set_label('robot shape')
			ax.legend(loc='lower left', handles=[circle_front, arrow_n, arrow_c, pts])
			ax.set_aspect('equal')
			plt.savefig(s.export + str(i) + '_scanpoints.png', bbox_inches='tight', dpi=199)
			plt.show(block=False)
			plt.pause(0.1)
			plt.close()

	if DISTANCE:
		min_d = [np.nan]*sub_range.shape[0]
		for i in range(i_start, i_end+1):
			for j in range(points[0, i].shape[1]):
				d = min_distance(points[0, i][0, j], points[0, i][1, j])
				if np.isnan(min_d[i-i_start]):
					min_d[i-i_start] = d
				elif min_d[i-i_start] > d:
					min_d[i-i_start] = d

		distance_plot_data.append([t_s-t_s[0], min_d])

		print (np.min(np.array(min_d)))

	if RISK:
		risk = 0.0
		severity = 0.0
		for j in range(1):
			for i in range(t_s.shape[0]):
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

if len(distance_plot_data)==0:
	quit()

matplotlib.rcParams.update({'font.size': 24})
plt.figure(figsize=(20,10))
ax1 = plt.subplot(131)
ax2 = plt.subplot(132, sharey=ax1)
ax3 = plt.subplot(133, sharey=ax1)
ax1.plot(distance_plot_data[0][0], distance_plot_data[0][1], 'bo-')
ax2.plot(distance_plot_data[1][0], distance_plot_data[1][1], 'bo-')
ax3.plot(distance_plot_data[2][0], distance_plot_data[2][1], 'bo-')
plt.setp(ax2.get_yticklabels(), visible=False) 
plt.setp(ax3.get_yticklabels(), visible=False)
ax1.set_ylabel('distance [m]')
ax1.set_xlabel('t [s]')
ax2.set_xlabel('t [s]')
ax3.set_xlabel('t [s]')
ax1.set_title("Case 0")
ax2.set_title("Case 1")
ax3.set_title("Case 2")
plt.savefig(s.export + 'distances.png', bbox_inches='tight', dpi=199)
plt.show(block=False)
plt.pause(0.1)
plt.close()