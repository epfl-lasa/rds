import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

VIDEO = False

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

	plt.scatter(points[0, i_snap][0,:], points[0, i_snap][1,:], color='g', edgecolors='g')

	circle_front = plt.Circle((0.0, 0.056), 0.45, color='b', fill=False)	
	circle_back = plt.Circle((0.0, -0.517), 0.35, color='b', fill=False)
	ax = plt.gca()
	ax.add_artist(circle_front)
	ax.add_artist(circle_back)
	props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
	# place a text box in upper left in axes coords
	textstr = 't = %.2f s'%(t_commands[i_snap] - t_commands[i_start])
	ax.text(0.7, 0.1, textstr, transform=ax.transAxes, fontsize=14,
        verticalalignment='top', bbox=props)
	ax.set_xlabel('[m]')
	ax.set_ylabel('[m]')
	ax.set_xlim([-2.5, 2.5])
	ax.set_ylim([-2.5, 2.5])
	ax.set_aspect('equal')
	plt.savefig(s.export + 'scanpoints.png', bbox_inches='tight', dpi=199)
	plt.show(block=False)
	plt.pause(0.1)
	plt.close()