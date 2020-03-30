import numpy as np
from scipy.interpolate import interp1d #BSpline
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

def read_data(filename):
	with open(filename) as f:
		for line in f:
			n_splines = int(line.split()[0])
			break
		f.seek(0)

		splines_data_indices = np.zeros([n_splines + 1], dtype=int)
		i = 1
		for line in f:
			if line.split()[-1] == "points":
				splines_data_indices[i] = int(line.split()[0]) + splines_data_indices[i - 1]
				i += 1
		f.seek(0)

		splines_position_data = np.zeros([splines_data_indices[-1], 2], dtype=float)
		splines_frame_data = np.zeros([splines_data_indices[-1]], dtype=int)
		i = 0
		for line in f:
			if line.split()[-1] == "points" or line.split()[-1] == "splines":
				continue
			splines_position_data[i, 0] = float(line.split()[0])
			splines_position_data[i, 1] = float(line.split()[1])
			splines_frame_data[i] = int(line.split()[2])
			i += 1
	return (splines_data_indices, splines_position_data, splines_frame_data)

def create_spline_curve(position_data, frame_data):
	return [interp1d(frame_data, position_data[:, 0], kind='quadratic', bounds_error = False),
			interp1d(frame_data, position_data[:, 1], kind='quadratic', bounds_error = False)]

def create_splines(splines_data_indices, splines_position_data, splines_frame_data):
	n_splines = np.size(splines_data_indices) - 1
	splines = [None]*n_splines
	for i in range(n_splines):
		j_1 = splines_data_indices[i]
		j_2 = splines_data_indices[i + 1]
		splines[i] = create_spline_curve(splines_position_data[j_1:j_2, :], splines_frame_data[j_1:j_2])
	return splines

def plot_spline_curves(spline_curves, time_vector):
	fig = plt.figure()
	ax = fig.add_subplot(111)
	for sc in spline_curves:
		ax.plot(sc[0](time_vector), sc[1](time_vector))
	ax.set_aspect('equal')
	plt.show()

class AnimateCurves:
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.xdata, self.ydata = [], []
		self.ln, = plt.plot([], [], 'ro')
		self.spline_curves = None
		self.x_lim = [-1.0, 1.0]
		self.y_lim = [-1.0, 1.0]

	def init_ani(self):
		self.ax.set_xlim(self.x_lim[0], self.x_lim[1])
		self.ax.set_ylim(self.y_lim[0], self.y_lim[1])
		self.ax.set_aspect('equal')
		return self.ln,
	
	def update(self, frame):
		for i in range(len(spline_curves)):
			self.xdata[i] = self.spline_curves[i][0](frame)
			self.ydata[i] = self.spline_curves[i][1](frame)
		self.ln.set_data(self.xdata, self.ydata)
		return self.ln,

	def animate(self, spline_curves, time_vector, x_lim, y_lim, save):
		self.spline_curves = spline_curves
		self.xdata = [0.0]*len(spline_curves)
		self.ydata = [0.0]*len(spline_curves)
		self.x_lim = x_lim
		self.y_lim = y_lim
		ani = FuncAnimation(self.fig, self.update, frames=time_vector,
			init_func = self.init_ani, blit=True, interval = 0)
		if save:
			ani = FuncAnimation(self.fig, self.update, frames=time_vector,
				init_func = self.init_ani, blit=True, interval = 50)
			# Set up formatting for the movie files
			Writer = animation.writers['ffmpeg']
			writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
			ani.save('ani.mp4', writer=writer)
		plt.show()


splines_data_indices, splines_position_data, splines_frame_data = read_data('students003_no_obstacles.vsp')
spline_curves = create_splines(splines_data_indices, splines_position_data, splines_frame_data)
#plot_spline_curves(spline_curves, np.linspace(0.0, splines_frame_data[-1], 10000))

anicrvs = AnimateCurves()
anicrvs.animate(spline_curves, np.linspace(0.0, splines_frame_data[-1], 4000),
	[-300.0, 300.0], [-300.0, 300.0], False)