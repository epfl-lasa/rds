import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

class AnimateCurves:
	def __init__(self):
		self.fig, self.ax = plt.subplots()
		self.xdata, self.ydata = [], []
		self.ln, = plt.plot([], [], 'ro')
		self.curves = None
		self.x_lim = [-1.0, 1.0]
		self.y_lim = [-1.0, 1.0]
		self.n_circle_corners = 100

	def init_ani(self):
		self.ax.set_xlim(self.x_lim[0], self.x_lim[1])
		self.ax.set_ylim(self.y_lim[0], self.y_lim[1])
		self.ax.set_aspect('equal')
		return self.ln,
	
	def update(self, frame):
		for i in range(len(self.curves)):
			center_x = self.curves[i][0](frame)
			center_y = self.curves[i][1](frame)
			self.xdata[i*(self.n_circle_corners + 1)] = center_x
			self.ydata[i*(self.n_circle_corners + 1)] = center_y
			for j in range(self.n_circle_corners):
				phi = j*np.pi*2.0/self.n_circle_corners
				px = center_x + self.curves[i][2]*np.cos(phi)
				py = center_y + self.curves[i][2]*np.sin(phi)
				self.xdata[i*(self.n_circle_corners + 1) + j + 1] = px
				self.ydata[i*(self.n_circle_corners + 1) + j + 1] = py
		self.ln.set_data(self.xdata, self.ydata)
		return self.ln,

	def animate(self, curves, time_vector, x_lim, y_lim):
		self.curves = curves
		self.xdata = [0.0]*len(curves)*(self.n_circle_corners + 1)
		self.ydata = [0.0]*len(curves)*(self.n_circle_corners + 1)
		self.x_lim = x_lim
		self.y_lim = y_lim
		ani = FuncAnimation(self.fig, self.update, frames=time_vector,
			init_func = self.init_ani, blit=True, interval = 10)
		plt.show()


def motion_A_x(t):
	return t*0.6

def motion_A_y(t):
	return t*0.2

def motion_B_x(t):
	return 2.0 + t*0.4

def motion_B_y(t):
	return 8.0 - t*0.05

radius_A = 1.0
radius_B = 2.3 + radius_A
linear_curves = [[motion_A_x, motion_A_y, 0.001], [motion_B_x, motion_B_y, radius_B]]

v_rel_x = motion_A_x(1.0) - motion_B_x(1.0) - (motion_A_x(0.0) - motion_B_x(0.0))
v_rel_y = motion_A_y(1.0) - motion_B_y(1.0) - (motion_A_y(0.0) - motion_B_y(0.0))
v_rel = np.sqrt(v_rel_x*v_rel_x + v_rel_y*v_rel_y)

def motion_contact_point_x(t):
	return motion_B_x(t) + v_rel_y/v_rel*radius_B

def motion_contact_point_y(t):
	return motion_B_y(t) - v_rel_x/v_rel*radius_B

linear_curves.append([motion_contact_point_x, motion_contact_point_y, 0.01])

anicrvs = AnimateCurves()
anicrvs.animate(linear_curves, np.array([0.0]), [0.0, 30.0], [0.0, 10.0])

anicrvs = AnimateCurves()
anicrvs.animate(linear_curves, np.arange(0.0, 60.0, 0.05), [0.0, 30.0], [0.0, 10.0])