import math
import matplotlib.pyplot as plt
import numpy as np

class Capsule:
	def __init__(self, y_front, y_back, r):
		self.y_front = y_front
		self.y_back = y_back
		self.r = r
	
	def distance(self, x, y):
		if y > self.y_front:
			return math.sqrt(x*x + (y - self.y_front)*(y - self.y_front)) - self.r
		elif y < self.y_back:
			return math.sqrt(x*x + (y - self.y_back)*(y - self.y_back)) - self.r
		else:
			return math.fabs(x) - self.r

	def plot_at_pose(self, x, y, phi, ax, orca=False):
		R = np.array([
			[math.cos(phi), -math.sin(phi)],
			[math.sin(phi),  math.cos(phi)]])
		T = np.array([x, y])
		c_front = np.matmul(R, np.array([0.0, self.y_front])) + T
		c_back = np.matmul(R, np.array([0.0, self.y_back])) + T
		left_front = np.matmul(R, np.array([-self.r, self.y_front])) + T
		left_back = np.matmul(R, np.array([-self.r, self.y_back])) + T
		right_front = np.matmul(R, np.array([self.r, self.y_front])) + T
		right_back = np.matmul(R, np.array([self.r, self.y_back])) + T

		circle_front = plt.Circle((c_front[0], c_front[1]), self.r, color='b', fill=False)
		circle_back = plt.Circle((c_back[0], c_back[1]), self.r, color='b', fill=False)
		ax.add_artist(circle_front)
		ax.add_artist(circle_back)
		ax.plot([left_front[0], left_back[0]], [left_front[1], left_back[1]], 'b')
		ax.plot([right_front[0], right_back[0]], [right_front[1], right_back[1]], 'b')
		if orca:
			r_orca = self.r + (self.y_front - self.y_back)
			circle_orca = plt.Circle((c_front[0], c_front[1]), r_orca, color='b', fill=False)
			ax.add_artist(circle_orca)