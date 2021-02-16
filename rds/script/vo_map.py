import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
from matplotlib.patches import Ellipse

v_2 = np.array([-0.5, 0.15])
mid_points_y = np.arange(-3.85, 3.85, 0.01)
p_i = np.concatenate([np.zeros([mid_points_y.shape[0], 1]), np.reshape(mid_points_y, (mid_points_y.shape[0], 1))], 1)
p_2 = np.array([-0.5, 0.25])
p_ref_y = 0.3

y_prox = np.min([np.max([p_2[1], mid_points_y[0]]), mid_points_y[-1]])

tau = 1.0

combined_radius = 0.8

VO_centers_local = (np.reshape(p_2, (1, 2)) - p_i)/tau + np.reshape(v_2, (1, 2))
VO_c_proj_local = (np.reshape(p_2, (1, 2)) - np.array([[0.0, y_prox]]))/tau  + np.reshape(v_2, (1, 2))
VO_centers_ref = np.concatenate([np.reshape(VO_centers_local[:, 0]*p_ref_y/mid_points_y, (mid_points_y.shape[0], 1)),
	np.reshape(VO_centers_local[:, 1], (mid_points_y.shape[0], 1))], 1)
VO_c_proj_ref = np.concatenate([np.reshape(VO_c_proj_local[:, 0]*p_ref_y/y_prox, (1, 1)),
	np.reshape(VO_c_proj_local[:, 1], (1, 1))], 1)

VO_cap_x_half_axes = combined_radius*p_ref_y/mid_points_y/tau

l_trail = 10.0
VO_trails_local = (np.reshape(p_2, (1, 2)) - p_i)/tau*l_trail + np.reshape(v_2, (1, 2))
VO_trails_ref = np.concatenate([np.reshape(VO_trails_local[:, 0]*p_ref_y/mid_points_y, (mid_points_y.shape[0], 1)),
	np.reshape(VO_trails_local[:, 1], (mid_points_y.shape[0], 1))], 1)

fig, axes = plt.subplots(1,2)
axes[0].scatter(VO_centers_ref[:, 0], VO_centers_ref[:, 1],
	c=np.linspace(0,1,mid_points_y.shape[0]), cmap=cm.cool, label="VO axis")
axes[0].scatter(VO_c_proj_ref[:, 0], VO_c_proj_ref[:, 1], color="g", label="VO prox.")
for i in range(VO_trails_ref.shape[0]):
	axes[0].plot([VO_trails_ref[i, 0], VO_centers_ref[i, 0]], [VO_trails_ref[i, 1], VO_centers_ref[i, 1]],
		"r", zorder=0)
ellipses = []
for i in range(VO_cap_x_half_axes.shape[0]):
	ellipses.append(Ellipse(VO_centers_ref[i, :], width=2*VO_cap_x_half_axes[i], height=2*combined_radius/tau, zorder=0))
for el in ellipses:
	el.set_facecolor([0, 0, 0, 0])
	el.set_edgecolor("red")
	axes[0].add_artist(el)

#x_hyperbel = np.linspace(-10,10, 200)
#y_hyperbel = (-p_ref_y/tau*(v_2[0] + p_2[0]/tau))/x_hyperbel + p_2[1]/tau + v_2[1] - combined_radius/tau
#axes[0].plot(x_hyperbel, y_hyperbel, "k")

axes[0].set_xlim([-5, 5])
axes[0].set_ylim([-5, 5])
axes[0].set_aspect("equal")
axes[0].legend()
axes[0].set_title("ref. point velocity space")
axes[1].scatter(np.zeros([mid_points_y.shape[0]]), mid_points_y,
	c=np.linspace(0,1,mid_points_y.shape[0]), cmap=cm.cool, label="mid-axis")
axes[1].plot([0.0], [p_ref_y], "ko", label="ref. point")
axes[1].plot(p_2[0], p_2[1], "yo", label="obstacle")
axes[1].plot([0.0], [y_prox], "go", label="prox. point")
axes[1].set_title("position space")
axes[1].legend()
plt.show()
