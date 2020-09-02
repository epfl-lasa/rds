import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

import capsule_distance

capsule = capsule_distance.Capsule(0.18, -0.5, 0.45)

#from matplotlib_scalebar.scalebar import ScaleBar

#plt.rcParams['axes.facecolor'] = "xkcd:spring green"#'black'

# plt.rcParams.update({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.serif": ["Palatino"],
# })
from matplotlib import rc
rc('font', **{'family': 'serif', 'serif': ['Computer Modern'], 'size' : 12})
#rc('text', usetex=True)
#from matplotlib.ticker import FormatStrFormatter

all_traces_rds = np.genfromtxt('../traces_dynamic_systematic_rds.csv',
	delimiter=';')

all_traces_baseline = np.genfromtxt('../traces_dynamic_systematic_baseline.csv',
	delimiter=';')

data = [all_traces_rds, all_traces_baseline]
fig, axes = plt.subplots(1, 2, sharey=True, subplot_kw={"adjustable":'box-forced'})

fig.subplots_adjust(wspace=0.025)

for m in [0, 1]:
	all_traces = data[m]

	traces_max_index = np.max(all_traces[:, 4])
	#traces_list = [None]*traces_max_index
	#for i in range(traces_max_index):
	#	this_trace_row_indices = (all_traces[:, 4] == i)
	#	traces_list[i] = all_traces[this_trace_row_indices, 0:4]

	#for trace in traces_list:
	#	plt.plot(trace[])
	trace_start_indices = [0]
	for i in range(all_traces.shape[0]):
		if len(trace_start_indices)-1 < all_traces[i, 4]:
			trace_start_indices.append(i)

	# re-arrange the data
	trace_start_indices.append(all_traces.shape[0])
	traces_ordered = np.empty([all_traces.shape[0]*2, 3])
	write_start = 0
	for i in range(10):
		sub_range_alpha = range(trace_start_indices[i], trace_start_indices[i+1])
		write_length = trace_start_indices[i+1] - trace_start_indices[i]
		traces_ordered[write_start:(write_start+write_length), 0:2] = all_traces[sub_range_alpha, 0:2]
		traces_ordered[write_start:(write_start+write_length), 2] = all_traces[sub_range_alpha, 4]
		write_start += write_length
		traces_ordered[write_start:(write_start+write_length), 0:2] = all_traces[sub_range_alpha, 2:4]
		traces_ordered[write_start:(write_start+write_length), 2] = all_traces[sub_range_alpha, 4]
		write_start += write_length
		sub_range_omega = range(trace_start_indices[19 - i], trace_start_indices[20 - i])
		write_length = trace_start_indices[20 - i] - trace_start_indices[19 - i]
		traces_ordered[write_start:(write_start+write_length), 0:2] = all_traces[sub_range_omega, 0:2]
		traces_ordered[write_start:(write_start+write_length), 2] = all_traces[sub_range_omega, 4]
		write_start += write_length
		traces_ordered[write_start:(write_start+write_length), 0:2] = all_traces[sub_range_omega, 2:4]
		traces_ordered[write_start:(write_start+write_length), 2] = all_traces[sub_range_omega, 4]
		write_start += write_length

	ax = axes[m]
	the_cmap = cm.coolwarm#cm.PiYG#cm.brg#cm.viridis#cm.plasma #cm.cool
	ax.scatter(traces_ordered[:,0], traces_ordered[:,1], c=traces_ordered[:,2]/float(traces_max_index),
		cmap=the_cmap, marker='o', lw=0.1, s=8, edgecolor='k')
	ax.set_aspect("equal")
	ax.set_xlim([-2.7, 3.5])
	ax.set_ylim([-2.7, 3.5])
	ax.plot([-2.7, 3.5], [0.0, 0.0], 'k--', linewidth=1)
	ax.plot([0.0, 0.0], [-2.7, 3.5], 'k--', linewidth=1)
	
	if m == 0:
		shift_x = -5.0
		shift_y = -5.0
		ax.text(2.9+shift_x, 2.6+shift_y, "1 m")
		ax.plot([2.75+shift_x,3.75+shift_x], [2.5+shift_y, 2.5+shift_y],'k', linewidth=1)
		ax.plot([2.75+shift_x,2.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)
		ax.plot([3.75+shift_x,3.75+shift_x], [2.45+shift_y, 2.55+shift_y],'k', linewidth=1)
		y_colorscale = np.linspace(0.75, 2.0, 20)
		x_colorscale = np.linspace(1.75, 1.75, 20)
		c_array = np.linspace(0.0, 1.0, 20)
		ax.scatter(x_colorscale, y_colorscale, c=c_array, cmap=the_cmap, marker='s', lw=0.0)
		ax.text(x_colorscale[0]+0.05, y_colorscale[0]-0.05, " -1.5s")
		ax.text(x_colorscale[0]+0.05, y_colorscale[-1]-0.05, "+1.5s")
		ax.text(x_colorscale[0]-0.7, y_colorscale[-1]+0.5, "pedestrian\nhead start")

	capsule.plot_at_pose(-1.7, 2.0, -np.pi/2, ax, orca=(m == 1))
	ax.quiver(-1.7, 2.0, 2.0, 0.0)
	ax.plot([-1.7, -2.6], [1.9, 0.1], 'k', linewidth=1)
	circle_pedestrian = plt.Circle((2.0, -1.7), 0.3, color=[0.8,0.8,0.0], fill=False)
	ax.add_artist(circle_pedestrian)
	ax.quiver(2.0, -1.7, 0.0, 2.0)
	ax.plot([1.9, 0.1], [-1.7, -2.6], 'k', linewidth=1)
	ax.xaxis.set_ticks([])
	ax.xaxis.set_ticklabels([])
	ax.yaxis.set_ticks([])
	ax.yaxis.set_ticklabels([])
	# continue

	# sub_range = range(trace_start_indices[10])

	# the_cmap = cm.coolwarm#cm.PiYG#cm.brg#cm.viridis#cm.plasma #cm.cool
	# plt.scatter(all_traces[sub_range,0], all_traces[sub_range,1], c=all_traces[sub_range,4]/float(traces_max_index), cmap=the_cmap, marker='o', lw=0.1)
	# plt.scatter(all_traces[sub_range,2], all_traces[sub_range,3], c=all_traces[sub_range,4]/float(traces_max_index), cmap=the_cmap, marker='o', lw=0.1)
	# plt.gca().set_aspect("equal")
	# plt.gca().set_xlim([-2.5, 3.5])
	# plt.gca().set_ylim([-2.5, 3.5])
	# plt.show()

#plt.setp(axes, xlim=[-3, 4.0], ylim=[-2.5, 4.0])
plt.savefig('dynamic_systematic_plot.png', bbox_inches='tight', dpi=299)
plt.show()