import numpy as np
from os.path import expanduser
import math
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({'font.size': 16})

all_filenames = [
"v0.500000-d0.200000.txt",
"v0.500000-d0.600000.txt",
"v0.500000-d1.000000.txt",
"v0.700000-d0.200000.txt",
"v0.700000-d0.600000.txt",
"v0.700000-d1.000000.txt",
"v0.900000-d0.200000.txt",
"v0.900000-d0.600000.txt",
"missing",
"v1.200000-d0.200000.txt",
"v1.200000-d0.600000.txt",
"v1.200000-d1.000000.txt",
"v1.500000-d0.200000.txt",
"v1.500000-d0.600000.txt",
"v1.500000-d1.000000.txt"
]

previous_v_data = [
0.46,
0.28,
0.16,
0.64,
0.39,
0.22,
0.83,
0.56,
0.31,
1.02,
0.74,
0.43,
1.36,
0.91,
0.51
]

def compute_mean_and_std(filename, previous_v_value):
	if filename == "missing":
		return (np.nan, np.nan)
	with open(expanduser('~/Desktop/v_log/' + filename), 'r') as f:
	    vt_list = [float(word) for line in f for word in line.split()]

	    v_list = vt_list[0:len(vt_list)/2]
	    t_list = vt_list[len(vt_list)/2:]

	    #v_list = v_list[len(v_list)/5:]
	    #t_list = t_list[len(t_list)/5:]

	    dtsum = 0.0
	    t = []
	    for dt in t_list:
	    	t.append(dtsum)
	    	dtsum += dt

	    samples = []
	    n = 0
	    local_mean = 0.0
	    for i in range(len(t)):
	    	if t[i] < dtsum/10*(len(samples)+0.5):
	    		continue
	    	n += 1
	    	local_mean += v_list[i]

	    	if (t[i] > dtsum/10*(len(samples)+1.0)) or ((i == len(t)-1) and (len(samples) < 10)):
	    		samples.append(local_mean/n)
	    		local_mean = 0.0
	    		n = 0

	    samples.append(previous_v_value)
	    sample_weights = [0.07]*10
	    sample_weights.append(0.3)
	    bias_correction = 0.0
	    for w in sample_weights:
	    	bias_correction += w*w

	    mean_mean = 0.0
	    for i in range(11):
	    	mean_mean += samples[i]*sample_weights[i]

	    std = 0.0
	    for i in range(11):
	    	std += (samples[i] - mean_mean)*(samples[i] - mean_mean)*sample_weights[i]
	    std = math.sqrt(std/(1 - bias_correction))

	    return (mean_mean, std)

	    #N = len(v_list)
	    #mean = 0.0
	    #for i in range(N):
	    #    mean += v_list[i]*t_list[i]/dtsum

	    #std = 0.0
	    #for i in range(N):
	    #    std += (v_list[i] - mean)*(v_list[i] - mean)*t_list[i]
	    #std = math.sqrt(std/(dtsum - 1.0))

	    print ("mean = %f" % mean_mean)
	    print ("std = %f" % std)


v_lines = [[],[],[]]
std_lines = [[],[],[]]

for j in range(5):
	for i in range(3):
		mean, std = compute_mean_and_std(all_filenames[j*3+i], previous_v_data[j*3+i])
		v_lines[i].append(mean)
		std_lines[i].append(std)

v_nominal = [0.5, 0.7, 0.9, 1.2, 1.5]
density = np.array([0.2, 0.6, 1.0])

plt.plot(v_nominal, v_lines[0], "ko-", label=r"$v$ for $\rho=" + str(density[0]) + "$")
plt.plot()
plt.plot(v_nominal, v_lines[1], "ro-", label=r"$v$ for $\rho=" + str(density[1]) + "$")
plt.plot(
	[v_nominal[0], v_nominal[1], v_nominal[3], v_nominal[4]],
	[v_lines[2][0], v_lines[2][1], v_lines[2][3], v_lines[2][4]],
	"co-", label=r"$v$ for $\rho=" + str(density[2]) + "$")

line_specs = ['k_-', 'r_-', 'c_-']
for j in range(5):
	for i in range(3):
		if j == 2 and i == 3:
			continue
		plt.plot([v_nominal[j], v_nominal[j]],
			[v_lines[i][j] + std_lines[i][j], v_lines[i][j] - std_lines[i][j]], line_specs[i])

plt.gca().set_ylabel("$v$ [m/s]")
plt.gca().set_xlabel("$v_{des}$ [m/s]")
plt.legend()
plt.gca().set_aspect("equal")
v_range = [0.0, 1.55]
plt.gca().set_xlim(v_range)
plt.gca().set_ylim(v_range)
plt.plot(v_range, v_range, "k:")
plt.savefig("v_unity.png",bbox_inches='tight',dpi=200)
plt.show()