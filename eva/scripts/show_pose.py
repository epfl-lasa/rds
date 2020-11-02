import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import sys

file_name = 'pose.mat'
if len(sys.argv) > 1:
	file_name = str(sys.argv[1])

mat = sio.loadmat(file_name)
print (np.shape(mat['data']))

plt.plot(mat['data'][:, 1], mat['data'][:, 2], 'ko-')
plt.plot(mat['data'][-1, 1], mat['data'][-1, 2], 'ro')
plt.plot(mat['data'][0, 1], mat['data'][0, 2], 'bo')
ax = plt.gca()
ax.set_aspect('equal')
plt.show()

ax1 = plt.subplot(311)
ax2 = plt.subplot(312, sharex=ax1)
ax3 = plt.subplot(313, sharex=ax1)
ax1.plot(mat['data'][:, 0], mat['data'][:, 1])
ax2.plot(mat['data'][:, 0], mat['data'][:, 2])
ax3.plot(mat['data'][:, 0], mat['data'][:, 3])
ax1.set_ylabel('x [m]')
ax2.set_ylabel('y [m]')
ax3.set_ylabel('phi [rad]')
ax3.set_xlabel('t [s]')
plt.show()