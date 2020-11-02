import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import sys

file_name = 'command.mat'
if len(sys.argv) > 1:
	file_name = str(sys.argv[1])

mat = sio.loadmat(file_name)
print (np.shape(mat['data']))

ax1 = plt.subplot(211)
ax2 = plt.subplot(212, sharex=ax1)

ax1.plot(mat['data'][:, 0], mat['data'][:, 1], color='k', label='nominal')
ax1.plot(mat['data'][:, 0], mat['data'][:, 3], color='r', label='corrected')

ax2.plot(mat['data'][:, 0], mat['data'][:, 2], color='k', label='nominal')
ax2.plot(mat['data'][:, 0], mat['data'][:, 4], color='r', label='corrected')

ax1.set_ylabel('v [m/s]')
ax2.set_ylabel('omega [rad/s]')
ax2.set_xlabel('t [s]')
ax1.legend()
ax2.legend()
plt.show()