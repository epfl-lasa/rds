import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt

mat = sio.loadmat('processed_rosbag.mat')
print (np.shape(mat['data']))
plt.plot(mat['data'][:, 1], mat['data'][:, 2], 'ko-')
plt.plot(mat['data'][-1, 1], mat['data'][-1, 2], 'ro')
plt.show()