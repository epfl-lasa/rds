import matplotlib.pyplot as plt
import numpy as np

x = np.arange(-np.pi/2, np.pi/2, 0.001)
cosx = np.cos(x)
sinx = np.sin(x)
cosxapp = 1.0 - 0.5*np.power(x, 2)
sinxapp = x

xcosx = x*np.cos(x)
xsinx = x*np.sin(x)
xcosxapp = x
xsinxapp = x*x

plt.plot(x, cosx, 'b', label='cos(x)')
plt.plot(x, sinx, 'k', label='sin(x)')
plt.plot(x, cosxapp, 'r', label='1 - x^2/2')
plt.plot(x, sinxapp, 'c', label='x')
plt.plot(x, xcosx, 'b--', label='x cos(x)')
plt.plot(x, xsinx, 'k--', label='x sin(x)')
plt.plot(x, xcosxapp, 'r--', label='x')
plt.plot(x, xsinxapp, 'c--', label='x^2')
plt.gca().set_xlim([-np.pi/2, np.pi/2])
plt.gca().set_ylim([-1, 1])
plt.legend(loc='lower right')
plt.show()