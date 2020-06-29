import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.rcParams.update({'font.size': 16})

#T = np.array([ [47.1, 35.3, 28.0], [70.3, 52.9, 43.0] ])
V = np.array([ [0.46, 0.64, 0.83, 1.02, 1.36],
	[0.28, 0.39, 0.56, 0.74, 0.91],
	[0.16, 0.22, 0.31, 0.43, 0.51] ])
v_nominal = np.array([0.5, 0.7, 0.9, 1.2, 1.5])
density = np.array([0.2, 0.6, 1.0])


plt.plot(v_nominal, V[0, :], "ko-", label=r"$v$ for $\rho=" + str(density[0]) + "$")
plt.plot(v_nominal, V[1, :], "ro-", label=r"$v$ for $\rho=" + str(density[1]) + "$")
plt.plot(v_nominal, V[2, :], "co-", label=r"$v$ for $\rho=" + str(density[2]) + "$")
plt.gca().set_ylabel("$v$ [m/s]")
plt.gca().set_xlabel("$v_{des}$ [m/s]")
plt.legend(loc="upper left")
plt.gca().set_aspect("equal")
v_range = [0.0, 1.55]
plt.gca().set_xlim(v_range)
plt.gca().set_ylim(v_range)
plt.plot(v_range, v_range, "k:")
plt.savefig("v_unity.png",bbox_inches='tight',dpi=100)
plt.show()

quit()

plt.plot(v_nominal, T[0, :], "ko-", label=r"$T$ for $\rho=" + str(density[0]) + "$")
plt.plot(v_nominal, T[1, :], "ro-", label=r"$T$ for $\rho=" + str(density[1]) + "$")
plt.gca().set_ylabel("$T$ [s]")
plt.gca().set_xlabel("$v_{des}$ [m/s]")
plt.legend()
v_range = [0.85, 1.55]
plt.gca().set_xlim(v_range)
t_range = [25.0, 75.0]
plt.gca().set_ylim(t_range)
plt.savefig("T_unity.png",bbox_inches='tight',dpi=100)
plt.show()