import csv
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({'font.size': 22})

filename = "../../rds/metrics_evaluation.csv"

E_t_RDS = []
E_t_ORCA = []
N_ttg_RDS = []
N_ttg_ORCA = []

with open(filename) as csvfile:
    spamreader = csv.reader(csvfile, delimiter=';')
    i = 0
    for row in spamreader:
    	if i == 0:
    		i += 1
    		continue
    	E_t_RDS.append(float(row[0]))
    	E_t_ORCA.append(float(row[1]))
    	N_ttg_RDS.append(float(row[4]))
    	N_ttg_ORCA.append(float(row[5]))
        i += 1

fig1, ax1 = plt.subplots()
ax1.set_title('Crowd time to goal ratio')
ax1.boxplot([E_t_RDS, E_t_ORCA], sym="bo", notch=True)
plt.xticks([1, 2], ['RDS', 'ORCA'])
ax1.set_ylabel("$E_t$ [-]")
plt.savefig("boxplot_E_t.png",bbox_inches='tight',dpi=100)
plt.show()

fig1, ax1 = plt.subplots()
ax1.set_title('Neighbours time to goal ratio')
ax1.boxplot([N_ttg_RDS, N_ttg_ORCA], sym="bo", notch=True)
plt.xticks([1, 2], ['RDS', 'ORCA'])
ax1.set_ylabel("$N_{ttg}$ [-]")
plt.savefig("boxplot_N_ttg.png",bbox_inches='tight',dpi=100)
plt.show()