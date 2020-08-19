import numpy as np
import matplotlib.pyplot as plt

samples_metrics = np.genfromtxt('../metrics_evaluation.csv',
	delimiter=';')
file = open('../metrics_evaluation.csv')
header = file.readline().split(';')
samples_metrics = samples_metrics[1:,:]

selection = np.array([9,10,11,12,3,4,7,8], dtype=int) - 1
new_labels = ['$ E_r $', '$ E_r $', '$ E_p $', '$ E_p $', '$ V_c $', '$ V_c $', '$ V_n $', '$ V_n $']
scaling = [1/5.0, 1/5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

for k in range(selection.shape[0]/2):
	fig, ax = plt.subplots(1, 1, figsize=(1, 3))
	i_rds = selection[k*2]
	i_baseline = selection[k*2 + 1]
	data = [samples_metrics[:, i_rds], samples_metrics[:, i_baseline]]
	labels = ['RDS', 'Baseline']
	title = new_labels[k*2]# + header[i_rds].strip()
	ax.boxplot(data, 1, 'x', showmeans=False)#, positions=[0.5,1.5])
	#ax.set_xlim([0.0, 2.0])
	#plt.subplots_adjust(left=0.3, right=0.7)
	ax.set_xticklabels(labels, rotation=45, fontsize=12)
	ax.set_title(title)

	plt.savefig(title[1:-1].strip()+'boxplots.png', bbox_inches='tight', dpi=199)
	plt.show()