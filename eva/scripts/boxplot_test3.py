import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
import matplotlib

import csv

# import seaborn as sns, matplotlib.pyplot as plt
# tips = sns.load_dataset("tips")
# sns.boxplot(x="day", y="total_bill", data=tips, palette="PRGn")

# # statistical annotation
# x1, x2 = 2, 3   # columns 'Sat' and 'Sun' (first column: 0, see plt.xticks())
# y, h, col = tips['total_bill'].max() + 2, 2, 'k'
# plt.plot([x1, x1, x2, x2], [y, y+h, y+h, y], lw=1.5, c=col)
# plt.text((x1+x2)*.5, y+h, "ns", ha='center', va='bottom', color=col)

# plt.show()

matplotlib.rcParams['font.family'] = 'Times New Roman'

_, ax1 = plt.subplots()
ax2 = ax1.twinx()
# fig2, ax1 = plt.subplots()

# reader = csv.reader(open("metrics_evaluation.csv", "rb"), delimiter=",")
# x = list(reader)
# data = numpy.array(x).astype("float")

data = np.genfromtxt("metrics_evaluation_2.csv", delimiter=';', names=True, case_sensitive=True)

ORCA_N_v = data['ORCA_N_v']
RDS_N_v = data['RDS_N_v']
ORCA_N_v =[ (i/10) + 0.9 for i in ORCA_N_v]
RDS_N_v = [ (i/10) + 0.9 for i in RDS_N_v]

plt.figure(1)

# ORCA = [data['ORCA_E_t'], data['ORCA_E_v'], data['ORCA_N_ttg'], ORCA_N_v]
# RDS = [data['RDS_E_t'], data['RDS_E_v'], data['RDS_N_ttg'], RDS_N_v]

ORCA = [data['ORCA_E_v'], ORCA_N_v]
RDS = [data['RDS_E_v'], RDS_N_v]

# ticks = ['Deviation','E_t', 'E_v', 'N_ttg', 'N_v']
ticks = ['V_c','V_n']

def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)


bpl = ax1.boxplot(ORCA, positions=np.array(range(len(ORCA)))*2.0-0.4, sym='k+', widths=0.6,notch=True, showfliers=False, showmeans=True)
bpr = ax1.boxplot(RDS, positions=np.array(range(len(RDS)))*2.0+0.4, sym='k+', widths=0.6,notch=True, showfliers=False, showmeans=True)
set_box_color(bpl, '#D7191C') # colors are from http://colorbrewer2.org/
set_box_color(bpr, '#2C7BB6')

# draw temporary red and blue lines and use them to create a legend
ax1.plot([], c='#D7191C', label='ORCA')
ax1.plot([], c='#2C7BB6', label='RDS')
ax1.legend(prop={'size': 11})

# set up ticks
ax1.set_xticks(range(0, len(ticks) * 2, 2))
ax1.set_xticklabels(ticks, rotation=0, ha = 'center') # ha indicate center of rotation, could be 'right'，'center'，'left'
# ax1.set_xticklabels([])
ax1.set_xlim(-2, len (ticks)*2)
ax1.set_ylim(0.9,1.1)
ax1.set_ylabel('Effect on Velocity', fontsize=15, fontstyle='normal')

ax1.set_yticks([0.9,0.95,1.0,1.05,1.1])
# ax1.set_yticklabels([r'$0$', r'$25$', r'$50$', r'$75$', r'$100$']))
ax1.set_yticklabels([0.9,0.95,1.0,1.05,1.1])

ax2.set_yticks(np.linspace(0, 2, 5))
ax2.set_ylim(0,2.0)
ax2.set_ylabel('Effect on Neighbours', fontsize=15, fontstyle='normal')

ax1.tick_params(labelsize=15)
ax2.tick_params(labelsize=15)
x1=-0.8; x2 = 0.8
y = 1.5; h = 0.03
b_col='k'
plt.plot([x1,x1, x2, x2], [y, y+h, y+h, y], linewidth=1, color=b_col)
plt.text((x1+x2)*.5, y+h, "*", ha='center', va='bottom', color=b_col)
# plt.plot([1.2,1.2, 2.8, 2.8], [1.6, 1.62, 1.62, 1.6], linewidth=1, color='k')

# grid(color='r', linestyle='-', linewidth=2)
plt.grid(True)
plt.tight_layout()
plt.savefig('boxplot_rds_orca_3.pdf')
plt.show()

# input("Press any key to continue")

##################### Plotting Only Deviation error #######################
# plt.clf()
# ax4 = ax1.twinx()
# plt.figure(1)


# ORCA_ped_track_err = data['ORCA_ped_track_err']
# RDS_ped_track_err = data['RDS_ped_track_err']

# ORCA_ped_track_err = [i*6 for i in ORCA_ped_track_err]
# RDS_ped_track_err = [i*6 for i in RDS_ped_track_err]

# ORCA_Data = [data['ORCA_rob_track_err'], ORCA_ped_track_err] #, data['ORCA_N_v']]
# RDS_Data = [data['RDS_rob_track_err'], RDS_ped_track_err]# , data['RDS_N_v']]
# # ticks = ['Deviation','E_t', 'E_v', 'N_ttg', 'N_v']
# ticks = ['Robot','Pedestrians']

# def set_box_color(bp, color):
#     plt.setp(bp['boxes'], color=color)
#     plt.setp(bp['whiskers'], color=color)
#     plt.setp(bp['caps'], color=color)
#     plt.setp(bp['medians'], color=color)

# bpl_2 = ax1.boxplot(ORCA_Data, positions=np.array(range(len(ORCA_Data)))*2.0-0.4, sym='k+', widths=0.6,notch=True, showfliers=False, showmeans=True)
# bpr_2 = ax1.boxplot(RDS_Data, positions=np.array(range(len(RDS_Data)))*2.0+0.4, sym='k+', widths=0.6,notch=True, showfliers=False, showmeans=True)
# set_box_color(bpl_2, '#D7191C') # colors are from http://colorbrewer2.org/
# set_box_color(bpr_2, '#2C7BB6')

# # draw temporary red and blue lines and use them to create a legend
# ax1.plot([], c='#D7191C', label='ORCA')
# ax1.plot([], c='#2C7BB6', label='RDS')
# ax1.legend(prop={'size': 11})

# # set up ticks
# ax1.set_xticks(range(0, len(ticks) * 2, 2))
# ax1.set_xticklabels(ticks, rotation=0, ha = 'center') # ha indicate center of rotation, could be 'right'，'center'，'left'
# # ax1.set_xticklabels([])
# ax1.set_xlim(-2, len (ticks)*2)
# ax1.set_ylim(0.0,6.0)
# ax1.set_ylabel('(E_r) Deviation Error [m]', fontsize=15, fontstyle='normal')

# ax1.set_yticks(np.linspace(0, 6, 5))
# # ax1.set_yticklabels([r'$0$', r'$25$', r'$50$', r'$75$', r'$100$']))
# ax1.set_yticklabels(np.linspace(0, 6, 5))

# ax1.tick_params(labelsize=15)
# ax2.tick_params(labelsize=15)
# ax2.set_yticks(np.linspace(0, 1, 5))
# ax2.set_ylim(0,1.0)
# ax2.set_ylabel('(E_p) Deviation Error [m]', fontsize=15, fontstyle='normal')

# ax1.tick_params(labelsize=15)
# ax2.tick_params(labelsize=15)

# # Plotting significance bars
# b_col='k'
# x1=-0.8; x2 = 0.8
# y = 0.92; h = 0.03
# plt.plot([x1,x1, x2, x2], [y, y+h, y+h, y], linewidth=1, color=b_col)
# plt.text((x1+x2)*.5, y+h, "*", ha='center', va='bottom', color=b_col)
# # plt.plot([-0.8,-0.8, 0.8, 0.8], [0.92, 0.94, 0.94, 0.92], linewidth=1, color='k')
# x1=1.2; x2 = 2.8
# y = 0.53; h = 0.03
# plt.plot([x1,x1, x2, x2], [y, y+h, y+h, y], linewidth=1, color=b_col)
# plt.text((x1+x2)*.5, y+h, "*", ha='center', va='bottom', color=b_col)
# # plt.plot([1.2,1.2, 2.8, 2.8], [0.52, 0.54, 0.54, 0.52], linewidth=1, color='k')

# plt.grid(True)
# plt.tight_layout()
# plt.savefig('boxplot_rds_orca_error_3.pdf')
# plt.show()#

# input("Press any key to continue")
