#!/usr/bin/env python

import rospy
from rds_network_ros.msg import ToGui

import signal
import sys

from matplotlib import pyplot as plt

import numpy as np
import scipy.io as sio

time_begin = []

time = []
corrected_command_linear = []
corrected_command_angular = []
nominal_command_linear = []
nominal_command_angular = []
#collision_points_on_obstacles = []

def signal_handler(sig, frame):
	plt.plot(nominal_command_linear, color='blue', label='nominal')
	plt.plot(corrected_command_linear, color='green', label='corrected')
	plt.title("Linear command (nominal and corrected)")
	plt.show()
	plt.plot(nominal_command_angular, color='blue', label='nominal')
	plt.plot(corrected_command_angular, color='green', label='corrected')
	plt.title("Angular command (nominal and corrected)")
	plt.show()

	# write the result to a npy-file and a mat-file
	result = np.array([time, nominal_command_linear, nominal_command_angular,
		corrected_command_linear, corrected_command_angular])
	np.save('command_log.npy', result)
	sio.savemat('command_log.mat', {'result': result})
	#np.save('collision_points_on_obstacles.npy', collision_points_on_obstacles)
	#sio.savemat('collision_points_on_obstacles.mat', collision_points_on_obstacles)

	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
#print('Press Ctrl+C')
#signal.pause()

def callback(data):
	#command_time.append()
	global time_begin, time, corrected_command_linear, corrected_command_angular, nominal_command_linear, nominal_command_angular
	if time_begin == []:
		time_begin = rospy.Time.now()

	time.append((rospy.Time.now() - time_begin).to_sec())
	corrected_command_linear.append(data.corrected_command.linear)
	corrected_command_angular.append(data.corrected_command.angular)
	nominal_command_linear.append(data.nominal_command.linear)
	nominal_command_angular.append(data.nominal_command.angular)
	#collision_points_on_obstacles.append(data.collision_points_on_obstacles)

def main():
	rospy.init_node('plot_commands_node')
	rospy.Subscriber("rds_to_gui", ToGui, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()