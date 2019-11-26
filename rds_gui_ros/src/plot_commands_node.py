#!/usr/bin/env python

import rospy
from rds_network_ros.msg import ToGui

import signal
import sys

from matplotlib import pyplot as plt

#command_time = []
corrected_command_linear = []
corrected_command_angular = []
nominal_command_linear = []
nominal_command_angular = []

def signal_handler(sig, frame):
	plt.plot(nominal_command_linear, color='blue', label='nominal')
	plt.plot(corrected_command_linear, color='green', label='corrected')
	plt.title("Linear command (nominal and corrected)")
	plt.show()
	plt.plot(nominal_command_angular, color='blue', label='nominal')
	plt.plot(corrected_command_angular, color='green', label='corrected')
	plt.title("Angular command (nominal and corrected)")
	plt.show()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
#print('Press Ctrl+C')
#signal.pause()

def callback(data):
	#command_time.append()
	corrected_command_linear.append(data.corrected_command.linear)
	corrected_command_angular.append(data.corrected_command.linear)
	nominal_command_linear.append(data.nominal_command.linear)
	nominal_command_angular.append(data.nominal_command.linear)
    
def main():
	rospy.init_node('plot_commands_node')
	rospy.Subscriber("rds_to_gui", ToGui, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()