#!/usr/bin/env python

import rospy
from rds_network_ros.msg import ToGui
import numpy as np
import time
import scipy.io as sio
import signal
import sys

n_max = 30*300 # 30 Hz for 5 min

data = np.empty([n_max, 5])
# [[time, nominal_v, nominal_w, corrected_v, corrected_w]]

counter = 0
start_time = None
previous_t = -100.0

def save():
	global data
	data = data[0:counter, :]
	sio.savemat('command.mat', {'data' : data})

def signal_handler(sig, frame):
	save()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callbackToGui(msg):
	global data
	global counter
	global start_time
	global previous_t

	nominal_v = msg.nominal_command.linear
	nominal_w = msg.nominal_command.angular
	corrected_v = msg.corrected_command.linear
	corrected_w = msg.corrected_command.angular

	if start_time == None:
		start_time = rospy.Time.now() #time.time()
		t = 0.0
	else:
		t = (rospy.Time.now() - start_time).to_sec() #time.time() - start_time
		previous_t = t

	if counter >= n_max:
		print ('Longer than expected')
		return
	data[counter, :] = np.array([t, nominal_v, nominal_w, corrected_v, corrected_w])
	counter += 1

def main():
	rospy.init_node('extract_rosbag_command_node')
	rospy.Subscriber('rds_to_gui', ToGui, callbackToGui)
	print ('Ready ...')
	rospy.spin()

if __name__ == '__main__':
	main()