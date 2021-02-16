#!/usr/bin/env python

import rospy
from rds_network_ros.msg import ToGui
import numpy as np
import time
import scipy.io as sio
import signal
import sys

n_max = 30*300 # 30 Hz for 5 min

commands_data = np.empty([n_max, 5])
# [[time, nominal_v, nominal_w, corrected_v, corrected_w]]
lrf_objects_data = np.empty([n_max, 4000]) # store xy for 2000 objects
lrf_objects_data[:] = np.nan
tracker_objects_data = np.empty([n_max, 20]) # store xy and xy_pred for 5 objects
tracker_objects_data[:] = np.nan

counter = 0
start_time = None
previous_t = -100.0

def save():
	global commands_data
	global lrf_objects_data
	global tracker_objects_data
	commands_data = commands_data[0:counter, :]
	lrf_objects_data = lrf_objects_data[0:counter, :]
	tracker_objects_data = tracker_objects_data[0:counter, :]
	sio.savemat('command.mat', {'data' : commands_data})
	sio.savemat('tracker_object.mat', {'data' : tracker_objects_data})
	sio.savemat('lrf_object.mat', {'data' : lrf_objects_data})

def signal_handler(sig, frame):
	save()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callbackToGui(msg):
	global commands_data
	global lrf_objects_data
	global tracker_objects_data
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
	commands_data[counter, :] = np.array([t, nominal_v, nominal_w, corrected_v, corrected_w])

	n_tracker_objects = len(msg.moving_objects_predictions)/2
	for i in range(n_tracker_objects):
		x_pred = msg.moving_objects_predictions[i*2].x
		y_pred = msg.moving_objects_predictions[i*2].y
		x_now = msg.moving_objects_predictions[i*2 + 1].x
		y_now = msg.moving_objects_predictions[i*2 + 1].y
		if i*4 + 3 >= tracker_objects_data.shape[1]:
			print('More tracker-objects than expected')
			break
		tracker_objects_data[counter, i*4] = x_now
		tracker_objects_data[counter, i*4 + 1] = y_now
		tracker_objects_data[counter, i*4 + 2] = x_pred
		tracker_objects_data[counter, i*4 + 3] = y_pred

	n_lrf_objects = len(msg.moving_objects) - n_tracker_objects
	for i in range(n_lrf_objects):
		if i*2 + 1 >= lrf_objects_data.shape[1]:
			print('More lrf-objects than expected')
			break 
		lrf_objects_data[counter, i*2] = msg.moving_objects[i].center.x
		lrf_objects_data[counter, i*2 + 1] = msg.moving_objects[i].center.y
	counter += 1

def main():
	rospy.init_node('extract_rosbag_command_object_node')
	rospy.Subscriber('rds_to_gui', ToGui, callbackToGui)
	print ('Ready ...')
	rospy.spin()

if __name__ == '__main__':
	main()