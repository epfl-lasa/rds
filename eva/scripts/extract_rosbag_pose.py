#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import numpy as np
import time
import scipy.io as sio
import signal
import sys

n_max = 30*300 # 30 Hz for 5 min

data = np.empty([n_max, 4])
# [[time, x, y, phi]]

counter = 0
start_time = None
previous_t = -100.0

def save():
	global data
	data = data[0:counter, :]
	sio.savemat('pose.mat', {'data' : data})

def signal_handler(sig, frame):
	save()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def callbackPoseUpdate(msg):
	global data
	global counter
	global start_time
	global previous_t

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	rpy = tf.transformations.euler_from_quaternion(q)
	phi =  rpy[2]

	if start_time == None:
		start_time = rospy.Time.now() #time.time()
		t = 0.0
	else:
		t = (rospy.Time.now() - start_time).to_sec() #time.time() - start_time
		previous_t = t

	if counter >= n_max:
		print ('Longer than expected')
		return
	data[counter, :] = np.array([t, x, y, phi])
	counter += 1

def main():
	rospy.init_node('extract_rosbag_pose_node')
	rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, callbackPoseUpdate)
	print ('Ready ...')
	rospy.spin()

if __name__ == '__main__':
	main()