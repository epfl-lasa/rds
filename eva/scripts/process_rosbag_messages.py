#!/usr/bin/env python

import rospy
from rds_network_ros.msg import ToGui
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import numpy as np
import time
import scipy.io as sio
import signal
import sys

t_max = 300.0 # [s] is the maximum rosbag length
log_frequency = 10.0 # [Hz] is the frequency for grabbing messages from the rosbag
log_period = 1.0/log_frequency
n_max = int(t_max*log_frequency)
data = np.empty([n_max, 14])
# [[time, x, y, phi, v_nominal, w_nominal, v_corr, w_corr, ref_x, ref_y, ref_v_nominal_x, ref_v_nominal_y, ref_v_corr_x, ref_v_corr_y]]

counter = 0
start_time = None
previous_t = -100.0
#tf_listener = None
x = None
y = None
phi = None

def zero_pose():
	global x
	global y
	global phi
	x = 0.0
	y = 0.0
	phi = 0.0

def save():
	global data
	data = data[0:counter, :]
	sio.savemat('processed_rosbag.mat', {'data' : data})

def signal_handler(sig, frame):
	save()
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#def get_pose():
	#global tf_listener
	#(trans, rot) = tf_listener.lookupTransform('/tf_qolo_world', '/tf_rds', rospy.Time(0))
#	rpy = tf.transformations.euler_from_quaternion(rot)
#	return (trans[0], trans[1], rpy[2])

def callbackPoseUpdate(msg):
	global x
	global y
	global phi

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
	#print (x, y, phi, rospy.Time.now())

	if start_time == None:
		start_time = rospy.Time.now() #time.time()
		t = 0.0
	else:
		t = (rospy.Time.now() - start_time).to_sec() #time.time() - start_time
		#if t - previous_t < log_period:
		#	return
		previous_t = t

	if counter >= n_max:
		print ('Longer than expected')
		return
	data[counter, :] = np.array([t, x, y, phi,
		0.0, 0.0,
		0.0, 0.0,
		0.0, 0.0,
		0.0, 0.0,
		0.0, 0.0 ])
	counter += 1

def callbackToGui(msg):
	global counter
	global data
	global start_time
	global previous_t

	#try:
	#	(x, y, phi) = get_pose()
	#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	#	return
	if phi == None:
		return

	if start_time == None:
		start_time = rospy.Time.now() #time.time()
		t = 0.0
	else:
		t = (rospy.Time.now() - start_time).to_sec() #time.time() - start_time
		if t - previous_t < log_period:
			return
		previous_t = t

	print (rospy.Time.now())

	R = np.array([
	 [np.cos(phi), -np.sin(phi)],
	 [np.sin(phi),  np.cos(phi)]])
	translation = np.array([[x], [y]])

	p_ref_local = np.array([[0.0], [msg.reference_point.y]])
	p_ref_global = np.matmul(R, p_ref_local) + translation

	v_ref_nominal_local = np.array([[msg.reference_point_nominal_velocity.x],
		[msg.reference_point_nominal_velocity.y]])
	v_ref_corr_local = np.array([[msg.reference_point_velocity_solution.x],
		[msg.reference_point_velocity_solution.y]])

	v_ref_nominal_global = np.matmul(R, v_ref_nominal_local)
	v_ref_corr_global = np.matmul(R, v_ref_corr_local)

	#ped_local = np.array([[msg.moving_objects[0].center.x], [msg.moving_objects[0].center.y]])
	#ped_local

	if counter >= n_max:
		print ('Longer than expected')
		return
	data[counter, :] = np.array([t, x, y, phi,
		msg.nominal_command.linear, msg.nominal_command.angular,
		msg.corrected_command.linear, msg.corrected_command.angular,
		p_ref_global[0, 0], p_ref_global[1, 0],
		v_ref_nominal_global[0, 0], v_ref_nominal_global[1, 0],
		v_ref_corr_global[0, 0], v_ref_corr_global[1, 0] ])
	counter += 1

def main():
	#global tf_listener
	rospy.init_node('process_rosbag_messages_node')
	#tf_listener = tf.TransformListener()
	#rospy.Subscriber("rds_to_gui", ToGui, callbackToGui)
	#zero_pose()
	rospy.Subscriber('poseupdate', PoseWithCovarianceStamped, callbackPoseUpdate)
	print ('Ready ...')
	rospy.spin()

if __name__ == '__main__':
	main()