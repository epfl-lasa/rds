#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

qolo_message = ''
qolo_message_at_first_camera_message = ''
qolo_message_at_camera_message = ''

def qolo_callback(msg):
	global qolo_message
	qolo_message = msg.data

def camera_callback(msg):
	global qolo_message_at_first_camera_message
	global qolo_message_at_camera_message
	if qolo_message_at_first_camera_message == '':
		#if qolo_message != '':
		qolo_message_at_first_camera_message = qolo_message
	qolo_message_at_camera_message = qolo_message


    
def main():
	rospy.init_node('message_capturer')
	rospy.Subscriber("qolo", String, qolo_callback)
	rospy.Subscriber("camera/color/image_raw", Image, camera_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
	print "qolo_message_at_first_camera_message:"
	print qolo_message_at_first_camera_message
	print "qolo_message_at_last_camera_message:"
	print qolo_message_at_camera_message