#! /usr/bin/env python
#########  Trajectory Logging ##########
##### Author: David Gonon
##### Date: 2020/05/21

import time
import math
import rospy
# from rds_network_ros.srv import * #VelocityCommandCorrectionRDS
import tf
import numpy as np
import matplotlib.pyplot as plt

tf_listener = None

trajectory_slam_xyphit = np.zeros([10000,4])
trajectory_slam_counter = 0

def get_pose():
   global tf_listener
   (trans, rot) = tf_listener.lookupTransform('/tf_qolo_world', '/tf_rds', rospy.Time(0))
   rpy = tf.transformations.euler_from_quaternion(rot)
   #print ("phi=", rpy[2])
   return (trans[0], trans[1], rpy[2])

def lookup():
   global trajectory_slam_counter
   global trajectory_slam_xyphit

   try:
      (x, y, phi) = get_pose()

      if trajectory_slam_counter < trajectory_slam_xyphit.shape[0]:
         trajectory_slam_xyphit[trajectory_slam_counter, :] = np.array(
            [[x, y, phi, time.time()]])
         trajectory_slam_counter += 1
   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print ("Exception during tf lookup ...")

def main():
   global tf_listener
   rospy.init_node('qolo_trajectory_logging')
   tf_listener = tf.TransformListener()
   rate = rospy.Rate(20)
   while not rospy.is_shutdown():
      lookup()
      try:
         rate.sleep()
      except:
         return

if __name__ == '__main__':
   main()

   plt.plot(trajectory_slam_xyphit[0:trajectory_slam_counter,0], trajectory_slam_xyphit[0:trajectory_slam_counter,1])
   plt.show()