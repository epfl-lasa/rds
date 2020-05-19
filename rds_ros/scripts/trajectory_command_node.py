#! /usr/bin/env python
#########  ROS version of Trajectory Tracking with safety ##########
##### Author: Diego F. Paez G. & David Gonon
##### Preliminar version: David Gonon
##### Data: 2020/05/18

import time
import math
import rospy
# from rds_network_ros.srv import * #VelocityCommandCorrectionRDS
import tf
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension 
import numpy as np
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt
 
trajectory_xyt = np.array([
   [ 0.0, 0.0,  0.0], # accelerating
   [ 1.0, 0.0,  5.0],
   [ 2.0, 0.0, 10.0] # decelerating
   ])
# trajectory_xyt = np.array([
#    [ 0.0, 0.0,  0.0], # accelerating
#    [ 0.0, 1.0,  4.0],
#    [ 1.0, 5.0, 10.0],
#    [ 5.0, 6.0, 16.0],
#    [ 6.0, 6.0, 20.0] # decelerating
#    ])
# plot_spline_curve(trajectory_spline, np.arange(-5.0, 25.0, 0.15), trajectory_xyt)

tf_listener = None
command_publisher = None
t_lost_tf = -1.0
previous_command_linear = None
previous_command_angular = None
data_remote = Float32MultiArray()

def create_spline_curve(XYT):
   return [
      UnivariateSpline(XYT[:, 2], XYT[:, 0], k=2, s=0, ext=3),
      UnivariateSpline(XYT[:, 2], XYT[:, 1], k=2, s=0, ext=3)]

def plot_spline_curve(spline_curve, time_vector, data_xyt):
   v_x = spline_curve[0].derivative()
   v_y = spline_curve[1].derivative()
   tau = 4.0
   fig = plt.figure()
   ax = fig.add_subplot(111)
   for t in time_vector:
      tail_x = spline_curve[0](t)
      tail_y = spline_curve[1](t)
      head_x = tail_x + tau*v_x(t)
      head_y = tail_y + tau*v_y(t)
      ax.plot([tail_x, head_x], [tail_y, head_y], 'g')
   ax.plot(spline_curve[0](time_vector), spline_curve[1](time_vector), 'ro-')
   ax.plot(data_xyt[:,0], data_xyt[:,1], 'ko')
   ax.set_aspect('equal')
   ax.set(xlabel='x', ylabel='y')
   plt.show()
   fig, axs = plt.subplots(2)
   axs[0].plot(time_vector, spline_curve[0](time_vector))
   axs[0].set(xlabel='t', ylabel='x')
   axs[1].plot(time_vector, spline_curve[1](time_vector))
   axs[1].set(xlabel='t', ylabel='y')
   plt.show()


trajectory_spline = create_spline_curve(trajectory_xyt)
trajectory_spline_derivative = [trajectory_spline[0].derivative(), trajectory_spline[1].derivative()]

def get_pose():
   global tf_listener
   (trans, rot) = tf_listener.lookupTransform('/tf_rds', '/tf_qolo_world', rospy.Time(0))
   rpy = tf.transformations.euler_from_quaternion(rot)
   return (trans[0], trans[1], rpy[2])

def feedforward_feedback_controller(t):
   global t_lost_tf
   global previous_command_linear
   global previous_command_angular
   global trajectory_spline
   global trajectory_spline_derivative
   try:
      (x, y, phi) = get_pose()
      t_lost_tf = -1.0

      R = np.array([
         [np.cos(phi), -np.sin(phi)],
         [np.sin(phi),  np.cos(phi)]])
      translation = np.array([[x], [y]])

      p_ref_local = np.array([[0.0], [0.3]])
      p_ref_global = np.matmul(R, p_ref_local) + translation

      feedforward_velocity = np.array([[trajectory_spline_derivative[0](t)],
         [trajectory_spline_derivative[1](t)]])
      position_setpoint = np.array([[trajectory_spline[0](t)],
         [trajectory_spline[1](t)]])
      feedback_velocity = 0.25*(position_setpoint - p_ref_global)
      v_command_p_ref_global = feedforward_velocity + feedback_velocity
      v_command_p_ref_local = np.matmul(np.transpose(R), v_command_p_ref_global)

      J_p_ref_inv = np.array([
         [p_ref_local[0,0]/p_ref_local[1,0], 1.0],
         [-1.0/p_ref_local[1,0], 0.0]])
      command_linear_angular = np.matmul(J_p_ref_inv, v_command_p_ref_local)
      command_linear = command_linear_angular[0]
      command_angular = command_linear_angular[1]
      previous_command_linear = command_linear
      previous_command_angular = command_angular
      return (command_linear, command_angular)
   except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print ("Exception during tf lookup ...")
      if t_lost_tf == -1.0:
         t_lost_tf = t
      decay_factor = 1.0 - np.min([2.0, t - t_lost_tf])/2.0
      if type(previous_command_linear) == type(None):
         return (0.0, 0.0)
      return (decay_factor*previous_command_linear,
         decay_factor*previous_command_angular)

def publish_command(command_linear, command_angular, t):
   global data_remote, command_publisher
   
   Ctime = round(time.clock(),4)
   data_remote.data = [Ctime,command_linear,command_angular]
   # msg.data = np.array([
   #    rospy.get_rostime(),
   #    command_linear,
   #    command_angular])
   command_publisher.publish(data_remote)
   rospy.loginfo(data_remote)

# def rds_service(t):
#    # print "Waiting for RDS Service"

#    rospy.wait_for_service('rds_velocity_command_correction')
#    # try:
#    RDS = rospy.ServiceProxy('rds_velocity_command_correction',
#       VelocityCommandCorrectionRDS)

#    request = VelocityCommandCorrectionRDSRequest()

#    (Trajectory_V, Trajectory_W) = feedforward_feedback_controller(t)

#    request.nominal_command.linear = Trajectory_V;
#    request.nominal_command.angular = Trajectory_W;

#    response = RDS(request)
#    Output_V = round(response.corrected_command.linear, 4)
#    Output_W = round(response.corrected_command.angular, 4)

#    publish_command(Output_V, Output_W, t)

def trajectory_service(t):
   # print "Waiting for RDS Service"
   try:
      (Trajectory_V, Trajectory_W) = feedforward_feedback_controller(t)
      publish_command(Trajectory_V, Trajectory_W, t)
   except:
        publish_command(0., 0., 0.)
        print ('Trajectory Error Stopping [0 0]')


def main():
   global tf_listener, command_publisher, data_remote
   rospy.init_node('qolo_trajectory_tracking')
   tf_listener = tf.TransformListener()
   command_publisher = rospy.Publisher('qolo/remote_commands',Float32MultiArray, queue_size=1)

   data_remote.layout.dim.append(MultiArrayDimension())
   data_remote.layout.dim[0].label = 'Trajectory Commands [V, W]'
   data_remote.layout.dim[0].size = 3
   data_remote.data = [0]*3

   start_time = time.time()
   while not rospy.is_shutdown():
      trajectory_service(time.time() - start_time)

if __name__ == '__main__':
   main()