import rospy
from rds_network_ros.srv import * #VelocityCommandCorrectionRDS
import time

# Setting for the RDS service
max_linear = 1.0;
min_linear = -1.0;
absolute_angular_at_min_linear = 0.;
absolute_angular_at_max_linear = 0.;
absolute_angular_at_zero_linear = 1.0;
linear_acceleration_limit = 4.0
angular_acceleration_limit = 2.0
feasible = 0
Output_V = 0.;
Output_W = 0.;
last_v = 0.;
last_w = 0.;
cycle=0.
y_coordinate_of_reference_point_for_command_limits = 0.5;
weight_scaling_of_reference_point_for_command_limits = 0.;
tau = 2.;
delta = 0.10;
clearance_from_axle_of_final_reference_point = 0.15;

User_V = 1.0
User_W = 1.0

def rds_service():
   global User_V, User_W, Output_V, Output_W, last_v, last_w, cycle, feasible
   # print "Waiting for RDS Service"

   rospy.wait_for_service('rds_velocity_command_correction')
   # try:
   RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

   request = VelocityCommandCorrectionRDSRequest()

   request.nominal_command.linear = User_V;
   request.nominal_command.angular = User_W;

   request.velocity_limits.max_linear = max_linear;
   request.velocity_limits.min_linear = min_linear;
   request.velocity_limits.abs_angular_at_min_linear = absolute_angular_at_min_linear;
   request.velocity_limits.abs_angular_at_max_linear = absolute_angular_at_max_linear;
   request.velocity_limits.abs_angular_at_zero_linear = absolute_angular_at_zero_linear;
   request.abs_linear_acceleration_limit = linear_acceleration_limit;
   request.abs_angular_acceleration_limit = angular_acceleration_limit;

   request.y_coordinate_of_reference_point_for_command_limits = y_coordinate_of_reference_point_for_command_limits;
   request.weight_scaling_of_reference_point_for_command_limits = weight_scaling_of_reference_point_for_command_limits;
   request.clearance_from_axle_of_final_reference_point = clearance_from_axle_of_final_reference_point;
   request.delta = delta;
   request.tau = tau;
   request.y_coordinate_of_reference_biasing_point = 1.;
   request.weight_of_reference_biasing_point = 0.;

   request.last_actual_command.linear = last_v;
   request.last_actual_command.angular = last_w;

   if cycle==0:
       delta_time = 0.005;
   else:
       delta_time = time.clock() - cycle;

   request.command_cycle_time = delta_time

   response = RDS(request)
   Output_V = round(response.corrected_command.linear,4)
   Output_W = round(response.corrected_command.angular,4)
   feasible = response.feasible

   last_v = Output_V
   last_w = Output_W
   cycle = time.clock()


def main():
   rospy.init_node('rds_client_ros_node')
   while not rospy.is_shutdown():
      rds_service()

if __name__ == '__main__':
   main()