import rospy
from rds_network_ros.srv import * #VelocityCommandCorrectionRDS

def rds_service():
   rospy.wait_for_service('rds_velocity_command_correction')
   # try:
   RDS = rospy.ServiceProxy('rds_velocity_command_correction',VelocityCommandCorrectionRDS)

   request = VelocityCommandCorrectionRDSRequest()

   request.nominal_command.linear = 1.0
   request.nominal_command.angular = 0.5
   request.capsule_center_front_y = 0.18
   request.capsule_center_rear_y = -0.5
   request.capsule_radius = 0.45
   request.reference_point_y = 0.18
   request.rds_tau = 1.5
   request.rds_delta = 0.05
   request.vel_lim_linear_min = -0.5
   request.vel_lim_linear_max = 1.5
   request.vel_lim_angular_abs_max = 1.0
   request.vel_linear_at_angular_abs_max = 0.2
   request.acc_limit_linear_abs_max = 0.5
   request.acc_limit_angular_abs_max = 0.5
   request.dt = 0.01

   # shall rds consider lrf measurements?
   request.lrf_point_obstacles = True

   # execute the baseline method (ORCA-like) instead of RDS?
   request.ORCA_implementation = False

   response = RDS(request)
   
   Output_V = round(response.corrected_command.linear,4)
   Output_W = round(response.corrected_command.angular,4)
   print (Output_V, Output_W, response.call_counter)


def main():
   rospy.init_node('rds_client_ros_node')
   while not rospy.is_shutdown():
      rds_service()

if __name__ == '__main__':
   main()