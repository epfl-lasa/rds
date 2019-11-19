#include "qolo_rds_node.hpp"
#include "differential_drive_kinematics.hpp"
#include "rds_wrap.hpp"

QoloRDSNode::QoloRDSNode(ros::NodeHandle* n)
	: corrected_command_publisher(n->advertise<std_msgs::Float32MultiArray>("rds_command", 10))
	, publisher_for_gui(n->advertise<std_msgs::Float32MultiArray>("rds_to_gui", 10))
	, nominal_command_subscriber(n->subscribe("nominal_velocity_command", 1000,
		&QoloRDSNode::nominalCommandCallbackToPublishCorrectedCommand, this))
	, lrf_subscriber(n->subscribe("/sick_laser_front/cropped_scan", 1000,
		&QoloCollisionPointGenerator::LRFCallbackToUpdateObstacleCirclesAndVelocities,
		&qolo_collision_point_generator))
{
	ros::spin();
}

void QoloRDSNode::nominalCommandCallbackToPublishCorrectedCommand(const std_msgs::Float32MultiArray::ConstPtr& nominal_command_msg);
{
	RDS::VelocityCommand nominal_command(nominal_command_msg->data[0], nominal_command_msg->data[1]);

	RDSWrap rds_wrap();
}
	
