#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include <rds/rds_wrap.hpp>
#include <rds/geometry.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct QoloCollisionPointGenerator : public CollisionPointGenerator<sensor_msgs::LaserScan::ConstPtr>
{
	QoloCollisionPointGenerator();

	void obstacleMessageCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_sensor_msg);

	Geometry2D::Vec2 lrf_location;
	float lrf_orientation;
	float angle_cutoff_from_forward_direction;
	float range_cutoff_lower;

private:
	void defineQoloShape();
};

struct RDSNode
{
	RDSNode(ros::NodeHandle* n);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

	QoloCollisionPointGenerator qolo_cpg;
	ros::Subscriber laserscan_subscriber;
	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;
};

#endif