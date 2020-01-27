#ifndef RDS_HOLONOMIC_ROS_NODE_HPP
#define RDS_HOLONOMIC_ROS_NODE_HPP

#include <rds/rds_wrap.hpp>
#include <rds/geometry.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct MyCollisionPointGenerator : public CollisionPointGenerator<sensor_msgs::LaserScan::ConstPtr>
{
	MyCollisionPointGenerator();

	void obstacleMessageCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_sensor_msg);

	Geometry2D::Vec2 front_lrf_location;
	float front_lrf_orientation;
	float front_angle_cutoff_from_forward_direction;
	float front_range_cutoff_lower;

private:
	void defineMyShape();
};

struct RDSHolonomicNode
{
	RDSHolonomicNode(ros::NodeHandle* n);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

	MyCollisionPointGenerator my_cpg;
	ros::Subscriber laserscan_subscriber;
	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;
};

#endif