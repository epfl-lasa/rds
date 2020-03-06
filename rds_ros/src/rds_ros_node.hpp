#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include "aggregate_two_lrf.hpp"
#include <rds/rds_wrap.hpp>
#include <rds/geometry.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct QoloCollisionPointGenerator : public CollisionPointGenerator<sensor_msgs::LaserScan::ConstPtr>
{
	QoloCollisionPointGenerator(const AggregatorTwoLRF& aggregator_two_lrf);

	void obstacleMessageCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_sensor_msg) { }
	//void frontLRFMessageCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_sensor_msg);

	const CollisionPointGenerator<sensor_msgs::LaserScan::ConstPtr>& generateCollisionPoints();

	AggregatorTwoLRF aggregator_two_lrf;

private:
	void defineQoloShape();
};

struct RDSNode
{
	RDSNode(ros::NodeHandle* n, const AggregatorTwoLRF& aggregator_two_lrf);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

	QoloCollisionPointGenerator qolo_cpg;
	ros::Subscriber subscriber_lrf_front;
	ros::Subscriber subscriber_lrf_rear;
	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;
};

#endif