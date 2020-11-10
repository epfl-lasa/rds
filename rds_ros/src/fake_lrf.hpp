#ifndef FAKE_LRF_HPP
#define FAKE_LRF_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>

struct FakeLRF
{
	FakeLRF(ros::NodeHandle* n);

	const std::chrono::milliseconds sampling_time = std::chrono::milliseconds(50);
	ros::Publisher publisher_lrf_front, publisher_lrf_rear;
};

#endif