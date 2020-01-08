#ifndef TWO_LIDAR_TO_LRF_NODE_HPP
#define TWO_LIDAR_TO_LRF_NODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>

#include <vector>

struct TwoLidarToLRFNode
{
	TwoLidarToLRFNode(ros::NodeHandle* n);

	void frontVelodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan);
	void rearVelodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan);

	ros::Publisher lrf_publisher;
	sensor_msgs::LaserScan lrf_scan;
	bool received_one_velodyne_scan;
	ros::Subscriber front_velodyne_subscriber;
	ros::Subscriber rear_velodyne_subscriber;

	const float front_lidar_y_coordinate;
	const float front_lidar_mount_height;
	const float front_lidar_tilt_angle;
	const float rear_lidar_y_coordinate;
	const float rear_lidar_mount_height;
	const float rear_lidar_tilt_angle;
	const float lrf_y_coordinate;
	const int lrf_number_of_bins;
	const float lrf_angle_step;

	struct Point3D
	{
		Point3D(float x, float y, float z) : x(x), y(y), z(z) { }
		float x,y,z;
	};

	struct PointCloud3D : public velodyne_rawdata::DataContainerBase
	{
		virtual void addPoint(const float& x, const float& y, const float& z,
			const uint16_t& ring, const uint16_t& azimuth, const float& distance,
			const float& intensity)
		{
			points.push_back(Point3D(x, y, z));
		}

		std::vector<Point3D> points;
	};
};

#endif