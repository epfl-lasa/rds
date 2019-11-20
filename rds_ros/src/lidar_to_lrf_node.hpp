#ifndef LIDAR_TO_LRF_NODE_HPP
#define LIDAR_TO_LRF_NODE_HPP

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>

#include <vector>

struct LidarToLRFNode
{
	LidarToLRFNode(ros::NodeHandle* n);

	void velodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan);

	ros::Publisher lrf_publisher;
	ros::Subscriber velodyne_subscriber;

	const float lidar_mount_height;
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