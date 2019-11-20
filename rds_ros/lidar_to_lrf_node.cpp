// sudo apt-get install ros-kinetic-velodyne

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>
#include <sensor_msgs/LaserScan.h>

#include <ros/package.h> // to find the path to the calibration file in the velodyne package

//#include <velodyne_pointcloud/calibration.h>
//#include <velodyne_msgs/VelodynePacket.h>
//#include <velodyne_pointcloud/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <string>
#include <cmath>

struct MyPoint
{
	float x,y,z,i;
};

class MyPointCloud : public velodyne_rawdata::DataContainerBase
{
public:
	std::vector<MyPoint> points;
	int size()
	{
		return points.size();
	}
	virtual void addPoint(const float& x, const float& y, const float& z,
		const uint16_t& ring, const uint16_t& azimuth, const float& distance,
		const float& intensity)
	{
		MyPoint p;
		p.x = x;
		p.y = y;
		p.z = z;
		p.i = intensity;
		points.push_back(p);
	}
};


/*    #include "velodyne_pointcloud/calibration.h"
    #include "velodyne_msgs/VelodynePacket.h"
    #include "velodyne_msgs/VelodyneScan.h"
    #include "velodyne_pointcloud/rawdata.h"
    #include "velodyne_pointcloud/point_types.h"
    #include "velodyne_pointcloud/CloudNodeConfig.h"
    #include "velodyne_pointcloud/point_types.h"
    #include  "velodyne_pointcloud/calibration.h"
    #include <pcl_conversions/pcl_conversions.h>*/

ros::Publisher* lrf_msg_publisher = 0;

float positive_angle(float angle)
{
	while (angle < 0.f)
		angle += 2.f*2*std::asin(1.0);
	return angle;
}


void convert_velodyne_scan_to_lrf_laserscan_and_publish(velodyne_msgs::VelodyneScan::ConstPtr v_scan)
{

	std::string velodyne_pointcloud_package_path = ros::package::getPath("velodyne_pointcloud");
	std::string calibration_file_path = velodyne_pointcloud_package_path + "params/VLP16db.yaml";

	float min_range = 0.4f; // velodyne scanner's minimum range in [m]
	float max_range = 50.f; // velodyne scanner's maximum range in [m]
	float view_angle_start = 0.f;
	float view_angle_width = 2.f*2*std::asin(1.0); // 2*pi
	float z_lower_bound = -0.2f;
	int n_angular_bins = 900;
	float angular_bin_width = 2.f*2*std::asin(1.0)/n_angular_bins;

	velodyne_rawdata::RawData data;

	sensor_msgs::LaserScan lrf_scan;
	for (int i = 0; i < n_angular_bins; i++)
		lrf_scan.ranges.push_back(max_range);

	for (int i = 0; i < v_scan->packets.size(); i++)
	{
		MyPointCloud v_point_cloud;

		data.setParameters(min_range, max_range, view_angle_start, view_angle_width);
		data.setupOffline(calibration_file_path, max_range, min_range); // setup(nodehandle) maybe better
		data.unpack(v_scan->packets[i], v_point_cloud);

		for(int j=0;j<v_point_cloud.size();j++)
		{
			if (z_lower_bound < v_point_cloud.points[j].z)
			{
				float phi = std::atan2(v_point_cloud.points[j].y, v_point_cloud.points[j].x);
				float r = std::sqrt(v_point_cloud.points[j].x*v_point_cloud.points[j].x +
					v_point_cloud.points[j].y*v_point_cloud.points[j].y);
				int lrf_bin_index = positive_angle(phi)/angular_bin_width;
				if ((lrf_bin_index < n_angular_bins) && (lrf_scan.ranges[lrf_bin_index] > r))
					lrf_scan.ranges[lrf_bin_index] = r;
			}
			//v_point_cloud.points[j].z;
			//v_point_cloud.points[j].i;
		}
	}
	lrf_msg_publisher->publish(lrf_scan);
}

int main(int argc, char** argv)
{
	int dummy_argc = 1;
	char arg1[] = "gui_node";
	char* dummy_argv[] = {arg1};
	ros::init(dummy_argc, dummy_argv, "gui_node");
	ros::NodeHandle n;

	lrf_msg_publisher = new ros::Publisher(n.advertise<sensor_msgs::LaserScan>("lrf_scan", 10));
	ros::Subscriber lidar_subscriber(n.subscribe("here_goes_the_velodyne_topic_name",
				1000, convert_velodyne_scan_to_lrf_laserscan_and_publish));

	ros::spin();

	delete lrf_msg_publisher;
	lrf_msg_publisher = 0;
}