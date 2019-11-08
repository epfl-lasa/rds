// sudo apt-get install ros-kinetic-velodyne

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>
#include <sensor_msgs/LaserScan.h>

#include <velodyne_pointcloud/calibration.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_pointcloud/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <vector>

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
		return;
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

void convert_velodyne_scan_to_lrf_laserscan_and_publish(velodyne_msgs::VelodyneScan::ConstPtr v_scan)
{

	//pcl::PointCloud<pcl::PointXYZI> cloud_3d;
	//pcl::PointXYZI point;
	velodyne_rawdata::RawData data;
	MyPoint point;

	//cloud_3d.clear();

	for(int i=0;i<v_scan->packets.size();i++)
	{
		MyPointCloud v_point_cloud;

		data.setParameters(0.9,130.0,0.0,6.28);
		data.setupOffline("file adresss",130.0,0.9);
		data.unpack(v_scan->packets[i],v_point_cloud);

		for(int j=0;j<v_point_cloud.size();j++)
		{
			point.x=v_point_cloud.points[j].x;
			point.y=v_point_cloud.points[j].y;
			point.z=v_point_cloud.points[j].z;
			point.i=v_point_cloud.points[j].i;
			//cloud_3d.push_back(point);
		}
	}

	sensor_msgs::LaserScan lrf_scan;
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