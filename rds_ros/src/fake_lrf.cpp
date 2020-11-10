#include "fake_lrf.hpp"

#include <thread>
#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

FakeLRF::FakeLRF(ros::NodeHandle* n)
	: publisher_lrf_front(n->advertise<sensor_msgs::LaserScan>("front_lidar/scan", 1))
	, publisher_lrf_rear(n->advertise<sensor_msgs::LaserScan>("rear_lidar/scan", 1))
{
	sensor_msgs::LaserScan msg_front, msg_rear;
	msg_front.header.seq = 0;
	msg_front.header.frame_id = "tf_rds";
	msg_front.angle_min = 0.0;
	unsigned int n_ranges = 900;
	msg_front.angle_increment = 2.0*M_PI/n_ranges;
	msg_front.angle_max = (n_ranges - 1)*msg_front.angle_increment;
	msg_front.time_increment = 0.0;
	msg_front.scan_time = sampling_time.count()*0.001;
	msg_front.range_min = 0.0;
	msg_front.range_max = 1000000.0;
	msg_front.ranges = std::vector<float>(n_ranges, 0.0);
	msg_front.intensities.resize(0);
	msg_rear = msg_front;

	float t_shaker_1 = 0.0;
	float t_shaker_2 = 0.0;
	float period_1 = 1.0;
	float period_2 = 1.65;

	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	while (ros::ok())
	{
		msg_front.ranges[112] = 1.25 + 0.1*std::sin(t_shaker_1/period_1*2.0*M_PI);
		msg_rear.ranges[788] = 1.75 + 0.05*std::sin(t_shaker_2/period_2*2.0*M_PI);
		msg_front.header.stamp = msg_rear.header.stamp = ros::Time::now();
		msg_front.header.seq++;
		msg_rear.header.seq++;

		publisher_lrf_front.publish(msg_front);
		publisher_lrf_rear.publish(msg_rear);

		t_shaker_1 += sampling_time.count()*0.001;
		t_shaker_2 += sampling_time.count()*0.001;
		if (t_shaker_1 > period_1)
			t_shaker_1 -= period_1;
		if (t_shaker_2 > period_2)
			t_shaker_2 -= period_2;

		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(sampling_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "fake_lrf_node");
	ros::NodeHandle n;
	FakeLRF fake_lrf_node(&n);
	return 0;
}