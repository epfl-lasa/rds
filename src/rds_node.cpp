#include "geometry.hpp"
#include "rds.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>
#include <cstring>

class LRFMeasurement
{
public:
	std::vector<float> range_scan;
	float angular_step_in_rad;
	float scan_start_angle_in_rad;
	float min_range;
	float max_range;
};

LRFMeasurement lrf_measurement;

class CircleMeasurement
{
public:
	std::vector<AdditionalPrimitives2D::Circle> obstacles;
	std::vector<Geometry2D::Vec2> obstacle_velocities;
};

CircleMeasurement circle_measurement;

RDS::VelocityCommand previous_command;

ros::Publisher* rds_command_publisher = 0;
ros::Publisher* rds_gui_msg_publisher = 0;

void transform_command_using_circles(const std_msgs::Float32MultiArray::ConstPtr& nominal_command_msg)
{
	//ROS_INFO("Previous command: (%f, %f).", previous_command.linear, previous_command.angular);

	RDS::CircleCollisionPointGenerator ccpg(circle_measurement.obstacles,
		circle_measurement.obstacle_velocities);

	RDS::VelocityCommand nominal_command(nominal_command_msg->data[0], nominal_command_msg->data[1]);
	RDS::CommandGenerator cg(nominal_command, previous_command, &ccpg);
	RDS::VelocityCommand transformed_command(cg.command);
	previous_command = transformed_command;

	std_msgs::Float32MultiArray transformed_command_msg;
	transformed_command_msg.data.push_back(transformed_command.linear);
	transformed_command_msg.data.push_back(transformed_command.angular);
	rds_command_publisher->publish(transformed_command_msg);

	std_msgs::Float32MultiArray rds_gui_msg;
	rds_gui_msg.data.push_back(cg.solution_distance_minimization.x);
	rds_gui_msg.data.push_back(cg.solution_distance_minimization.y);
	const std::vector<Geometry2D::HalfPlane2>& constraints(cg.constraint_generator.getConstraints());
	for (std::vector<Geometry2D::HalfPlane2>::size_type i = 0; i != constraints.size(); i++)
	{
		rds_gui_msg.data.push_back(constraints[i].getNormal().x);
		rds_gui_msg.data.push_back(constraints[i].getNormal().y);
		rds_gui_msg.data.push_back(constraints[i].getOffset());
	}
	rds_gui_msg_publisher->publish(rds_gui_msg);
}

void transform_command_using_lrf(const std_msgs::Float32MultiArray::ConstPtr& nominal_command_msg)
{
	RDS::LRFCollisionPointGenerator ccpg(lrf_measurement.range_scan, lrf_measurement.angular_step_in_rad,
		lrf_measurement.scan_start_angle_in_rad, lrf_measurement.min_range, lrf_measurement.max_range);

	RDS::VelocityCommand nominal_command(nominal_command_msg->data[0], nominal_command_msg->data[1]);
	RDS::CommandGenerator cg(nominal_command, previous_command, &ccpg);
	RDS::VelocityCommand transformed_command(cg.command);
	previous_command = transformed_command;

	std_msgs::Float32MultiArray transformed_command_msg;
	transformed_command_msg.data.push_back(transformed_command.linear);
	transformed_command_msg.data.push_back(transformed_command.angular);
	rds_command_publisher->publish(transformed_command_msg);

	std_msgs::Float32MultiArray rds_gui_msg;
	rds_gui_msg.data.push_back(cg.solution_distance_minimization.x);
	rds_gui_msg.data.push_back(cg.solution_distance_minimization.y);
	const std::vector<Geometry2D::HalfPlane2>& constraints(cg.constraint_generator.getConstraints());
	for (std::vector<Geometry2D::HalfPlane2>::size_type i = 0; i != constraints.size(); i++)
	{
		rds_gui_msg.data.push_back(constraints[i].getNormal().x);
		rds_gui_msg.data.push_back(constraints[i].getNormal().y);
		rds_gui_msg.data.push_back(constraints[i].getOffset());
	}
	rds_gui_msg_publisher->publish(rds_gui_msg);
}

void update_circle_measurement(const std_msgs::Float32MultiArray::ConstPtr& circles_msg)
{
	circle_measurement.obstacles.resize(0);
	circle_measurement.obstacle_velocities.resize(0);

	int n_circles = circles_msg->data.size()/5;
	for (int i = 0; i < n_circles; i++)
	{
		circle_measurement.obstacles.push_back(AdditionalPrimitives2D::Circle(
			Geometry2D::Vec2(circles_msg->data[i*3], circles_msg->data[i*3+1]), circles_msg->data[i*3+2]));
		circle_measurement.obstacle_velocities.push_back(
			Geometry2D::Vec2(circles_msg->data[i*3+3], circles_msg->data[i*3+4]));
	}
}

void update_laserscan_measurement(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg)
{
	lrf_measurement.range_scan = laserscan_msg->ranges;
	lrf_measurement.angular_step_in_rad = laserscan_msg->angle_increment;
	lrf_measurement.scan_start_angle_in_rad = laserscan_msg->angle_min;
	lrf_measurement.min_range = laserscan_msg->range_min;
	lrf_measurement.max_range = laserscan_msg->range_max;
}

int main(int argc, char** argv)
{
	int dummy_argc = 1;
	char arg1[] = "rds_node";
	char* dummy_argv[] = {arg1};
	ros::init(dummy_argc, dummy_argv, "rds_node");
	ros::NodeHandle n;
	ros::Subscriber obstacle_measurement_subscriber;
	ros::Subscriber nominal_command_subscriber;
	const char nominal_command_topic_name[] = "nominal_velocity_command";

	if (argc > 1)
	{
		if (0 == std::strcmp(argv[1], "circle"))
		{
			const char circle_topic_name[] = "circle_obstacles";
			obstacle_measurement_subscriber = n.subscribe(circle_topic_name,
				1000, update_circle_measurement);
			nominal_command_subscriber = n.subscribe(nominal_command_topic_name,
				1000, transform_command_using_circles);
		}
		else if (0 == std::strcmp(argv[1], "lrf"))
		{
			const char laserscan_topic_name[] = "/sick_laser_front/cropped_scan";
			obstacle_measurement_subscriber = n.subscribe(laserscan_topic_name,
				1000, update_laserscan_measurement);
			nominal_command_subscriber = n.subscribe(nominal_command_topic_name,
				1000, transform_command_using_lrf);
		}
	}
	else
	{
		ROS_ERROR("Missing one command line argument (can be circle or lrf).");
		return 0;
	}

	rds_command_publisher = new ros::Publisher(
		n.advertise<std_msgs::Float32MultiArray>("rds_command", 10));

	rds_gui_msg_publisher = new ros::Publisher(
		n.advertise<std_msgs::Float32MultiArray>("rds_to_gui", 10));

	ros::spin();

	delete rds_gui_msg_publisher;
	rds_gui_msg_publisher = 0;
	delete rds_command_publisher;
	rds_command_publisher = 0;
	return 0;
}
