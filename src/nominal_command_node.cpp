#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char** argv)
{
	int dummy_argc = 1;
	char arg1[] = "nominal_command_node";
	char* dummy_argv[] = {arg1};
	ros::init(dummy_argc, dummy_argv, "nominal_command_node");
	ros::NodeHandle n;
	ros::Publisher nominal_command_publisher = n.advertise<std_msgs::Float32MultiArray>(
		"nominal_velocity_command", 10);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		std_msgs::Float32MultiArray nominal_command_msg;
		nominal_command_msg.data.push_back(0.5); // linear
		nominal_command_msg.data.push_back(0.25); // angular

		nominal_command_publisher.publish(nominal_command_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

}