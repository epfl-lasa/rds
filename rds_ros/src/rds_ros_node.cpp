#include "rds_ros_node.hpp"

#include <rds/differential_drive_kinematics.hpp>
#include <rds/collision_point.hpp>

#include <rds_network_ros/ToGui.h>
#include <rds_network_ros/HalfPlane2D.h>
#include <rds_network_ros/Point2D.h>
#include <rds_network_ros/Circle.h>

#define _USE_MATH_DEFINES
#include <cmath>

QoloCollisionPointGenerator::QoloCollisionPointGenerator()
	: front_lrf_location(Geometry2D::Vec2(0.f, 0.056f))
	, front_lrf_orientation(M_PI/2.f)
	, front_angle_cutoff_from_forward_direction(3.f*M_PI/4.f)
	, front_range_cutoff_lower(0.05f)
	, rear_lrf_location(0.f, -0.517)
{
	defineQoloShape();
}

void QoloCollisionPointGenerator::defineQoloShape()
{
	// create a shape containing the two LIDAR blind zones (it contains Qolo as well)
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(front_lrf_location, 0.4f));
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(rear_lrf_location, 0.3f));
	return;

	// create a Qolo-like shape using 8 circles
	float scale = 0.3f*2.f/357.0f;//*2.f;

	Geometry2D::Vec2 position;
	float radius;

	position.x = 208.391*scale;
	position.y = 95.028*scale;
	radius = 77.637*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -208.391*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 153.585*scale;
	position.y = 4.019*scale;
	radius = 136.843*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -153.585*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 57.833*scale;
	position.y = -105.522*scale;
	radius = 227.437*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));
	position.x = -57.833*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 0.0;
	position.y = -284.338*scale;
	radius = 230.051*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));

	position.x = 0.0;
	position.y = -383.821*scale;
	radius = 174.055*scale;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(position, radius));
}

float angleToPlus270Minus90(float angle)
{
	while (angle < -M_PI/2.f)
		angle += 2.f*M_PI;
	while (angle > 3.f*M_PI/2.f)
		angle -= 2.f*M_PI;
	return angle;
}

void QoloCollisionPointGenerator::obstacleMessageCallback(const visualization_msgs::MarkerArray::ConstPtr& circle_msg)
{
	obstacle_circles.resize(0);//lrf_msg->ranges.size());
	obstacle_velocities.resize(0);//lrf_msg->ranges.size());
	for (std::vector<float>::size_type i = 0; i != circle_msg->markers.size(); i++)
	{
		//obstacle_velocities[i] = Geometry2D::Vec2(0.f, 0.f);
		Geometry2D::Vec2 center(front_lrf_location + Geometry2D::Vec2(circle_msg->markers[i].pose.position.x,
			circle_msg->markers[i].pose.position.y));
		//if (lrf_msg->ranges[i] > 1.f)
		//obstacle_circles[i] = AdditionalPrimitives2D::Circle(center, 0.f);
		{
			obstacle_circles.push_back(AdditionalPrimitives2D::Circle(center, 0.f));
			obstacle_velocities.push_back(Geometry2D::Vec2(0.f, 0.f));
		}
	}
}

bool RDSNode::commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
	rds_network_ros::VelocityCommandCorrectionRDS::Response& response)
{
	/////////////////////////////////////////////////////////////////////////////////////////////
	// access to the stored circles (from measurements)
	for (int i = 0; i < qolo_cpg.obstacle_circles.size(); i++)
	{
		//... = qolo_cpg.obstacle_circles[i].center.x;
		//... = qolo_cpg.obstacle_circles[i].center.y;
		//... = qolo_cpg.obstacle_circles[i].radius;
	}

	// calling the MDS with the request and then setting the response object
	// ...

	return true;
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	RDS::VelocityCommand nominal_command(request.nominal_command.linear, request.nominal_command.angular);

	RDS::VelocityCommandHexagonLimits hexagon_limits;
	hexagon_limits.min_linear = request.velocity_limits.min_linear;
	hexagon_limits.max_linear = request.velocity_limits.max_linear;
	hexagon_limits.absolute_angular_at_min_linear = request.velocity_limits.abs_angular_at_min_linear;
	hexagon_limits.absolute_angular_at_max_linear = request.velocity_limits.abs_angular_at_max_linear;
	hexagon_limits.absolute_angular_at_zero_linear = request.velocity_limits.abs_angular_at_zero_linear;

	RDS::VelocityCommandBoxLimits box_limits;
	box_limits.min_linear = request.last_actual_command.linear - request.command_cycle_time*request.abs_linear_acceleration_limit;
	box_limits.min_angular = request.last_actual_command.angular - request.command_cycle_time*request.abs_angular_acceleration_limit;
	box_limits.max_linear = request.last_actual_command.linear + request.command_cycle_time*request.abs_linear_acceleration_limit;
	box_limits.max_angular = request.last_actual_command.angular + request.command_cycle_time*request.abs_angular_acceleration_limit;

	/*
	float y_coordinate_of_reference_point_for_command_limits = 0.5f;
	float weight_scaling_of_reference_point_for_command_limits = 1.f;
	float tau = 2.f;
	float delta = 0.05f;
	float clearance_from_axle_of_final_reference_point = 0.15f;
	*/
	float y_coordinate_of_reference_point_for_command_limits = request.y_coordinate_of_reference_point_for_command_limits;
	float weight_scaling_of_reference_point_for_command_limits = request.weight_scaling_of_reference_point_for_command_limits;
	float tau = request.tau;
	float delta = request.delta;
	float clearance_from_axle_of_final_reference_point = request.clearance_from_axle_of_final_reference_point;

	float y_coordinate_of_reference_biasing_point = request.y_coordinate_of_reference_biasing_point;
	float weight_of_reference_biasing_point = request.weight_of_reference_biasing_point;

	//try
	//{
	RDSWrap rds_wrap(nominal_command,
		box_limits,
		hexagon_limits,
		qolo_cpg.generateCollisionPoints().collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		weight_scaling_of_reference_point_for_command_limits,
		tau,
		delta,
		clearance_from_axle_of_final_reference_point,
		y_coordinate_of_reference_biasing_point,
		weight_of_reference_biasing_point);

	response.corrected_command.linear = rds_wrap.getCommandSolution().linear;
	response.corrected_command.angular = rds_wrap.getCommandSolution().angular;
	response.feasible = rds_wrap.isFeasible();

	rds_network_ros::ToGui msg_to_gui;
	msg_to_gui.nominal_command.linear = nominal_command.linear;
	msg_to_gui.nominal_command.angular = nominal_command.angular;
	msg_to_gui.corrected_command.linear = rds_wrap.getCommandSolution().linear;
	msg_to_gui.corrected_command.angular = rds_wrap.getCommandSolution().angular;
	msg_to_gui.reference_point.x = rds_wrap.getReferencePoint().x;
	msg_to_gui.reference_point.y = rds_wrap.getReferencePoint().y;
	msg_to_gui.reference_point_velocity_solution.x = rds_wrap.getReferencePointVelocitySolution().x;
	msg_to_gui.reference_point_velocity_solution.y = rds_wrap.getReferencePointVelocitySolution().y;
	msg_to_gui.reference_point_nominal_velocity.x = nominal_command.pointVelocity(rds_wrap.getReferencePoint()).x;
	msg_to_gui.reference_point_nominal_velocity.y = nominal_command.pointVelocity(rds_wrap.getReferencePoint()).y;
	rds_network_ros::HalfPlane2D h_msg;
	for (auto& h : rds_wrap.getReferencePointVelocityConstraints())
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.reference_point_velocity_constraints.push_back(h_msg);
	}
	msg_to_gui.solver_solution.x = rds_wrap.getScaledShiftedSolution().x;
	msg_to_gui.solver_solution.y = rds_wrap.getScaledShiftedSolution().y;
	for (auto& h : rds_wrap.getScaledShiftedConstraints())
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.solver_constraints.push_back(h_msg);
	}
	msg_to_gui.reference_point_for_command_limits.x = 0.f;
	msg_to_gui.reference_point_for_command_limits.y = y_coordinate_of_reference_point_for_command_limits;
	rds_network_ros::Point2D p_msg;
	for (auto& cp : qolo_cpg.collision_points)
	{
		p_msg.x = cp.p.x;
		p_msg.y = cp.p.y;
		msg_to_gui.collision_points_on_robot.push_back(p_msg);
		p_msg.x = (cp.p + cp.p_to_q).x;
		p_msg.y = (cp.p + cp.p_to_q).y;
		msg_to_gui.collision_points_on_obstacles.push_back(p_msg);
	}
	msg_to_gui.limit_velocity_command_abs_angular_at_zero_linear = hexagon_limits.absolute_angular_at_zero_linear;
	msg_to_gui.limit_velocity_command_max_linear = hexagon_limits.max_linear;
	rds_network_ros::Circle c_msg;
	for (auto& c : qolo_cpg.robot_shape_circles)
	{
		c_msg.center.x = c.center.x;
		c_msg.center.y = c.center.y;
		c_msg.radius = c.radius;
		msg_to_gui.robot_shape.push_back(c_msg);
	}
	publisher_for_gui.publish(msg_to_gui);

	return true;
	//}
	//catch(...)
	//{}
	//return false;
}

RDSNode::RDSNode(ros::NodeHandle* n)
	: qolo_cpg()
	, laserscan_subscriber(n->subscribe<visualization_msgs::MarkerArray>("circles", 1,
		&QoloCollisionPointGenerator::obstacleMessageCallback, &qolo_cpg))
	, publisher_for_gui(n->advertise<rds_network_ros::ToGui>("rds_to_gui", 1)) 
	, command_correction_server(n->advertiseService("rds_velocity_command_correction",
		&RDSNode::commandCorrectionService, this))
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_ros_node");
	ros::NodeHandle n;
	RDSNode rds_node(&n);
	return 0;
}