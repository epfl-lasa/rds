#include "rds_ros_node.hpp"

#include <rds/capsule.hpp>
#include <rds/config_rds_5.hpp>
#include <rds/distance_minimizer.hpp>

#include <rds_network_ros/ToGui.h>
#include <rds_network_ros/HalfPlane2D.h>
#include <rds_network_ros/Point2D.h>
#include <rds_network_ros/Circle.h>

#include <geometry_msgs/TransformStamped.h>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;

PersonTracks::PersonTracks(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg)
	: time(std::chrono::high_resolution_clock::now())
	, frame_id(tracks_msg->header.frame_id)
	, delay(0.1f)
{
	MovingObject3 pers_global;
	for (const auto& track : tracks_msg->tracks)
	{
		pers_global.position.setX(track.pose.pose.position.x);
		pers_global.position.setY(track.pose.pose.position.y);
		pers_global.position.setZ(track.pose.pose.position.z);
		pers_global.velocity.setX(track.twist.twist.linear.x);
		pers_global.velocity.setY(track.twist.twist.linear.y);
		pers_global.velocity.setZ(track.twist.twist.linear.z);
		persons_global.push_back(pers_global);
	}
	for (auto& pers : persons_global)
		pers.position += delay*pers.velocity;
}

void PersonTracks::updatePositions(const std::chrono::time_point<std::chrono::high_resolution_clock>& time_now)
{
	std::chrono::duration<double> t_step_duration(time_now - time);
	time = time_now;
	for (auto& pers : persons_global)
		pers.position += t_step_duration.count()*pers.velocity;
}

int RDSNode::obtainTf(const std::string& frame_id_1, const std::string& frame_id_2, tf2::Transform* tf)
{
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		transformStamped = tf_buffer.lookupTransform(
			frame_id_1, frame_id_2 //"sick_laser_front"//
			, ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s excpetion, when looking up tf from %s to %s", ex.what(), frame_id_1.c_str(), frame_id_2.c_str());
		return 1;
	}

	tf2::Quaternion rotation(transformStamped.transform.rotation.x,
		transformStamped.transform.rotation.y,
		transformStamped.transform.rotation.z,
		transformStamped.transform.rotation.w);

	tf2::Vector3 translation(transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);

	*tf = tf2::Transform(rotation, translation);
	return 0;
}

void RDSNode::callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg)
{
	m_person_tracks = PersonTracks(tracks_msg);
}

int RDSNode::makeLocalPersons(const std::vector<MovingObject3>& persons_global,
	const std::string& tracks_frame_id, std::vector<MovingCircle>* persons_local)
{
	if (persons_global.size() == 0)
		return 0;
	tf2::Transform tf;
	int error_tf_lookup = obtainTf("tf_rds", tracks_frame_id, &tf);
	if (error_tf_lookup)
		return 1;
	tf2::Transform tf_only_rotation(tf.getRotation());

	if (persons_global.size() != persons_local->size())
		persons_local->resize(persons_global.size());

	tf2::Vector3 position_local, velocity_local;
	for (unsigned int i = 0; i != persons_global.size(); i++)
	{
		position_local = tf*persons_global[i].position;
		velocity_local = tf_only_rotation*persons_global[i].velocity;
		(*persons_local)[i].circle.center.x = position_local.getX();
		(*persons_local)[i].circle.center.y = position_local.getY();
		(*persons_local)[i].circle.radius = 0.3f;
		(*persons_local)[i].velocity.x = velocity_local.getX();
		(*persons_local)[i].velocity.y = velocity_local.getY();
	}
	return 0;
}

bool RDSNode::commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
	rds_network_ros::VelocityCommandCorrectionRDS::Response& response)
{
	// prepare pedestrian tracks/ scan points retrieved from recent messages
	std::vector<MovingCircle> lrf_moving_objects;
	std::vector<MovingCircle> all_moving_objects;
	if (request.lrf_point_obstacles)
	{
		MovingCircle moving_object;
		moving_object.velocity = Vec2(0.0, 0.0);
		moving_object.circle.radius = 0.0;
		for (int i = 0; i < m_aggregator_two_lrf.size(); i++)
		{
			moving_object.circle.center = m_aggregator_two_lrf.getPoint(i);
			lrf_moving_objects.push_back(moving_object);
			all_moving_objects.push_back(moving_object);
		}
	}

	m_person_tracks.updatePositions(std::chrono::high_resolution_clock::now());
	if (makeLocalPersons(m_person_tracks.getPersonsGlobal(),
		m_person_tracks.getFrameId(), &m_person_tracks.persons_local) != 0)
		ROS_WARN("Using old local person positions (could be none).");
	for (auto& pedestrian : m_person_tracks.persons_local)
		all_moving_objects.push_back(pedestrian);

	// parse service parameters
	float a_v_min = command_correct_previous_linear - request.dt*request.acc_limit_linear_abs_max;
	float a_v_max = command_correct_previous_linear + request.dt*request.acc_limit_linear_abs_max;
	float a_w_min = command_correct_previous_angular - request.dt*request.acc_limit_angular_abs_max;
	float a_w_max = command_correct_previous_angular + request.dt*request.acc_limit_angular_abs_max;
	VWBox vw_box_limits(a_v_min, a_v_max, a_w_min, a_w_max);

	VWDiamond vw_diamond_limits(request.vel_lim_linear_min, request.vel_lim_linear_max,
		request.vel_lim_angular_abs_max, request.vel_linear_at_angular_abs_max);

	const RDS5CapsuleConfiguration rds_5_config = ConfigRDS5::ConfigWrap(request.dt).rds_5_config;

	float tau = rds_5_config.tau; //request.rds_tau
	float delta = rds_5_config.delta; //request.rds_delta
	float y_p_ref = rds_5_config.y_p_ref; //request.reference_point_y

	Geometry2D::RDS5 rds_5(tau, delta, y_p_ref, vw_box_limits, vw_diamond_limits);

	rds_5.use_conservative_shift = false;
	rds_5.keep_origin_feasible = false;
	rds_5.no_VO_shift_at_contact = false;
	rds_5.shift_reduction_range = 0.35f;
	rds_5.ORCA_implementation = request.ORCA_implementation;
	rds_5.ORCA_use_p_ref = true;
	rds_5.ORCA_solver = true;

	float capsule_radius = rds_5_config.robot_shape.radius();//request.capsule_radius
	float capsule_center_front_y = rds_5_config.robot_shape.center_a().y; //request.capsule_center_front_y
	float capsule_center_rear_y = rds_5_config.robot_shape.center_b().y; //request.capsule_center_rear_y

	Capsule robot_shape(capsule_radius, Vec2(0.0, capsule_center_front_y),
		Vec2(0.0, capsule_center_rear_y)); //0.45, 0.05, -0.5

	Vec2 v_nominal_p_ref(-y_p_ref*request.nominal_command.angular,
		request.nominal_command.linear);

	Vec2 v_previous_command(-command_correct_previous_angular*y_p_ref,
		command_correct_previous_linear);

	Vec2 v_corrected_p_ref(0.f, 0.f);

	if (v_nominal_p_ref.norm() > std::abs(vw_diamond_limits.v_max))
		v_nominal_p_ref = v_nominal_p_ref.normalized()*std::abs(vw_diamond_limits.v_max);

	// compute collision avoidance command
	try
	{
		rds_5.computeCorrectedVelocity(robot_shape, v_nominal_p_ref, v_previous_command,
			std::vector<MovingCircle>(), all_moving_objects, &v_corrected_p_ref);
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		float breaking_step_linear = request.dt*rds_5_config.breaking_deceleration_linear;
		float breaking_step_angular = request.dt*rds_5_config.breaking_deceleration_angular;
		float new_v_linear, new_v_angular;
		if (command_correct_previous_linear > 0.f)
			new_v_linear = std::max(0.f, command_correct_previous_linear - breaking_step_linear);
		else
			new_v_linear = std::min(0.f, command_correct_previous_linear + breaking_step_linear);
		if (command_correct_previous_angular > 0.f)
			new_v_angular = std::max(0.f, command_correct_previous_angular - breaking_step_angular);
		else
			new_v_angular = std::min(0.f, command_correct_previous_angular + breaking_step_angular);
		v_corrected_p_ref.y = new_v_linear;
		v_corrected_p_ref.x = -new_v_angular*rds_5_config.y_p_ref;
	}

	// communicate the result and the underlying representations 
	response.corrected_command.linear = v_corrected_p_ref.y;
	response.corrected_command.angular = -1.0/y_p_ref*v_corrected_p_ref.x;
	
	command_correct_previous_linear = response.corrected_command.linear;
	command_correct_previous_angular = response.corrected_command.angular;

	call_counter++;
	response.call_counter = call_counter;

	rds_network_ros::ToGui msg_to_gui;
	msg_to_gui.nominal_command.linear = request.nominal_command.linear;
	msg_to_gui.nominal_command.angular = request.nominal_command.angular;
	msg_to_gui.corrected_command.linear = response.corrected_command.linear;
	msg_to_gui.corrected_command.angular = response.corrected_command.angular;
	msg_to_gui.reference_point.x = 0.f;
	msg_to_gui.reference_point.y = y_p_ref;
	msg_to_gui.reference_point_velocity_solution.x = v_corrected_p_ref.x;
	msg_to_gui.reference_point_velocity_solution.y = v_corrected_p_ref.y;
	msg_to_gui.reference_point_nominal_velocity.x = v_nominal_p_ref.x;
	msg_to_gui.reference_point_nominal_velocity.y = v_nominal_p_ref.y;
	rds_network_ros::HalfPlane2D h_msg;
	for (auto& h : rds_5.constraints)
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.reference_point_velocity_constraints.push_back(h_msg);
	}

	rds_network_ros::Circle c_msg;
	for (auto & mo : all_moving_objects)
	{
		c_msg.center.x = mo.circle.center.x;
		c_msg.center.y = mo.circle.center.y;
		c_msg.radius = mo.circle.radius + delta;
		msg_to_gui.moving_objects.push_back(c_msg);
	}
	rds_network_ros::Point2D head_msg, tail_msg;
	for (auto & mo : all_moving_objects)
	{
		if ((mo.velocity.x != 0.f) || (mo.velocity.y != 0.f))
		{
			head_msg.x = mo.circle.center.x + mo.velocity.x*tau;
			head_msg.y = mo.circle.center.y + mo.velocity.y*tau;
			tail_msg.x = mo.circle.center.x;
			tail_msg.y = mo.circle.center.y;
			msg_to_gui.moving_objects_predictions.push_back(head_msg);
			msg_to_gui.moving_objects_predictions.push_back(tail_msg);
		}
	}

	msg_to_gui.robot_shape.radius = robot_shape.radius();
	msg_to_gui.robot_shape.center_a.x = robot_shape.center_a().x;
	msg_to_gui.robot_shape.center_a.y = robot_shape.center_a().y;
	msg_to_gui.robot_shape.center_b.x = robot_shape.center_b().x;
	msg_to_gui.robot_shape.center_b.y = robot_shape.center_b().y;

	if (rds_5.n_bounding_circles > 2)
	{
		rds_network_ros::Circle bc_msg;
		for (const auto& bc : rds_5.bounding_circles.circles())
		{
			bc_msg.center.x = bc.center.x;
			bc_msg.center.y = bc.center.y;
			bc_msg.radius = bc.radius;
			msg_to_gui.bounding_circles.push_back(bc_msg);
		}
	}

	publisher_for_gui.publish(msg_to_gui);

	return true;
}

RDSNode::RDSNode(ros::NodeHandle* n, AggregatorTwoLRF& agg)
	: m_aggregator_two_lrf(agg)
	, subscriber_lrf_front(n->subscribe<sensor_msgs::LaserScan>("front_lidar/scan"//"sick_laser_front/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFFront, &m_aggregator_two_lrf))
	, subscriber_lrf_rear(n->subscribe<sensor_msgs::LaserScan>("rear_lidar/scan"//"sick_laser_rear/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFRear, &m_aggregator_two_lrf))
	, subscriber_tracker(n->subscribe<frame_msgs::TrackedPersons>("rwth_tracker/tracked_persons"
		, 1, &RDSNode::callbackTracker, this) )
	, publisher_for_gui(n->advertise<rds_network_ros::ToGui>("rds_to_gui", 1)) 
	, command_correction_server(n->advertiseService("rds_velocity_command_correction",
		&RDSNode::commandCorrectionService, this))
	, tf_listener(tf_buffer)
	, command_correct_previous_linear(0.f)
	, command_correct_previous_angular(0.f)
	, call_counter(0)
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_ros_node");

	AggregatorTwoLRF aggregator_two_lrf(
		3.f*M_PI/4.f,
		0.05,
		100.f,
		3.f*M_PI/4.f,
		0.05,
		100.f);

	ros::NodeHandle n;
	RDSNode rds_node(&n, aggregator_two_lrf);
	return 0;
}
