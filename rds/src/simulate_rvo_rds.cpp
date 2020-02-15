#include "simulate_rvo_rds.hpp"

#include "geometry.hpp"

#include "rds_wrap.hpp"

#include "rds_2.hpp"

#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using RDS::VelocityCommand;

static float orientationReferenceTracking(float orientation, float orientation_reference, float gain)
{
	while (orientation_reference - orientation > M_PI)
		orientation += 2.f*M_PI;
	while (orientation_reference - orientation < -M_PI)
		orientation -= 2.f*M_PI;
	return gain*(orientation_reference - orientation);
}

void SimulateRvoRds::stepEuler(float dt)
{
	float rxx = std::cos(s_rvo->agents[0]->orientation);
	float rxy = std::sin(s_rvo->agents[0]->orientation);
	float ryx = -rxy;
	float ryy = rxx;

	Vec2 v_ref_global;
	s_rvo->agents[0]->environment->getReferenceVelocity(s_rvo->time,
		s_rvo->agents[0]->position, &v_ref_global);

	Vec2 v_ref_local(rxx*v_ref_global.x - ryx*v_ref_global.y,
		-rxy*v_ref_global.x + ryy*v_ref_global.y);

	/*float orientation_ref = std::atan2(v_ref_global.y, v_ref_global.x);
	float gain = 0.1f;
	float angular_v_ref = orientationReferenceTracking(s_rvo->agents[0]->orientation + M_PI/2.f, orientation_ref, gain);
	float linear_v_ref = v_ref_local.y;*/

	float linear_v_ref = s_rvo->agents[0]->reference_point.x/s_rvo->agents[0]->reference_point.y*v_ref_local.x + v_ref_local.y;
	float angular_v_ref = -v_ref_local.x/s_rvo->agents[0]->reference_point.y;

	// set RDS parameters
	float abs_linear_acceleration_limit = 1000.f;
	float abs_angular_acceleration_limit = 1000.f;

	RDS::VelocityCommandHexagonLimits hexagon_limits;
	hexagon_limits.min_linear = -0.75f;
	hexagon_limits.max_linear = 1.75f;
	hexagon_limits.absolute_angular_at_min_linear = 0.f;
	hexagon_limits.absolute_angular_at_max_linear = 0.f;
	hexagon_limits.absolute_angular_at_zero_linear = 1.f;

	float y_coordinate_of_reference_point_for_command_limits = 0.5f;
	float weight_scaling_of_reference_point_for_command_limits = 1.f;
	//float tau = 5.f; 
	//float delta = 0.05f;
	float clearance_from_axle_of_final_reference_point = 0.15f;

	// compute RDS command correction
	VelocityCommand nominal_command(linear_v_ref, angular_v_ref);
	
	RDS::VelocityCommandBoxLimits box_limits;
	box_limits.min_linear = - abs_linear_acceleration_limit;
	box_limits.min_angular = - abs_angular_acceleration_limit;
	box_limits.max_linear = abs_linear_acceleration_limit;
	box_limits.max_angular = abs_angular_acceleration_limit;

	std::vector<RDS::CollisionPoint> collision_points;
	createCollisionPointsInRobotFrame(&collision_points);

	RDSWrap rds_wrap(
		nominal_command,
		box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		weight_scaling_of_reference_point_for_command_limits,
		tau,
		delta,
		clearance_from_axle_of_final_reference_point,
		false,
		0.15f,
		1.f);
	
	// perform the Euler integration step
	Vec2 robot_local_velocity = rds_wrap.getCommandSolution().pointVelocity(Vec2(0.f, 0.f));
	Vec2 robot_global_velocity(rxx*robot_local_velocity.x + ryx*robot_local_velocity.y,
		rxy*robot_local_velocity.x + ryy*robot_local_velocity.y);
	
	float current_robot_orientation = s_rvo->agents[0]->orientation;
	Vec2 current_robot_position = s_rvo->agents[0]->position;

	float new_robot_orientation = current_robot_orientation + dt*rds_wrap.getCommandSolution().angular;
	Vec2 new_robot_position = current_robot_position + dt*robot_global_velocity;

	s_rvo->stepEuler(dt);
	s_rvo->agents[0]->orientation = new_robot_orientation; // overwrite the result for the robot
	s_rvo->agents[0]->position = new_robot_position;

};

void SimulateRvoRds::createCollisionPointsInRobotFrame(std::vector<RDS::CollisionPoint>* collision_points)
{
	collision_points->resize(0);
	float rxx = std::cos(-s_rvo->agents[0]->orientation);
	float rxy = std::sin(-s_rvo->agents[0]->orientation);
	float ryx = -rxy;
	float ryy = rxx;
	Vec2 rob_pos = s_rvo->agents[0]->position;
	for (std::vector<Agent*>::size_type i = 1; i != s_rvo->agents.size(); i++)
	{
		for (std::vector<Circle>::size_type l = 0; l != s_rvo->agents[0]->circles.size(); l++)
		{
			for (std::vector<Circle>::size_type k = 0; k != s_rvo->agents[i]->circles.size(); k++)
			{
				Circle o_ik;
				Vec2 v_pref_ik;
				s_rvo->agents[i]->getCircleAndNominalVelocityGlobal(k, 0.f, &o_ik, &v_pref_ik);
				Circle o_ik_local;
				o_ik_local.radius = o_ik.radius;
				o_ik_local.center = Vec2(rxx*(o_ik.center - rob_pos).x + ryx*(o_ik.center - rob_pos).y,
					rxy*(o_ik.center - rob_pos).x + ryy*(o_ik.center - rob_pos).y);

				o_ik.center = o_ik.center - s_rvo->agents[0]->position;
				Circle robot_circle_local = s_rvo->agents[0]->circles[l];
				collision_points->push_back(RDS::CollisionPoint(robot_circle_local, o_ik_local, Vec2(0.f, 0.f)));
			}
		}
	}
}



void SimulateRvoRds::stepEulerRDS2(float dt, const RDS2Configuration& rds_2_config)
{
	float rxx = std::cos(s_rvo->agents[0]->orientation);
	float rxy = std::sin(s_rvo->agents[0]->orientation);
	float ryx = -rxy;
	float ryy = rxx;

	Vec2 v_ref_global;
	s_rvo->agents[0]->environment->getReferenceVelocity(s_rvo->time,
		s_rvo->agents[0]->position, &v_ref_global);

	Vec2 v_ref_local(rxx*v_ref_global.x - ryx*v_ref_global.y,
		-rxy*v_ref_global.x + ryy*v_ref_global.y);


	std::vector<Circle> objects;
	getObjectsInLocalFrame(&objects);

	Geometry2D::RDS2 rds_2(rds_2_config.T, rds_2_config.D, rds_2_config.delta, rds_2_config.v_max);
	Vec2 v_corrected_p_ref;
	rds_2.computeCorrectedVelocity(rds_2_config.robot_shape, rds_2_config.p_ref,
		v_ref_local, objects, &v_corrected_p_ref);

	rds_constraints = rds_2.constraints;

	float v_linear = v_corrected_p_ref.x*rds_2_config.p_ref.x/rds_2_config.p_ref.y + v_corrected_p_ref.y;
	float v_angular = -v_corrected_p_ref.x/rds_2_config.p_ref.y;

	// perform the Euler integration step
	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity(rxx*robot_local_velocity.x + ryx*robot_local_velocity.y,
		rxy*robot_local_velocity.x + ryy*robot_local_velocity.y);
	
	float current_robot_orientation = s_rvo->agents[0]->orientation;
	Vec2 current_robot_position = s_rvo->agents[0]->position;

	float new_robot_orientation = current_robot_orientation + dt*v_angular;
	Vec2 new_robot_position = current_robot_position + dt*robot_global_velocity;

	s_rvo->stepEuler(dt);
	s_rvo->agents[0]->orientation = new_robot_orientation; // overwrite the result for the robot
	s_rvo->agents[0]->position = new_robot_position;
}

void SimulateRvoRds::getObjectsInLocalFrame(std::vector<Circle>* objects)
{
	objects->resize(0);
	float rxx = std::cos(-s_rvo->agents[0]->orientation);
	float rxy = std::sin(-s_rvo->agents[0]->orientation);
	float ryx = -rxy;
	float ryy = rxx;
	Vec2 rob_pos = s_rvo->agents[0]->position;
	for (std::vector<Agent*>::size_type i = 1; i != s_rvo->agents.size(); i++)
	{
		for (std::vector<Circle>::size_type k = 0; k != s_rvo->agents[i]->circles.size(); k++)
		{
			Circle o_ik;
			Vec2 v_pref_ik;
			s_rvo->agents[i]->getCircleAndNominalVelocityGlobal(k, 0.f, &o_ik, &v_pref_ik);
			Circle o_ik_local;
			o_ik_local.radius = o_ik.radius;
			o_ik_local.center = Vec2(rxx*(o_ik.center - rob_pos).x + ryx*(o_ik.center - rob_pos).y,
				rxy*(o_ik.center - rob_pos).x + ryy*(o_ik.center - rob_pos).y);
			objects->push_back(o_ik_local);
		}
	}
}