#include "rds_5_agent.hpp"
#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using Geometry2D::Capsule;

void RDS5CapsuleAgent::stepEuler(float dt,
	const Vec2& v_nominal_p_ref,
	const std::vector<MovingCircle>& objects)
{
	std::vector<MovingCircle> objects_local;

	getObjectsInLocalFrame(objects, &objects_local);
	Vec2 v_nominal_p_ref_local;
	transformVectorGlobalToLocal(v_nominal_p_ref, &v_nominal_p_ref_local);
	
	float v_box_half = rds_configuration.linear_acceleration_limit*rds_configuration.dt_cycle;
	float w_box_half = rds_configuration.angular_acceleration_limit*rds_configuration.dt_cycle;
	float previous_v_linear = last_step_p_ref_velocity_local.y;
	float previous_v_angular = -last_step_p_ref_velocity_local.x/rds_configuration.y_p_ref;
	VWBox box_limits(previous_v_linear - v_box_half, previous_v_linear + v_box_half,
		previous_v_angular - w_box_half, previous_v_angular + w_box_half);

	Geometry2D::RDS5 rds_5(rds_configuration.tau, rds_configuration.delta, rds_configuration.y_p_ref,
		box_limits, rds_configuration.vw_diamond_limits);

	if (v_nominal_p_ref_local.norm() > std::abs(rds_configuration.vw_diamond_limits.v_max))
		v_nominal_p_ref_local = v_nominal_p_ref_local.normalized()*std::abs(rds_configuration.vw_diamond_limits.v_max);

	Vec2 v_corrected_p_ref_local;
	rds_5.computeCorrectedVelocity(rds_configuration.robot_shape, v_nominal_p_ref_local,
		last_step_p_ref_velocity_local, std::vector<MovingCircle>(), objects_local,
		&v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_configuration.y_p_ref;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
	last_step_p_ref_velocity_local = v_corrected_p_ref_local;
}

void RDS5CapsuleAgent::getObjectsInLocalFrame(const std::vector<MovingCircle>& objects,
		std::vector<MovingCircle>* objects_local)
{
	float rxx = std::cos(-orientation);
	float rxy = std::sin(-orientation);
	float ryx = -rxy;
	float ryy = rxx;
	objects_local->resize(0);
	for (auto& mc : objects)
	{
		objects_local->push_back(MovingCircle(Circle(Vec2(rxx*(mc.circle.center - position).x + ryx*(mc.circle.center - position).y,
			rxy*(mc.circle.center - position).x + ryy*(mc.circle.center - position).y), mc.circle.radius),
			Vec2(rxx*mc.velocity.x + ryx*mc.velocity.y, rxy*mc.velocity.x + ryy*mc.velocity.y)));
	}
}

void RDS5CapsuleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local) const
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS5CapsuleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global) const
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}

void RDS5CapsuleAgent::transformReferencePointVelocityToPointVelocity(const Vec2& p_local,
	const Vec2& v_p_ref_global, Vec2* v_global) const
{
	Vec2 v_p_ref_local;
	transformVectorGlobalToLocal(v_p_ref_global, &v_p_ref_local);
	float v_linear = v_p_ref_local.y;
	float v_angular = -v_p_ref_local.x/rds_configuration.y_p_ref;
	Vec2 v_local = Vec2(0.f, v_linear) + Vec2(-v_angular*p_local.y, v_angular*p_local.x);
	transformVectorLocalToGlobal(v_local, v_global);
}