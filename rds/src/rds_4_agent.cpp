#include "rds_4_agent.hpp"
#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using Geometry2D::Capsule;

void RDS4CapsuleAgent::stepEuler(float dt,
	const Vec2& v_nominal_p_ref,
	const std::vector<MovingCircle>& objects)
{
	std::vector<MovingCircle> objects_local;

	getObjectsInLocalFrame(objects, &objects_local);
	Vec2 v_nominal_p_ref_local;
	transformVectorGlobalToLocal(v_nominal_p_ref, &v_nominal_p_ref_local);
	Geometry2D::RDS4 rds_4(rds_configuration.tau,
		rds_configuration.delta, rds_configuration.v_max);
	Vec2 v_corrected_p_ref_local;
	rds_4.computeCorrectedVelocity(rds_configuration.robot_shape, rds_configuration.p_ref,
		v_nominal_p_ref_local, objects_local, &v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.x*rds_configuration.p_ref.x/
		rds_configuration.p_ref.y + v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_configuration.p_ref.y;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
}

void RDS4CapsuleAgent::getObjectsInLocalFrame(const std::vector<MovingCircle>& objects,
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

void RDS4CapsuleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS4CapsuleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}