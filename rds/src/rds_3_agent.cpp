#include "rds_3_agent.hpp"
#include "capsule.hpp"
#include "rds_3.hpp"
#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using Geometry2D::Capsule;

void RDS3CapsuleAgent::stepEuler(float dt,
	const Vec2& v_nominal_p_ref,
	const std::vector<Circle>& circle_objects)
{
	std::vector<Circle> objects_local;

	getCircleObjectsInLocalFrame(circle_objects, &objects_local);
	Vec2 v_nominal_p_ref_local;
	transformVectorGlobalToLocal(v_nominal_p_ref, &v_nominal_p_ref_local);
	Geometry2D::RDS3 rds_3(rds_3_configuration.tau,
		rds_3_configuration.delta, rds_3_configuration.v_max);
	Vec2 v_corrected_p_ref_local;
	rds_3.computeCorrectedVelocity(rds_3_configuration.robot_shape, rds_3_configuration.p_ref,
		v_nominal_p_ref_local, objects_local, &v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.x*rds_3_configuration.p_ref.x/
		rds_3_configuration.p_ref.y + v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_3_configuration.p_ref.y;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
}

void RDS3CapsuleAgent::getCircleObjectsInLocalFrame(const std::vector<Circle>& objects,
		std::vector<Circle>* objects_local)
{
	float rxx = std::cos(-orientation);
	float rxy = std::sin(-orientation);
	float ryx = -rxy;
	float ryy = rxx;
	objects_local->resize(0);
	for (auto& c : objects)
	{
		objects_local->push_back(Circle(Vec2(rxx*(c.center - position).x + ryx*(c.center - position).y,
			rxy*(c.center - position).x + ryy*(c.center - position).y), c.radius));
	}
}

void RDS3CapsuleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS3CapsuleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}

void RDS3CircleAgent::stepEuler(float dt, const Vec2& v_nominal_p_ref,
	const std::vector<Circle>& circle_objects,
	const std::vector<Capsule>& capsule_objects,
	std::vector<Circle>::size_type circle_index_to_skip)
{
	std::vector<Circle> circle_objects_local;
	getCircleObjectsInLocalFrame(circle_objects, &circle_objects_local, circle_index_to_skip);
	std::vector<Capsule> capsule_objects_local;
	getCapsuleObjectsInLocalFrame(capsule_objects, &capsule_objects_local);

	Vec2 v_nominal_p_ref_local;
	transformVectorGlobalToLocal(v_nominal_p_ref, &v_nominal_p_ref_local);
	Geometry2D::RDS3 rds_3(rds_3_configuration.tau,
		rds_3_configuration.delta, rds_3_configuration.v_max);
	Vec2 v_corrected_p_ref_local;
	rds_3.computeCorrectedVelocity(rds_3_configuration.robot_shape, rds_3_configuration.p_ref,
		v_nominal_p_ref_local, circle_objects_local, capsule_objects_local, &v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.x*rds_3_configuration.p_ref.x/
		rds_3_configuration.p_ref.y + v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_3_configuration.p_ref.y;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
}

void RDS3CircleAgent::getCircleObjectsInLocalFrame(const std::vector<Circle>& circle_objects,
	std::vector<Circle>* circle_objects_local,
	std::vector<Circle>::size_type circle_index_to_skip)
{
	float rxx = std::cos(-orientation);
	float rxy = std::sin(-orientation);
	float ryx = -rxy;
	float ryy = rxx;
	circle_objects_local->resize(0);
	for (std::vector<Circle>::size_type i = 0; i != circle_objects.size(); i++)
	{
		if (i != circle_index_to_skip)
		{
			const Circle& c(circle_objects[i]);
			circle_objects_local->push_back(Circle(Vec2(rxx*(c.center - position).x + ryx*(c.center - position).y,
				rxy*(c.center - position).x + ryy*(c.center - position).y), c.radius));
		}
	}
}

void RDS3CircleAgent::getCapsuleObjectsInLocalFrame(const std::vector<Capsule>& capsule_objects,
	std::vector<Capsule>* capsule_objects_local)
{
	float rxx = std::cos(-orientation);
	float rxy = std::sin(-orientation);
	float ryx = -rxy;
	float ryy = rxx;
	capsule_objects_local->resize(0);
	for (auto& c : capsule_objects)
	{
		capsule_objects_local->push_back(Capsule(c.radius,
			Vec2(rxx*(c.center_a - position).x + ryx*(c.center_a - position).y,
				rxy*(c.center_a - position).x + ryy*(c.center_a - position).y),
			Vec2(rxx*(c.center_b - position).x + ryx*(c.center_b - position).y,
				rxy*(c.center_b - position).x + ryy*(c.center_b - position).y)));
	}
}

void RDS3CircleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS3CircleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}