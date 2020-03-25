#include "rds_2_agent.hpp"
#include "distance_minimizer.hpp"
#include "capsule.hpp"
#include "rds_2.hpp"
#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
//using AdditionalPrimitives2D::Polygon;
using Geometry2D::HalfPlane2;
using Geometry2D::Capsule;

void RDS2CapsuleAgent::stepEuler(float dt,
	const Vec2& v_nominal_p_ref,
	const std::vector<Circle>& circle_objects)
	//, const std::vector<Polygon>& convex_polygon_objects)
{
	std::vector<Circle> objects_local;
	/*for (auto& pg : convex_polygon_objects)
	{
		Circle c_pg(Vec2(0.f, 0.f), 0.f);
		getClosestPointOfConvexPolygon(pg, position, )
	}*/
	getCircleObjectsInLocalFrame(circle_objects, &objects_local);
	Vec2 v_nominal_p_ref_local;
	transformVectorGlobalToLocal(v_nominal_p_ref, &v_nominal_p_ref_local);
	Geometry2D::RDS2 rds_2(rds_2_configuration.T, rds_2_configuration.D,
		rds_2_configuration.delta, rds_2_configuration.v_max);
	Vec2 v_corrected_p_ref_local;
	rds_2.computeCorrectedVelocity(rds_2_configuration.robot_shape, rds_2_configuration.p_ref,
		v_nominal_p_ref_local, objects_local, &v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.x*rds_2_configuration.p_ref.x/
		rds_2_configuration.p_ref.y + v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_2_configuration.p_ref.y;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
}
/*
void RDS2Agent::getClosestPointOfConvexPolygon(const Polygon& convex_polygon,
	const Vec2& pt_query, Vec2* closest_point)
{
	if (convex_polygon.size() == 0)
		throw PolygonException();
	if (convex_polygon.size() == 1)
	{
		*closest_point = convex_polygon[0];
		return;
	}
	if (convex_polygon.size() == 2)
	{
		Capsule cap(1.f, convex_polygon[0], convex_polygon[1]);
		cap.closestMidLineSegmentPoint(pt_query, closest_point);
		return;
	}

	Polygon shifted_scaled_polygon(convex_polygon);
	float max_abs_coordinate = 0.f;
	for (auto& v : shifted_scaled_polygon)
	{
		v = v - pt_query;
		if (std::abs(v.x) > max_abs_coordinate)
			max_abs_coordinate = std::abs(v.x);
		if (std::abs(v.y) > max_abs_coordinate)
			max_abs_coordinate = std::abs(v.y);
	}
	float scaling = 0.5f/(max_abs_coordinate + 0.01f);
	for (auto& v : shifted_scaled_polygon)
		v = v*scaling;

	Vec2 p_inside(0.f, 0.f);
	for (Polygon::size_type i = 0; i != convex_polygon.size(); i++)
		p_inside = p_inside + shifted_scaled_polygon[i]/float(convex_polygon.size());
	std::vector<HalfPlane2> H;
	for (Polygon::size_type i = 1; i != convex_polygon.size(); i++)
		H.push_back(HalfPlane2(p_inside, shifted_scaled_polygon[i - 1], shifted_scaled_polygon[i]));
	H.push_back(HalfPlane2(p_inside, shifted_scaled_polygon[0], shifted_scaled_polygon.back()));

	Vec2 shifted_scaled_solution = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(H);
	*closest_point = shifted_scaled_solution/scaling + pt_query;
}
*/

void RDS2CapsuleAgent::getCircleObjectsInLocalFrame(const std::vector<Circle>& objects,
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

void RDS2CapsuleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS2CapsuleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}

void RDS2CircleAgent::stepEuler(float dt, const Vec2& v_nominal_p_ref,
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
	Geometry2D::RDS2 rds_2(rds_2_configuration.T, rds_2_configuration.D,
		rds_2_configuration.delta, rds_2_configuration.v_max);
	Vec2 v_corrected_p_ref_local;
	rds_2.computeCorrectedVelocity(rds_2_configuration.robot_shape, rds_2_configuration.p_ref,
		v_nominal_p_ref_local, circle_objects_local, capsule_objects_local, &v_corrected_p_ref_local);
	
	float v_linear = v_corrected_p_ref_local.x*rds_2_configuration.p_ref.x/
		rds_2_configuration.p_ref.y + v_corrected_p_ref_local.y;
	float v_angular = -v_corrected_p_ref_local.x/rds_2_configuration.p_ref.y;

	Vec2 robot_local_velocity = Vec2(0.f, v_linear);
	Vec2 robot_global_velocity;
	transformVectorLocalToGlobal(robot_local_velocity, &robot_global_velocity);

	position = position + dt*robot_global_velocity;
	orientation = orientation + dt*v_angular;
	transformVectorLocalToGlobal(v_corrected_p_ref_local, &(this->last_step_p_ref_velocity));
}

void RDS2CircleAgent::getCircleObjectsInLocalFrame(const std::vector<Circle>& circle_objects,
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

void RDS2CircleAgent::getCapsuleObjectsInLocalFrame(const std::vector<Capsule>& capsule_objects,
	std::vector<Capsule>* capsule_objects_local)
{
	float rxx = std::cos(-orientation);
	float rxy = std::sin(-orientation);
	float ryx = -rxy;
	float ryy = rxx;
	capsule_objects_local->resize(0);
	for (auto& c : capsule_objects)
	{
		capsule_objects_local->push_back(Capsule(c.radius(),
			Vec2(rxx*(c.center_a() - position).x + ryx*(c.center_a() - position).y,
				rxy*(c.center_a() - position).x + ryy*(c.center_a() - position).y),
			Vec2(rxx*(c.center_b() - position).x + ryx*(c.center_b() - position).y,
				rxy*(c.center_b() - position).x + ryy*(c.center_b() - position).y)));
	}
}

void RDS2CircleAgent::transformVectorGlobalToLocal(const Vec2& v_global, Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void RDS2CircleAgent::transformVectorLocalToGlobal(const Vec2& v_local, Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}