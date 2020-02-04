#include "simulation.hpp"
#include "distance_minimizer.hpp"

#include <cmath>
#include <iostream>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;

void Simulation::stepEuler(float dt)
{
	for (auto& a : agents)
		a->constraints.resize(0);

	for (std::vector<Agent*>::size_type i = 0; i != agents.size(); i++)
	{
		for (std::vector<Agent*>::size_type j = i + 1; j != agents.size(); j++)
		{
			for (std::vector<Circle>::size_type k = 0; k != agents[i]->circles.size(); k++)
			{
				for (std::vector<Circle>::size_type l = 0; l != agents[j]->circles.size(); l++)
				{
					Circle o_ik, o_jl;
					Vec2 v_pref_ik, v_pref_jl;
					agents[i]->getCircleAndNominalVelocityGlobal(k, time, &o_ik, &v_pref_ik);
					agents[j]->getCircleAndNominalVelocityGlobal(l, time, &o_jl, &v_pref_jl);
					HalfPlane2 vo_ik, vo_jl;
					rvo.computeCoordinativeVelocityObstacles(o_ik, o_jl, v_pref_ik, v_pref_jl, &vo_ik, &vo_jl);
					agents[i]->addGlobalCircleVelocityConstraint(k, vo_ik);
					agents[j]->addGlobalCircleVelocityConstraint(l, vo_jl);
				}
			}
		}
	}

	for (auto& a : agents)
	{
		Vec2 v;
		float w;
		a->getConstrainedOptimizedVelocity(time, &v, &w);
		a->position = a->position + dt*v;
		a->orientation += dt*w;
	}
	time += dt;
}

void Agent::transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local)
{
	*v_local = Vec2(std::cos(orientation)*v_global.x + std::sin(orientation)*v_global.y,
		-std::sin(orientation)*v_global.x + std::cos(orientation)*v_global.y);
}

void Agent::transformPositionGlobalToLocal(const Geometry2D::Vec2& p_global, Geometry2D::Vec2* p_local)
{
	transformVectorGlobalToLocal(p_global - position, p_local);
}

void Agent::transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global)
{
	*v_global = Vec2(std::cos(orientation)*v_local.x - std::sin(orientation)*v_local.y,
		+std::sin(orientation)*v_local.x + std::cos(orientation)*v_local.y);
}

void Agent::transformPositionLocalToGlobal(const Geometry2D::Vec2& p_local, Geometry2D::Vec2* p_global)
{
	transformVectorLocalToGlobal(p_local, p_global);
	*p_global = *p_global + position;
}

void Agent::getCircleAndNominalVelocityGlobal(int circle_index, float time, Circle* circle, Vec2* velocity)
{
	Vec2 v_ref;
	environment->getReferenceVelocity(time, position, &v_ref);
	Vec2 v_ref_local;
	transformVectorGlobalToLocal(v_ref, &v_ref_local);
	Vec2 v_center_ref_local;
	this->transformReferenceVelocityToPointVelocity(v_ref_local, circles[circle_index].center, &v_center_ref_local);
	transformVectorLocalToGlobal(v_center_ref_local, velocity);
	Vec2 center_global;
	transformPositionLocalToGlobal(circles[circle_index].center, &center_global);
	*circle = Circle(center_global, circles[circle_index].radius);
}

void Agent::addGlobalCircleVelocityConstraint(int circle_index, const HalfPlane2& constraint)
{
	Vec2 normal_local;
	transformVectorGlobalToLocal(constraint.getNormal(), &normal_local);
	HalfPlane2 constraint_local(normal_local, constraint.getOffset());
	HalfPlane2 constraint_ref;
	this->transformVelocityConstraintToReferencePoint(constraint_local, circles[circle_index].center, &constraint_ref);
	constraints.push_back(constraint_ref);
}

void Agent::getConstrainedOptimizedVelocity(float time, Vec2* linear_velocity, float* angular_velocity)
{
	Vec2 v_ref;
	environment->getReferenceVelocity(time, position, &v_ref);
	Vec2 v_ref_local;
	transformVectorGlobalToLocal(v_ref, &v_ref_local);
	Vec2 v_reference_point_ref_local;
	this->transformReferenceVelocityToPointVelocity(v_ref_local, reference_point, &v_reference_point_ref_local);
	// shift and scale constraints
	float v_max = 5.f;
	constraints.push_back(HalfPlane2(Vec2(1.f, 0.f), v_max));
	constraints.push_back(HalfPlane2(Vec2(-1.f, 0.f), v_max));
	constraints.push_back(HalfPlane2(Vec2(0.f, 1.f), v_max));
	constraints.push_back(HalfPlane2(Vec2(0.f, -1.f), v_max));

	std::vector<HalfPlane2> constraints_normalized = constraints;
	Vec2 shift = -1.f*v_reference_point_ref_local;
	float scaling = 0.5f/(shift.norm() + v_max + 0.01f);
	for (auto& h : constraints_normalized)
		h.shift(shift).rescale(scaling);

	// solve the normalized problem
	bool feasible;
	Vec2 reference_point_velocity_solution;
	try
	{
		Vec2 scaled_shifted_solution = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(constraints_normalized);
		feasible = true;
		reference_point_velocity_solution = scaled_shifted_solution/scaling - shift;
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		feasible = false;
		reference_point_velocity_solution = Geometry2D::Vec2(0.f, 0.f);
		std::cout << "Infeasible constraints" << std::endl;
	}
	Vec2 linear_velocity_local;
	this->transformRefPointVelocityToLinearAngular(reference_point_velocity_solution, &linear_velocity_local, angular_velocity);
	transformVectorLocalToGlobal(linear_velocity_local, linear_velocity);
}

void Environment::getReferenceVelocity(float time, const Geometry2D::Vec2& position, Geometry2D::Vec2* velocity) const
{
	*velocity = speed*Vec2(std::cos(orientation), std::sin(orientation));
}

float orientationReferenceTracking(float orientation, float orientation_reference, float gain)
{
	while (orientation_reference - orientation > M_PI)
		orientation += 2.f*M_PI;
	while (orientation_reference - orientation < -M_PI)
		orientation -= 2.f*M_PI;
	return gain*(orientation_reference - orientation);
}

void DifferentialDriveAgent::transformReferenceVelocityToPointVelocity(const Geometry2D::Vec2& v_ref,
	const Geometry2D::Vec2& point, Geometry2D::Vec2* v_point_ref)
{
	/*
	float orientation_ref = std::atan2(v_ref.y, v_ref.x);
	float gain = 0.1f;
	float angular_v_ref = orientationReferenceTracking(orientation + M_PI/2.f, orientation_ref, gain);
	float linear_v_ref = v_ref.norm();//v_ref.y;
	*/

	float linear_v_ref = reference_point.x/reference_point.y*v_ref.x + v_ref.y;
	float angular_v_ref = -v_ref.x/reference_point.y;

	*v_point_ref = Vec2(-angular_v_ref*point.y, linear_v_ref + angular_v_ref*point.x);
}

void DifferentialDriveAgent::transformRefPointVelocityToLinearAngular(const Geometry2D::Vec2& v_p_ref,
	 Geometry2D::Vec2* linear, float* angular)
{
	*linear = Vec2(0.f, reference_point.x/reference_point.y*v_p_ref.x + v_p_ref.y);
	*angular = -v_p_ref.x/reference_point.y;
}

void DifferentialDriveAgent::transformVelocityConstraintToReferencePoint(const HalfPlane2& constraint_local,
	const Vec2& p_original, HalfPlane2* new_constraint_local)
{
	Vec2 n(constraint_local.getNormal());
	Vec2 n_new(n.x*p_original.y/reference_point.y + n.y*(reference_point.x-p_original.x)/reference_point.y, n.y);
	*new_constraint_local = HalfPlane2(n_new, 1.f).rescale(constraint_local.getOffset()/n_new.norm());
}