#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "geometry.hpp"
#include "RVO.hpp"

#include <vector>

struct Environment
{
	Environment() : orientation(0.f), speed(0.5f) { }
	virtual ~Environment() { }

	virtual void getReferenceVelocity(float time, const Geometry2D::Vec2& position, Geometry2D::Vec2* velocity) const;

	float orientation;
	float speed;
};

struct Agent
{
	Agent() : position(0.f, 0.f), orientation(0.f), reference_point(0.f, 0.f), environment(0) { }

	virtual ~Agent() { }

	void getCircleAndNominalVelocityGlobal(int circle_index, float time,
		AdditionalPrimitives2D::Circle* circle, Geometry2D::Vec2* velocity);
	void addGlobalCircleVelocityConstraint(int circle_index, const Geometry2D::HalfPlane2& constraint);
	void getConstrainedOptimizedVelocity(float time, Geometry2D::Vec2* linear_velocity,
		float* angular_velocity, float v_max);

	Geometry2D::Vec2 position;
	float orientation;
	std::vector<AdditionalPrimitives2D::Circle> circles;
	Geometry2D::Vec2 reference_point;
	const Environment* environment;
	std::vector<Geometry2D::HalfPlane2> constraints;

private:
	virtual void transformReferenceVelocityToPointVelocity(const Geometry2D::Vec2& v_ref,
		const Geometry2D::Vec2& point, Geometry2D::Vec2* v_point_ref)
	{
		*v_point_ref = v_ref;
	}

	virtual void transformRefPointVelocityToLinearAngular(const Geometry2D::Vec2& v_p_ref,
		 Geometry2D::Vec2* linear, float* angular)
	{
		*angular = 0.f;
		*linear = v_p_ref;
	}

	virtual void transformVelocityConstraintToReferencePoint(const Geometry2D::HalfPlane2& constraint_local,
		const Geometry2D::Vec2& p_original, Geometry2D::HalfPlane2* new_constraint_local)
	{
		*new_constraint_local = constraint_local;
	}

	void transformVectorGlobalToLocal(const Geometry2D::Vec2& v_global, Geometry2D::Vec2* v_local);
	void transformPositionGlobalToLocal(const Geometry2D::Vec2& p_global, Geometry2D::Vec2* p_local);
	void transformVectorLocalToGlobal(const Geometry2D::Vec2& v_local, Geometry2D::Vec2* v_global);
	void transformPositionLocalToGlobal(const Geometry2D::Vec2& p_local, Geometry2D::Vec2* p_global);
};

struct DifferentialDriveAgent : public Agent
{
	DifferentialDriveAgent() : Agent()
	{
		reference_point.y = 0.15f;
	}

private:
	virtual void transformReferenceVelocityToPointVelocity(const Geometry2D::Vec2& v_ref,
		const Geometry2D::Vec2& point, Geometry2D::Vec2* v_point_ref);
	virtual void transformRefPointVelocityToLinearAngular(const Geometry2D::Vec2& v_p_ref,
		 Geometry2D::Vec2* linear, float* angular);
	virtual void transformVelocityConstraintToReferencePoint(const Geometry2D::HalfPlane2& constraint_local,
		const Geometry2D::Vec2& p_original, Geometry2D::HalfPlane2* new_constraint_local);
};

struct Simulation
{
	Simulation(const RVO& rvo) : time(0.f), rvo(rvo), v_max(0.35f) { }

	void stepEuler(float dt);

	float time;
	std::vector<Agent*> agents;
	RVO rvo;
	float v_max;
};

#endif