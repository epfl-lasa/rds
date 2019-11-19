#ifndef DIFFERENTIAL_DRIVE_KINEMATICS_HPP
#define DIFFERENTIAL_DRIVE_KINEMATICS_HPP

#include "geometry.hpp"

namespace RDS
{
	struct VelocityCommandBoxLimits
	{
		float min_linear;
		float min_angular;
		float max_linear;
		float max_angular;
	};

	struct VelocityCommandHexagonLimits
	{
		float min_linear; // < zero
		float max_linear; // > zero
		float absolute_angular_at_min_linear; // >= zero
		float absolute_angular_at_zero_linear; // > zero
		float absolute_angular_at_max_linear; // >= zero
	};

	struct SingularityException { };

	struct VelocityCommand
	{
		VelocityCommand()
			: linear(0.f)
			, angular(0.f)
		{ }

		VelocityCommand(float linear, float angular)
			: linear(linear)
			, angular(angular)
		{ }

		VelocityCommand(Geometry2D::Vec2 point, Geometry2D::Vec2 point_velocity)
		{
			if ((point.y < 0.001f) && (point.y > -0.001f))
				throw SingularityException();
			linear = point.x/point.y*point_velocity.x + point_velocity.y;
			angular = -point_velocity.x/point.y;
		}

		Geometry2D::Vec2 pointVelocity(const Geometry2D::Vec2& point) const
		{
			return Geometry2D::Vec2(-angular*point.y, linear + angular*point.x);
		}

		float linear;
		float angular;
	};

	struct PointVelocityConstraint
	{
		PointVelocityConstraint() { }

		PointVelocityConstraint(Geometry2D::Vec2 p, Geometry2D::HalfPlane2 h)
			: p(p)
			, h(h)
		{ }

		Geometry2D::Vec2 p;
		Geometry2D::HalfPlane2 h;
	};

	inline Geometry2D::HalfPlane2 transformPointAVelocityConstraintToPointB(const PointVelocityConstraint& a,
		const Geometry2D::Vec2& p_b)
	{
		Geometry2D::Vec2 n_b(a.h.getNormal().x*a.p.y/p_b.y + a.h.getNormal().y*(p_b.x-a.p.x)/p_b.y, a.h.getNormal().y);
		return Geometry2D::HalfPlane2(n_b, 1.f).rescale(a.h.getOffset()/n_b.norm());
	}

	inline Geometry2D::HalfPlane2 transformPointVelocityConstraintToCommandConstraint(const PointVelocityConstraint& pvc,
		float linear_normalization, float angular_normalization)
	{
		Geometry2D::Vec2 n_unnormalized(pvc.h.getNormal().y*linear_normalization,
			(-pvc.p.y*pvc.h.getNormal().x + pvc.p.x*pvc.h.getNormal().y)*angular_normalization);
		return Geometry2D::HalfPlane2(n_unnormalized, 1.f).rescale(pvc.h.getOffset()/n_unnormalized.norm());
	}
}

#endif