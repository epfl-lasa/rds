#ifndef CIRCULAR_CONSTRAINTS_HPP
#define CIRCULAR_CONSTRAINTS_HPP

#include "geometry.hpp"
#include "vw_limits.hpp"
#include <vector>

namespace Geometry2D
{
	struct CircularCorrectionLowerPoints
	{
		CircularCorrectionLowerPoints(const VWDiamond& vw_diamond_limits, float y_p);

		void createCircularConstraints(const HalfPlane2& constraint,
			float tau, std::vector<HalfPlane2>* circular_constraints);

		const VWDiamond vw_diamond_limits;
		const float y_p;

		const std::vector<HalfPlane2>& getConstraintsVelocityLimits() const { return constraints_velocity_limits; }
	private:
		std::vector<HalfPlane2> constraints_velocity_limits;
		Vec2 corner_left, corner_lower, corner_right, corner_upper;
	};

	struct CircularCorrection
	{
		CircularCorrection(const VWDiamond& vw_diamond_limits, float y_p);

		void createCircularConstraints(const HalfPlane2& constraint,
			float tau, std::vector<HalfPlane2>* circular_constraints);

		const VWDiamond vw_diamond_limits;
		const float y_p;

		const std::vector<HalfPlane2>& getConstraintsVelocityLimits() const { return constraints_velocity_limits; }
	private:
		std::vector<HalfPlane2> constraints_velocity_limits;
		CircularCorrectionLowerPoints circular_correction_lower_points;
	};

	struct CircularCorrectionCapsuleCap
	{
		CircularCorrectionCapsuleCap(float y_center_front,
			float y_center_back, const VWDiamond& vw_diamond_limits);

		void createCircularConstraints(const HalfPlane2& constraint,
			float tau, std::vector<HalfPlane2>* circular_constraints);	

		const std::vector<HalfPlane2>& getConstraintsVelocityLimits() const;

		const float y_center_front, y_center_back;
		const VWDiamond vw_diamond_limits;

		struct CircularCorrectionCapException { };
	private:
		CircularCorrection circular_correction;
	};
}

#endif