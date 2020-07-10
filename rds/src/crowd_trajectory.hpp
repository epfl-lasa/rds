#ifndef CROWD_TRAJECTORY_HPP
#define CROWD_TRAJECTORY_HPP

#include "geometry.hpp"
#include "../../spline/spline.h" // from https://github.com/ttk592/spline

struct CrowdTrajectory
{
	CrowdTrajectory(const char* data_file_name, float frame_rate, float scaling);

	CrowdTrajectory() : m_time_shift(0.f), m_duration(-1.f), m_deceleration_period(1.f) { };

	struct Knot
	{
		Knot () : p(0.f, 0.f), t(0.f) { }
		Knot(const Geometry2D::Vec2& p, float t) : p(p), t(t) { }
		Geometry2D::Vec2 p;
		float t;
	};

	void addPedestrianTrajectory(const std::vector<Knot>& spline_data);

	void removePedestrian(unsigned int i);

	unsigned int getNumSplines() const { return m_splines_data.size(); }

	void getPedestrianPositionAtTime(unsigned int i, float t, Geometry2D::Vec2* p) const;

	void getPedestrianVelocityAtTime(unsigned int i, float t, Geometry2D::Vec2* v) const;

	const std::vector<std::vector<Knot> >& getSplinesData() const { return m_splines_data; }

	float m_time_shift, m_duration, m_deceleration_period;

private:
	std::vector<std::vector<Knot> > m_splines_data;
	std::vector<tk::spline> m_x_splines, m_y_splines;
};

#endif
