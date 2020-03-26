#ifndef CROWD_TRAJECTORY_HPP
#define CROWD_TRAJECTORY_HPP

#include "geometry.hpp"
#include "spline.h" // from https://kluge.in-chemnitz.de/opensource/spline/

struct CrowdTrajectory
{
	CrowdTrajectory(const char* data_file_name, float frame_rate);

	struct Knot
	{
		Knot () : p(0.f, 0.f), t(0.f) { }
		Knot(const Geometry2D::Vec2& p, float t) : p(p), t(t) { }
		Geometry2D::Vec2 p;
		float t;
	};

	unsigned int getNumSplines() const { return m_splines_data.size(); }

	void getPedestrianPositionAtTime(unsigned int i, float t, Geometry2D::Vec2* p);

	const std::vector<std::vector<Knot> >& getSplinesData()const { return m_splines_data; }

private:
	std::vector<std::vector<Knot> > m_splines_data;
	const float m_frame_rate;
	std::vector<tk::spline> m_x_splines, m_y_splines;
};

#endif