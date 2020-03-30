#include "crowd_trajectory.hpp"
#include <fstream>
#include <string>
#include <iostream>

using Geometry2D::Vec2;

CrowdTrajectory::CrowdTrajectory(const char* data_file_name, float frame_rate)
	: m_frame_rate(frame_rate)
{
	std::ifstream data_file(data_file_name);
	float pos_x, pos_y;
	long int frame_i;
	std::string any_word;
	const int n_words_first_line = 6;
	const int n_words_spline_header = 6;
	const int n_words_after_data = 5;

	for (int i = 0; i < n_words_first_line; i++)
		data_file >> any_word;
	while (true)
	{
		std::vector<Knot> spline_data;
		int n_knots;
		if (!(data_file >> n_knots))
			break;
		for (int i = 0; i < n_words_spline_header -1; i++)
			data_file >> any_word;
		for (int j = 0; j < n_knots; j++)
		{
			data_file >> pos_x >> pos_y >> frame_i;
			for (int i = 0; i < n_words_after_data; i++)
				data_file >> any_word;
			spline_data.push_back(Knot(Vec2(pos_x, pos_y), frame_i/m_frame_rate));
		}
		m_splines_data.push_back(spline_data);
	}

	for (auto& d : m_splines_data)
	{
		std::vector<double> T, X, Y;
		for (auto& knot : d)
		{
			T.push_back(knot.t);
			X.push_back(knot.p.x);
			Y.push_back(knot.p.y);
		}
		tk::spline sx, sy;
		sx.set_points(T, X);
		sy.set_points(T, Y);
		m_x_splines.push_back(sx);
		m_y_splines.push_back(sy);
	}
}

void CrowdTrajectory::getPedestrianPositionAtTime(unsigned int i, float t,
	Geometry2D::Vec2* p) const
{
	p->x = m_x_splines[i](t);
	p->y = m_y_splines[i](t);
}

void CrowdTrajectory::getPedestrianVelocityAtTime(unsigned int i, float t,
	Geometry2D::Vec2* v) const
{
	v->x = m_x_splines[i].deriv(1, t);
	v->y = m_y_splines[i].deriv(1, t);
}