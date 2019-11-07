#include "geometry.hpp"
#include "distance_minimizer.hpp"
#include "gui.hpp"

#include <iostream>
#include <vector>
#include <cmath>

#include <chrono>

#include <algorithm>
#include <random>

//#include <random>
//#include <limits>
/*
void generateNRandomConstraints(int n, std::vector<Geometry2D::HalfPlane2>* constraints)
{
	//std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(0);//gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(-0.0, 2.0);
	for (int n = 0; n < 10; ++n) {
		// Use dis to transform the random unsigned int generated by gen into a 
		// double in [1, 2). Each call to dis(gen) generates a new random double
		std::cout << dis(gen) << ' ';
	}
	std::cout << '\n';
}*/

using Geometry2D::HalfPlane2;
using Geometry2D::Vec2;
/*
std::uint_fast64_t factorial(std::vector<HalfPlane2>::size_type n)
{
	std::uint_fast64_t f = 1; // result type of std::mt19937_64
	std::uint_fast64_t max_value = std::numeric_limits<std::uint_fast64_t>::max();
	for (std::vector<HalfPlane2>::size_type i = n; i > 0; --i)
	{
		if (f < max_value/i)
			f *= i;
		else
			throw "Overflow"
	}
	return f;
}

void permuteConstraintsRandomly(const std::vector<HalfPlane2>& old_constraints,
	std::vector<HalfPlane2>* new_constraints)
{
	// N slots for N elements --> there are N! different ways to match slots with elements
	std::vector<HalfPlane2>::size_type n = constraints.size();
	std::uint_fast64_t fn = factorial(n); // result type of std::mt19937_64

	std::mt19937_64 gen(0);
	if (gen.max() < fn)
		throw "Overflow"
	std::uint_fast64_t draw = gen();

	for (std::vector<HalfPlane2>::size_type i = 0; i != n; ++i)
	{
		std::uint_fast64_t branch_size = factorial(n-i-1);
		for (std::vector<HalfPlane2>::size_type j = 0; j < n-i)

	}

}*/

void generate20NConstraints(int n, std::vector<HalfPlane2>* constraints)
{
	Vec2 shift(-0.3,-0.3);
	for (int j = 0; j < 20; j++)
	{	
		for (int i = 0; i < n; ++i)
		{
			Vec2 normal(std::cos(i*6.282/n), std::sin(i*6.282/n));
			constraints->push_back(HalfPlane2(normal, 0.2+0.05*(19-j) - normal.dot(shift)));
		}
	}
}

int main()
{
	Geometry2D::Vec2 nominal_point(-0.1, -0.1);

	// set up and solve a distance minimization problem
	{	
		std::vector<Geometry2D::HalfPlane2> constraints;
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5/10.f-nominal_point.dot(Geometry2D::Vec2(1.0, 0.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, 1.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-0.5, -1.0), 0.4/10.f-nominal_point.dot(Geometry2D::Vec2(-0.5, -1.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), 0.05/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, -1.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), -0.02/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, -1.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.12/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, 1.0).normalized())));
		constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-1.5, -0.5), -0.02/10.f-nominal_point.dot(Geometry2D::Vec2(-1.5, -0.5).normalized())));
		//constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), -0.1/10.f-nominal_point.dot(Geometry2D::Vec2(1.0, 0.0).normalized())));
		//Geometry2D::Vec2 v2(1.0, -0.5);
		//constraints.push_back(Geometry2D::HalfPlane2(v2/v2.norm(), -0.95));

		nominal_point = Geometry2D::Vec2(0.f, 0.f);
	}

	std::vector<Geometry2D::HalfPlane2> new_constraints;
	generate20NConstraints(50, &new_constraints);

	//auto rng = std::default_random_engine {};
	//for (int i = 0; i < 680; i++)
	//	std::shuffle(std::begin(new_constraints), std::end(new_constraints), rng);

	std::chrono::high_resolution_clock::time_point t1, t2;
	try
	{
		t1 = std::chrono::high_resolution_clock::now();

		Geometry2D::Vec2 p_s = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(new_constraints);

		t2 = std::chrono::high_resolution_clock::now();

		auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

		std::cout << "The minimization took " << duration << " microseconds." << std::endl;

		std::cout << "Solution: (" << p_s.x << ", " << p_s.y << ")"<< std::endl;
		// draw the problem and the solution in a GUI
		std::vector<Geometry2D::Vec2> points;
		points.push_back(p_s);
		points.push_back(nominal_point);
		//points.push_back(Vec2(0.3,0.3));

		GUI gui("Minimum Norm Point", 0.6);
		gui.halfplanes = &new_constraints;
		gui.points = &points;
		Window::sdlColor green;
		green.r = green.b = 0;
		green.g = 255;
		gui.points_colors.push_back(green);
		gui.points_colors.push_back(green);
		gui.blockingShowUntilClosed();
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		std::cout << "IncrementalDistanceMinimization: infeasible, code " << e.code << std::endl;
	}

	return 0;
}