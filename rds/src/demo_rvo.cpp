#include "simulation.hpp"
#include "gui.hpp"
#include "geometry.hpp"

#include "simulate_rvo_rds.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <chrono>
#include <thread>

#include <random>

#include <iostream>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;

struct GoalEnvironment : public Environment
{
	GoalEnvironment(const Vec2& goal, float speed) : Environment(), goal(goal)
	{
		this->speed = speed;
	}
	virtual void getReferenceVelocity(float time, const Geometry2D::Vec2& position, Geometry2D::Vec2* velocity) const
	{
		*velocity = speed*(goal - position)/std::max(1.f, (goal - position).norm());
	}
	Vec2 goal;
};

void simulate_while_displaying(Simulation* s, const char* title = "RVO", float window_size = 6.f, bool rds = false, bool show_trajectory = false, const Vec2* goal = 0)
{
	GUI gui_constraints(title, 6.f);
	std::vector<HalfPlane2> ref_p_constraints;
	gui_constraints.halfplanes = &ref_p_constraints;

	GUI gui_work_space(title, window_size);
	std::vector<Circle> work_space_circles;
	gui_work_space.circles = &work_space_circles;
	std::vector<Vec2> work_space_points;
	if (show_trajectory)
		gui_work_space.points = &work_space_points;

	GuiColor green, red;
	green.g = 255;
	green.r = green.b = 0;
	red.r = 255;
	red.g = red.b = 0;

	if (goal)
	{
		work_space_points.push_back(*goal);
		gui_work_space.points_colors.push_back(red);
	}

	std::vector<Vec2> robot_trajectory;

	std::chrono::milliseconds gui_cycle_time(50);
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	int n_iterations = 5;
	double dt = 0.01;
	while ((gui_work_space.update() == 0) | (gui_constraints.update() == 0))
	{
		if (!rds)
		{
			for (int i = 0; i < n_iterations; i++)
				s->stepEuler(dt);
		}
		else
		{
			SimulateRvoRds s_rvo_rds(s);
			s_rvo_rds.tau = s->rvo.getTau();
			s_rvo_rds.delta = s->rvo.getDelta();
			for (int i = 0; i < n_iterations; i++)
				s_rvo_rds.stepEuler(dt);
		}
		// transfer the positions to the gui variables
		{
			work_space_circles.resize(0);
			for (auto& a : s->agents)
			{
				for (std::vector<Circle>::size_type i = 0; i != a->circles.size(); i++)
				{
					Circle c;
					Vec2 v;
					a->getCircleAndNominalVelocityGlobal(i, 0.f, &c, &v);
					work_space_circles.push_back(c);
				}
			}
			gui_work_space.circles_colors.resize(0);
			for (auto& c : s->agents[0]->circles)
				gui_work_space.circles_colors.push_back(green);

			ref_p_constraints.resize(0);
			if (s->agents.size() > 0)
			{
				for(auto& h : s->agents[0]->constraints)
					ref_p_constraints.push_back(h);
			}

			if (show_trajectory)
				work_space_points.push_back(s->agents[0]->position);

			if (robot_trajectory.size() < 100000)
				robot_trajectory.push_back(s->agents[0]->position);
		}

		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}

	// evaluate robot trajectory
	double average_speed = 0.f;
	double average_acceleration = 0.f;
	for (std::vector<Vec2>::size_type i = 2; i != robot_trajectory.size(); i++)
	{
		float speed = (robot_trajectory[i] - robot_trajectory[i-1]).norm()/(n_iterations*dt);
		average_speed = (average_speed*i + speed)/(i + 1);
		Vec2 velocity_1 = (robot_trajectory[i-1] - robot_trajectory[i-2])/(n_iterations*dt);
		Vec2 velocity_2 = (robot_trajectory[i] - robot_trajectory[i-1])/(n_iterations*dt);
		float acceleration = (velocity_2 - velocity_1).norm()/(n_iterations*dt);
		average_acceleration = (average_acceleration*i + acceleration)/(i + 1);
	}
	std::cout << "Robot average speed = " << average_speed << std::endl;
	std::cout << "Robot average acceleration = " << average_acceleration << std::endl;
}

int main(int argc, char** argv)
{
	int simu_index = 0;
	if (argc > 1)
		simu_index = std::stoi(argv[1]);

	switch (simu_index)
	{
		case 0:
		{
			Agent a1, a2, a3;
			Simulation s(RVO(5.f, 0.f));
			s.agents.push_back(&a1);
			s.agents.push_back(&a2);
			s.agents.push_back(&a3);
	
			a1.position = Vec2(-1.5f, 0.f);
			a2.position = Vec2(0.f, -1.f);
			a3.position = Vec2(2.5f, 1.75f);
	
			a1.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
			a2.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
			/*a2.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.25));
			a2.circles.push_back(Circle(Vec2(0.f, -0.4f), 0.25));
			a2.circles.push_back(Circle(Vec2(0.f, -0.6f), 0.25));
			a2.circles.push_back(Circle(Vec2(-0.075f, -0.4f), 0.25));
			//a2.circles.push_back(Circle(Vec2(-0.2f, -0.3f), 0.25));
			a2.circles.push_back(Circle(Vec2(-0.15f, -0.1f), 0.25));
			a2.circles.push_back(Circle(Vec2(0.075f, -0.4f), 0.25));
			//a2.circles.push_back(Circle(Vec2(0.2f, -0.3f), 0.25));
			a2.circles.push_back(Circle(Vec2(0.15f, -0.1f), 0.25));*/
			a2.circles.push_back(Circle(Vec2(0.f, -0.35f), 0.2));
	
			a3.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
	
			Environment e1, e2, e3;
			e1.orientation = 0.f;
			e1.speed = 0.2f;
			e2.orientation = M_PI/2.f;
			e2.speed = 0.2f;
			e3.orientation = M_PI;
			e3.speed = 0.2f;
			a1.environment = &e1;
			a2.environment = &e2;
			a3.environment = &e3;
	
			simulate_while_displaying(&s);
			if (argc > 2)
				break;
		}
		case 1:
		{
			Agent a1, a2, a3;
			DifferentialDriveAgent d;
			Simulation s(RVO(4.f, 0.f));
			s.agents.push_back(&d);
			s.agents.push_back(&a1);


			a1.position = Vec2(-1.5f, 0.f);
			a3.position = Vec2(2.5f, 1.75f);
			d.position = Vec2(0.f, -1.65f);
			d.orientation = 0.f;

			a1.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
			a3.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25));
			d.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			d.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));

			Environment e1, e2, e3;
			e1.orientation = 0.f;
			e1.speed = 0.2f;
			e2.orientation = M_PI/2.f;
			e2.speed = 0.2f;
			e3.orientation = M_PI;
			e3.speed = 0.2f*0.0;;
			a1.environment = &e1;
			a3.environment = &e3;
			d.environment = &e2;

			simulate_while_displaying(&s);

			if (argc > 2)
				break;
		}
		case 2:
		{
			Simulation s(RVO(5.f, 0.f));

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					pedestrian.position = Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);

			//static_agent.position = Vec2(5.f, 2.f);
			//s.agents.push_back(&static_agent);

			std::vector<Agent> crossing_crowd;
			Agent crossing_pedestrian;
			Environment e_crossing_pedestrian;
			e_crossing_pedestrian.orientation = M_PI/2.f;
			e_crossing_pedestrian.speed = 0.2f;
			crossing_pedestrian.environment = &e_crossing_pedestrian;
			crossing_pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			crossing_pedestrian.position = Vec2(0.f, -1.f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(-1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(0.f, -3.5f);
			crossing_crowd.push_back(crossing_pedestrian);

			for (auto& a : crossing_crowd)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO pedestrians only", 12.f);
			if (argc > 2)
				break;
		}
		case 3:
		{
			Simulation s(RVO(5.f, 0.f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(0.f, -2.f);
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = M_PI/2.f;
			e_robot.speed = 0.2f;
			robot.environment = &e_robot;

			s.agents.push_back(&robot);

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					pedestrian.position = Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO robot and pedestrians", 12.f, true);
			if (argc > 2)
				break;
		}
		case 4:
		{
			Simulation s(RVO(5.f, 0.f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(0.f, -2.f);
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = M_PI/2.f;
			e_robot.speed = 0.2f;
			robot.environment = &e_robot;

			s.agents.push_back(&robot);

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO robot and dense pedestrians", 12.f, true);
			if (argc > 2)
				break;
		}
		case 5:
		{
			Simulation s(RVO(5.f, 0.f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(0.f, -2.f);
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = M_PI/2.f;
			e_robot.speed = 0.2f;
			robot.environment = &e_robot;

			s.agents.push_back(&robot);

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);

			std::vector<Agent> crossing_crowd;
			Agent crossing_pedestrian;
			Environment e_crossing_pedestrian;
			e_crossing_pedestrian.orientation = M_PI/2.f;
			e_crossing_pedestrian.speed = 0.2f;
			crossing_pedestrian.environment = &e_crossing_pedestrian;
			crossing_pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			crossing_pedestrian.position = Vec2(0.f, -1.f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(-1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(-1.f, -3.5f);
			crossing_crowd.push_back(crossing_pedestrian);

			for (auto& a : crossing_crowd)
				s.agents.push_back(&a);

			//static_agent.position = Vec2(0.1f, 2.f);
			//s.agents.push_back(&static_agent);
			
			simulate_while_displaying(&s, "RVO robot and dense pedestrians on both axes", 12.f, false);
			if (argc > 2)
				break;
		}
		case 6:
		{
			Simulation s(RVO(5.f, 0.f));

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);

			std::vector<Agent> crossing_crowd;
			Agent crossing_pedestrian;
			Environment e_crossing_pedestrian;
			e_crossing_pedestrian.orientation = M_PI/2.f;
			e_crossing_pedestrian.speed = 0.2f;
			crossing_pedestrian.environment = &e_crossing_pedestrian;
			crossing_pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			crossing_pedestrian.position = Vec2(0.f, -1.f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(-1.5f, -1.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(-1.f, -3.5f);
			crossing_crowd.push_back(crossing_pedestrian);
			crossing_pedestrian.position = Vec2(0.f, -1.85f);
			crossing_crowd.push_back(crossing_pedestrian);

			for (auto& a : crossing_crowd)
				s.agents.push_back(&a);

			//static_agent.position = Vec2(0.1f, 2.f);
			//s.agents.push_back(&static_agent);
			
			simulate_while_displaying(&s, "Only dense pedestrians on both axes", 12.f, false);
			if (argc > 2)
				break;
		}
		case 7:
		{
			Simulation s(RVO(5.f, 0.05f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(-3.2f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = 0.f;
			e_robot.speed = 0.3f;
			robot.environment = &e_robot;

			s.agents.push_back(&robot);

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				if ((i > 7) && (i < 19))
					continue;
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}
			for (int i = 0; i < 30; i++)
			{
				if ((i > 9) && (i < 19))
					continue;
				static_agent.position = Vec2(-2.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(3.f - 1.f, -4.f + i*0.5 - 1.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}

			for (auto& a : crowd)
				s.agents.push_back(&a);

			//static_agent.position = Vec2(0.1f, 2.f);
			//s.agents.push_back(&static_agent);
			
			simulate_while_displaying(&s, "RVO robot and dense pedestrians 1D flow", 12.f, true);
			if (argc > 2)
				break;
		}
		case 8:
		{
			Simulation s(RVO(1.f, 0.05f));

			Agent a, b, c;

			float angle = 0.f;//M_PI/2.f + 10.f*M_PI/8.f;

			Environment e_a;
			e_a.orientation = angle;
			e_a.speed = 0.2f;
			a.environment = &e_a;
			a.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			a.position = Vec2(std::cos(angle + M_PI), std::sin(angle + M_PI));
			//s.agents.push_back(&a);

			Environment e_b;
			e_b.orientation = angle + M_PI;
			e_b.speed = 0.2f;
			b.environment = &e_b;
			b.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			b.position = Vec2(std::cos(angle), std::sin(angle));
			s.agents.push_back(&b); s.agents.push_back(&a);

			//static_agent.position = Vec2(0.1f, 2.f);
			//s.agents.push_back(&static_agent);
			
			simulate_while_displaying(&s, "RVO collision test", 12.f, false);
			if (argc > 2)
				break;
		}
		case 9:
		{
			Simulation s(RVO(1.f, 0.05f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(-3.2f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = 0.f;
			e_robot.speed = 0.3f;
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}
			for (auto& a : crowd)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO robot in 1D flow", 18.f, false, true);
			if (argc > 2)
				break;
		}
		case 10:
		{
			Simulation s(RVO(1.f, 0.05f));

			DifferentialDriveAgent robot;
			robot.position = Vec2(-3.2f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = 0.f;
			e_robot.speed = 0.3f;
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.75*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}
			for (auto& a : crowd)
				s.agents.push_back(&a);

			std::vector<Agent> walls;
			Agent static_agent;
			Environment e_static_agent;
			e_static_agent.orientation = 0.f;
			e_static_agent.speed = 0.f;
			static_agent.environment = &e_static_agent;
			static_agent.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));
			for (int i = 0; i < 30; i++)
			{
				static_agent.position = Vec2(-7.f + i*0.5, 0.f);
				walls.push_back(static_agent);
				static_agent.position = Vec2(-7.f + i*0.5, 4.f);
				walls.push_back(static_agent);
			}

			for (auto& a : walls)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO robot in constrained 1D flow", 18.f, false, true);
			if (argc > 2)
				break;
		}
		case 11:
		{
			Simulation s(RVO(1.f, 0.05f));

			DifferentialDriveAgent robot;
			robot.position = 0.5/0.75*Vec2(-3.2f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = 0.f;
			e_robot.speed = 0.3f;
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			e_pedestrian.speed = 0.2f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = 3; j < 6; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.5*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					crowd.push_back(pedestrian);
				}
			}
			for (auto& a : crowd)
				s.agents.push_back(&a);
			
			simulate_while_displaying(&s, "RVO robot in dense 1D flow", 18.f, false, true);
			if (argc > 2)
				break;
		}
		case 12:
		{
			std::default_random_engine generator(1);
			std::normal_distribution<double> distribution (0.0,1.0);

			Simulation s(RVO(1.f, 0.05f)); // tau = 1.f, 0.5f
			s.v_max = 1.8f;

			DifferentialDriveAgent robot;
			robot.position = 0.5/0.75*Vec2(-3.8f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			Environment e_robot;
			e_robot.orientation = 0.f;
			e_robot.speed = 1.5f;
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			std::vector<Environment> e_crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = -5; j < 9; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.5*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					e_pedestrian.speed = 1.2f + 0.5f*std::max(-1.f, std::min(1.f, float(distribution(generator))));
					e_crowd.push_back(e_pedestrian);
					crowd.push_back(pedestrian);
				}
			}
			int i = 0;
			for (auto& a : crowd)
			{
				a.environment = &e_crowd[i];
				i++;
				s.agents.push_back(&a);
			}

			for (auto& a : s.agents)
				a->position = a->position + Vec2(-5.f, 0.f);
			
			simulate_while_displaying(&s, "RVO robot in dense 1D flow with individual speed values", 18.f, true, true);
			if (argc > 2)
				break;
		}
		case 13:
		{
			std::default_random_engine generator(1);
			std::normal_distribution<double> distribution (0.0,1.0);

			Simulation s(RVO(0.75f, 0.05f)); // tau = 1.f, 0.5f
			s.v_max = 1.8f;

			Vec2 goal(7.f, 0.f);

			DifferentialDriveAgent robot;
			//robot.reference_point.y = 1.35f;
			robot.position = 0.5/0.75*Vec2(-3.8f, 1.6f);
			robot.orientation = -M_PI/2.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			GoalEnvironment e_robot(goal, 1.5f);
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			std::vector<Environment> e_crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = -5; j < 9; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.5*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					e_pedestrian.speed = 1.2f + 0.5f*std::max(-1.f, std::min(1.f, float(distribution(generator))));
					e_crowd.push_back(e_pedestrian);
					crowd.push_back(pedestrian);
				}
			}
			int i = 0;
			for (auto& a : crowd)
			{
				a.environment = &e_crowd[i];
				i++;
				s.agents.push_back(&a);
			}

			for (auto& a : s.agents)
				a->position = a->position + Vec2(-5.f, 0.f);
			
			simulate_while_displaying(&s, "Goal-driven in dense 1D flow with individual speed values", 18.f, false, true, &goal);
			if (argc > 2)
				break;
		}
		case 14:
		{
			std::default_random_engine generator(1);
			std::normal_distribution<double> distribution (0.0,1.0);

			Simulation s(RVO(0.75f, 0.05f)); // tau = 1.f, 0.5f
			s.v_max = 1.8f;

			Vec2 goal(0.f, 7.f);

			DifferentialDriveAgent robot;
			//robot.reference_point.y = 1.35f;
			robot.position = Vec2(2.f, -8.f);
			robot.orientation = -0.f;
			robot.circles.push_back(Circle(Vec2(0.f, 0.15f), 0.25));
			robot.circles.push_back(Circle(Vec2(0.f, -0.2f), 0.2));
			GoalEnvironment e_robot(goal, 1.5f);
			robot.environment = &e_robot;
			s.agents.push_back(&robot);

			std::vector<Agent> crowd;
			std::vector<Environment> e_crowd;
			Agent pedestrian;
			Environment e_pedestrian;
			e_pedestrian.orientation = 0.f;
			pedestrian.environment = &e_pedestrian;
			pedestrian.circles.push_back(Circle(Vec2(0.f, 0.f), 0.25f));

			for (int i = -6; i < 7; i++)
			{
				for (int j = -5; j < 9; j++)
				{
					if ((j == 4) && (i == 0))
						continue;
					pedestrian.position = 0.5*Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2),
						-5.f + j*11.f/9 + 2.3f);
					e_pedestrian.speed = 1.2f + 0.5f*std::max(-1.f, std::min(1.f, float(distribution(generator))));
					e_crowd.push_back(e_pedestrian);
					crowd.push_back(pedestrian);
				}
			}
			int i = 0;
			for (auto& a : crowd)
			{
				a.environment = &e_crowd[i];
				i++;
				s.agents.push_back(&a);
			}

			for (auto& a : s.agents)
				a->position = a->position + Vec2(-5.f, 0.f);
			
			simulate_while_displaying(&s, "Goal-driven traversing dense 1D flow with individual speed values", 18.f, true, true, &goal);
			if (argc > 2)
				break;
		}
	}
}

