#include "simulator.hpp"
#include "gui.hpp"

#include <chrono>
#include <thread>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

using AdditionalPrimitives2D::Circle;
using Geometry2D::Vec2;
using RDS::VelocityCommand;

#include "random_angles.cpp"

void simulate_while_displaying(RDS::Simulator* simulator, const char* title = "Robot Motion")
{
	RDS::Simulator& simu = *simulator;
	GUI gui_work_space(title, 12.f);
	std::vector<Geometry2D::Vec2> work_space_points;
	//std::vector<AdditionalPrimitives2D::Arrow> work_space_arrows;
	std::vector<AdditionalPrimitives2D::Circle> work_space_circles;
	gui_work_space.points = &work_space_points;
	gui_work_space.circles = &work_space_circles;
	//gui_work_space.arrows = &work_space_arrows;

	GuiColor green, blue, red;
	green.r = green.b = 0;
	green.g = 255;
	blue.r = blue.g = 0;
	blue.b = 255;
	red.g = red.b = 0;
	red.r = 255;

	std::chrono::milliseconds gui_cycle_time(50);
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	while (gui_work_space.update() == 0)
	{
		for (int i = 0; i < 5; i++)
			simu.stepEuler(0.002);
		// transfer the positions to the gui variables
		{
			float rxx = std::cos(simu.robot.orientation);
			float rxy = std::sin(simu.robot.orientation);
			float ryx = -rxy;
			float ryy = rxx;

			work_space_circles.resize(0);
			for (auto& ob : simu.obstacles)
				work_space_circles.push_back(Circle(ob.position, ob.radius));

			for (auto& rs : simu.robot.shape)
			{
				Vec2 center_global = Vec2(rxx*rs.center.x + ryx*rs.center.y,
					rxy*rs.center.x + ryy*rs.center.y) + simu.robot.position;
				work_space_circles.push_back(Circle(center_global, rs.radius));
			}

			//work_space_points.resize(0);
			//work_space_points.push_back()
		}

		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}
}

float angleToPlus180Minus180(float angle)
{
	while (angle < -M_PI)
		angle += 2.f*M_PI;
	while (angle > M_PI)
		angle -= 2.f*M_PI;
	return angle;
}

float orientationReferenceTracking(float orientation, float orientation_reference, float gain)
{
	while (orientation_reference - orientation > M_PI)
		orientation += 2.f*M_PI;
	while (orientation_reference - orientation < -M_PI)
		orientation -= 2.f*M_PI;
	return gain*(orientation_reference - orientation);
}

float actifun(float time, float time_1, float time_2)
{
	if (time < time_1)
		return 0.f;
	else if (time > time_2)
		return 1.f;
	else
		return (time - time_1)/(time_2 - time_1);
}

float deactifun(float time, float time_1, float time_2)
{
	return 1.f - actifun(time, time_1, time_2);
}

float winfun(float time, float time_1, float time_2, float time_3, float time_4)
{
	return actifun(time, time_1, time_2) - actifun(time, time_3, time_4);
}

int main(int argc, char** argv)
{
	std::vector<Circle> robot_shape;
	robot_shape.push_back(Circle(Vec2(0.f, 0.056f), 0.4f));
	robot_shape.push_back(Circle(Vec2(0.f, -0.517), 0.3f));

	int simu_index = 0;
	if (argc > 1)
		simu_index = std::stoi(argv[1]);

	switch (simu_index)
	{
		case 0:
		{
			RDS::Simulator simu(
				[](float time, const Vec2& position, float orientation) {return VelocityCommand(0.5f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -3.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {
					float t = time/6.f-0.8f;
					return Vec2(-0.6f*std::exp(-t*t)*t, 1.f);},
				Vec2(0.0f, -6.f),
				0.25f));

			simulate_while_displaying(&simu, "Robot continues despite overtaking obstacle.");
			if (argc > 2)
				break;
		}
		case 1:
		{
			RDS::Simulator simu1(
				[](float time, const Vec2& position, float orientation) {return VelocityCommand(1.0f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -3.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu1.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {
					float t = time/4.f-1.0f;
					return Vec2(-0.6f*std::exp(-t*t)*t, 1.5f);},
				Vec2(0.0f, -6.f),
				0.25f));

			simulate_while_displaying(&simu1, "Robot stops due to closely overtaking obstacle.");
			if (argc > 2)
				break;
		}
		case 2:
		{
			RDS::Simulator simu1_2(
				[](float time, const Vec2& position, float orientation) {return VelocityCommand(1.f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -3.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu1_2.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {
					float t = time/4.f-1.0f;
					return Vec2(-2.8f*std::exp(-t*t)*t, 1.5f);},
				Vec2(0.0f, -6.f),
				0.25f));

			simulate_while_displaying(&simu1_2, "Robot deflects due to laterally approaching obstacle. See case 16 for a fix.");
			if (argc > 2)
				break;
		}
		case 3:
		{
			RDS::Simulator simu2(
				[](float time, const Vec2& position, float orientation) {return VelocityCommand(1.5f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -1.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu2.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(1.5f, 1.f),
				0.25f));

			simu2.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(-0.1f, 1.5f),
				0.25f));
			simulate_while_displaying(&simu2, "Robot motion among static obstacles.");	
			if (argc > 2)
				break;
		}
		case 4:
		{
			RDS::Simulator simu2_1(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = -M_PI/4.f*winfun(time, 3.f, 3.25f, 5.5f, 5.75f);
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 10.f));}, // nominal control law
				robot_shape,
				Vec2(0.f, -1.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu2_1.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(1.5f, 1.f),
				0.25f));

			simu2_1.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(-0.1f, 1.5f),
				0.25f));

			simulate_while_displaying(&simu2_1, "With orientation reference tracking among static obstacles.");		
			if (argc > 2)
				break;
		}
		case 5:
		{
			RDS::Simulator simu2_2(
				[](float time, const Vec2& position, float orientation) { 
					return VelocityCommand(0.5f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -1.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			for (int i = 0; i < 6; i++)
			{
				simu2_2.obstacles.push_back(RDS::Simulator::Obstacle(
					[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
					Vec2(-0.2f-i*0.5f, 1.5f),
					0.25f));

				simu2_2.obstacles.push_back(RDS::Simulator::Obstacle(
					[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
					Vec2(0.5f-0.2*i, 3.5f),
					0.25f));

				simu2_2.obstacles.push_back(RDS::Simulator::Obstacle(
					[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
					Vec2(1.5f, 1.5f+i*0.2f),
					0.25f));

				simu2_2.obstacles.push_back(RDS::Simulator::Obstacle(
					[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
					Vec2(1.5f-i*0.15f, 2.6f+i*0.15f),
					0.25f));
			}

			simulate_while_displaying(&simu2_2, "Getting stuck in a curved corridor.");
			if (argc > 2)
				break;
		}
		case 6:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					return VelocityCommand(0.5f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.3f, -6.f), // initial position
				-0.5f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
						Vec2(-5.f + i*10.f/6, -5.f + j*10.f/6),
						0.25f));
				}
			}
			simulate_while_displaying(&simu_x, "Getting stuck among evenly spaced pillars.");
			if (argc > 2)
				break;
		}
		case 7:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					return VelocityCommand(0.5f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				-0.5f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
						Vec2(-5.f + i*11.f/6, -5.f + j*11.f/6),
						0.25f));
				}
			}
			simulate_while_displaying(&simu_x, "Forward through evenly spaced pillars.");
			if (argc > 2)
				break;
		}
		case 8:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, std::sin(time)*std::sin(time)*orientationReferenceTracking(orientation, orientation_ref, 10.f));}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				-0.5f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
						Vec2(-5.f + i*11.f/6, -5.f + j*11.f/6),
						0.25f));
				}
			}
			simulate_while_displaying(&simu_x, "With on/off orientation control through evenly spaced pillars.");
			if (argc > 2)
				break;
		}
		case 9:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				-0.5f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
						0.25f));
				}
			}
			simulate_while_displaying(&simu_x, "With orientation control through alternating pillars.");
			if (argc > 2)
				break;
		}
		case 10:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					return VelocityCommand(1.25f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu_x.use_orca_style = true;

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(0.f, 0.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.65f, 0.f);},
				Vec2(-3.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(1.f, 0.5f);},
				Vec2(-6.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.5f, 1.2f);},
				Vec2(-5.f, -10.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(-3.5f, -5.5f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(2.f, -2.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(-2.f, 2.f),
				0.25f));

			simulate_while_displaying(&simu_x, "Robot moving through reactive dynamic obstacles.");
			if (argc > 2)
				break;
		}
		case 11:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu_x.use_orca_style = true;

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(0.f, 0.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.65f, 0.f);},
				Vec2(-3.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(1.f, 0.5f);},
				Vec2(-6.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.5f, 1.2f);},
				Vec2(-5.f, -10.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(-3.5f, -5.5f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(2.f, -2.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(-2.f, 2.f),
				0.25f));

			simulate_while_displaying(&simu_x, "With orientation control through reactive dynamic obstacles. Compare case 20.");
			if (argc > 2)
				break;
		}
		case 12:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 10.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f)); // previous command

			simu_x.use_orca_style = true;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					if (j > 2)
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, -1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));
					}
					else
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, 1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));	
					}
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through crowd.");
			if (argc > 2)
				break;
		}
		case 13:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 4.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f); // rds_tau

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 1.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					if (j > 2)
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, -1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));
					}
					else
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, 1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));	
					}
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through crowd with smaller taus.");
			if (argc > 2)
				break;
		}
		case 14:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, std::sin(time)*std::sin(time)*orientationReferenceTracking(orientation, orientation_ref, 10.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f); // rds_tau

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 1.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					if (j > 2)
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, -1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));
					}
					else
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, 1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));	
					}
				}
			}

			simulate_while_displaying(&simu_x, "Robot with on/off orientation control moving through crowd with smaller taus.");
			if (argc > 2)
				break;
		}
		case 15:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 4.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f, // rds_tau
				true); // rds unilateral velocity shift

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 1.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					if (j > 2)
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, -1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));
					}
					else
					{
						simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
							[](float time, const Vec2& position) {return Vec2();},
							Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6),
							0.25f,
							true,
							Vec2(0.f, 1.f) + 0.5*Vec2(std::cos(random_angles[i*7 + j]),
								std::sin(random_angles[i*7 + j]))));	
					}
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through crowd with smaller taus and unilateral velocity shifting.");
			if (argc > 2)
				break;
		}
		case 16:
		{
			RDS::Simulator simu1_2(
				[](float time, const Vec2& position, float orientation) {return VelocityCommand(1.f, 0.f);}, // nominal control law
				robot_shape,
				Vec2(0.f, -3.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				2.f, // rds_tau
				true); // rds unilateral velocity shift

			simu1_2.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {
					float t = time/4.f-1.0f;
					return Vec2(-2.8f*std::exp(-t*t)*t, 1.5f);},
				Vec2(0.0f, -6.f),
				0.25f));

			simulate_while_displaying(&simu1_2, "Robot does not deflect despite laterally approaching obstacle because of unilateral velocity shifting.");		
			if (argc > 2)
				break;
		}
		case 17:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.3f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f-6.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f, // rds_tau
				true); // rds unilateral velocity shift

			simu_x.use_velocities = false;

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 1.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2();},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6-6.f),
						0.25f,
						true,
						Vec2(0.f, 1.3f)));
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through in 1D flow without using velocities.");
			if (argc > 2)
				break;
		}
		case 18:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.3f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f, -3.1f-6.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f, // rds_tau
				true); // rds unilateral velocity shift

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 1.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2();},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2), -5.f + j*11.f/6-6.f),
						0.25f,
						true,
						Vec2(0.f, 1.3f)));
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through in 1D flow using velocities (unilateral shifting).");
			if (argc > 2)
				break;
		}
		case 19:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(-1.1f- 3.f, -3.1f-12.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				1.f, // rds_tau
				true); // rds unilateral velocity shift

			simu_x.use_orca_style = true;
			simu_x.tau_orca_style = 2.f;

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					if (j == 1 && i == 2)
						continue;
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.3f*std::sin(time), 1.3f);},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2)- 3.f, -5.f + j*11.f/6-12.f),
						0.25f,
						false,
						Vec2(0.f, 1.3f)));
				}
			}

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(1.3f, 0.3f*std::sin(2.5f*time-3.f));},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2)-12.f, -5.f + j*11.f/6 + 3.f),
						0.25f,
						false,
						Vec2(1.3f, 0.f)));
				}
			}

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(-1.3f, 0.3f*std::sin(1.5f*time-1.5f));},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2)+12.f, -5.f + j*11.f/6 - 3.f),
						0.25f,
						false,
						Vec2(-1.3f, 0.f)));
				}
			}

			for (int i = 0; i < 7; i++)
			{
				for (int j = 0; j < 7; j++)
				{
					simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
						[](float time, const Vec2& position) {return Vec2(0.3f*std::sin(0.5f*time-4.5), -1.3f);},
						Vec2(-5.f + i*11.f/6 + 11.f/12.f*(j%2) + 3.f, -5.f + j*11.f/6+12.f),
						0.25f,
						false,
						Vec2(0.f, -1.3f)));
				}
			}

			simulate_while_displaying(&simu_x, "Robot with orientation control moving through in cross flow using velocities (unilateral shifting).");
			if (argc > 2)
				break;
		}
		case 20:
		{
			RDS::Simulator simu_x(
				[](float time, const Vec2& position, float orientation) { 
					float orientation_ref = 0.f;
					return VelocityCommand(1.5f, orientationReferenceTracking(orientation, orientation_ref, 1.f));}, // nominal control law
				robot_shape,
				Vec2(0.5f, -6.f), // initial position
				0.f, // initial orientation
				VelocityCommand(0.f, 0.f), // previous command
				2.f, // rds_tau
				true); // rds unilateral velocity shift

			simu_x.use_orca_style = true;

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(0.f, 0.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.65f, 0.f);},
				Vec2(-3.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(1.f, 0.5f);},
				Vec2(-6.f, -3.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.5f, 1.2f);},
				Vec2(-5.f, -10.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, 0.f);},
				Vec2(-3.5f, -5.5f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(2.f, -2.f),
				0.25f));

			simu_x.obstacles.push_back(RDS::Simulator::Obstacle(
				[](float time, const Vec2& position) {return Vec2(0.f, -1.f);},
				Vec2(-2.f, 2.f),
				0.25f));

			simulate_while_displaying(&simu_x, "With unilateral velocity shift and orientation control through reactive dynamic obstacles.");
			if (argc > 2)
				break;
		}
	}
}