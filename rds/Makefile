all: geometry gui demo_geo rds_core build/demo_hierarchy build/demo_hierarchy_rds_3 build/demo_rds_3 build/demo_rds_4 build/demo_eval_rds_4 build/demo_marr build/demo_rds_orca_simu build/blocks build/demo_crowd_trajectory build/demo_crowd_rds_orca_simu build/demo_art_crowd_rds_orca_simu build/simple_rds_orca_simu build/basic_ex_standing_pedestrian build/basic_ex_overtaking build/basic_ex_crossing qolo_rds build/baseline_crowd_rds_5 build/check_cvo# list only what is ready to build
	@echo "\033[92mDone with building RDS.\033[0m"

FLAGS = -std=c++11 -g -Wall

# Qolo-RDS-5
qolo_rds: build/lib_rds_5.a 

build/lib_rds_5.a: build/rds/.hi src/rds_5.cpp src/rds_5.hpp src/capsule.hpp src/capsule.cpp src/geometry.hpp src/distance_minimizer.cpp src/distance_minimizer.hpp src/RVO.cpp src/RVO.hpp src/cvo.hpp src/cvo.cpp src/vw_limits.hpp
	g++ $(FLAGS) -c src/rds_5.cpp -o build/rds_5.o
	g++ $(FLAGS) -c src/capsule.cpp -o build/capsule.o
	g++ $(FLAGS) -c src/RVO.cpp -o build/RVO.o
	g++ $(FLAGS) -c src/cvo.cpp -o build/cvo.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/lib_rds_5.a build/rds_5.o build/capsule.o build/RVO.o build/distance_minimizer.o build/cvo.o
	cp src/rds_5.hpp build/rds
	cp src/capsule.hpp build/rds
	cp src/geometry.hpp build/rds
	cp src/distance_minimizer.hpp build/rds
	cp src/RVO.hpp build/rds
	cp src/cvo.hpp build/rds
	cp src/vw_limits.hpp build/rds

# DEMOS

build/check_cvo: src/check_cvo.cpp src/cvo.hpp src/cvo.cpp src/geometry.hpp build/libgui.a
	g++ $(FLAGS) src/check_cvo.cpp src/cvo.cpp -Lbuild -lgui -lSDL2 -o build/check_cvo

build/baseline_crowd_rds_5: src/baseline_crowd_rds_5.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_5_orca_simulator.hpp src/rds_5_orca_simulator.cpp src/rds_5_agent.cpp src/rds_5_agent.hpp src/rds_5.cpp src/rds_5.hpp src/RVO.hpp src/RVO.cpp src/cvo.hpp src/cvo.cpp build/libgeometry.a
	g++ $(FLAGS) src/baseline_crowd_rds_5.cpp src/crowd_trajectory.cpp src/rds_5_orca_simulator.cpp src/rds_5_agent.cpp src/rds_5.cpp src/RVO.cpp src/cvo.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/baseline_crowd_rds_5

build/show_circular_constraints: src/show_circular_constraints.cpp src/circular_constraints.cpp src/circular_constraints.hpp src/geometry.hpp src/vw_limits.hpp build/libgui.a
	g++ $(FLAGS) src/show_circular_constraints.cpp src/circular_constraints.cpp -Lbuild -lgui -lSDL2 -o build/show_circular_constraints

build/basic_ex_crossing: src/basic_ex_crossing.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/basic_ex_crossing.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/basic_ex_crossing

build/basic_ex_overtaking: src/basic_ex_overtaking.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/basic_ex_overtaking.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/basic_ex_overtaking

build/basic_ex_standing_pedestrian: src/basic_ex_standing_pedestrian.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/basic_ex_standing_pedestrian.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/basic_ex_standing_pedestrian

build/simple_rds_orca_simu: src/simple_rds_orca_simu.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/simple_rds_orca_simu.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/simple_rds_orca_simu

build/demo_art_crowd_rds_orca_simu: src/demo_art_crowd_rds_orca_simu.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp build/libgeometry.a
	g++ $(FLAGS) src/demo_art_crowd_rds_orca_simu.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_art_crowd_rds_orca_simu

build/demo_crowd_rds_orca_simu: src/demo_crowd_rds_orca_simu.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp build/libgeometry.a
	g++ $(FLAGS) src/demo_crowd_rds_orca_simu.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_crowd_rds_orca_simu

build/demo_crowd_trajectory: src/demo_crowd_trajectory.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a
	g++ $(FLAGS) src/demo_crowd_trajectory.cpp src/crowd_trajectory.cpp -Lbuild -lgui -lSDL2 -o build/demo_crowd_trajectory

build/demo_rds_orca_simu: src/demo_rds_orca_simu.cpp src/rds_orca_simulator.hpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp src/crowd_trajectory.cpp src/crowd_trajectory.hpp build/libgui.a
	g++ $(FLAGS) src/demo_rds_orca_simu.cpp src/crowd_trajectory.cpp src/rds_orca_simulator.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_rds_orca_simu

build/demo_marr: src/demo_marr.cpp src/simulation_marr.cpp src/simulation_marr.hpp build/libgui.a
	g++ $(FLAGS) src/demo_marr.cpp src/simulation_marr.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_marr

build/demo_eval_rds_4: build/libgeometry.a build/libgui.a src/demo_eval_rds_4.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/demo_eval_rds_4.cpp src/rds_4_agent.cpp src/rds_4.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_eval_rds_4

build/demo_rds_4: build/libgeometry.a build/libgui.a src/demo_rds_4.cpp src/rds_4_agent.cpp src/rds_4_agent.hpp src/rds_4.cpp src/rds_4.hpp src/rvo_3_agent.cpp src/rvo_3_agent.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/demo_rds_4.cpp src/rds_4_agent.cpp src/rds_4.cpp src/rvo_3_agent.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_rds_4

build/demo_rds_3: build/libgeometry.a build/libgui.a src/demo_rds_3.cpp src/rds_3_agent.cpp src/rds_3_agent.hpp src/rds_3.cpp src/rds_3.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/demo_rds_3.cpp src/rds_3_agent.cpp src/rds_3.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_rds_3

build/demo_hierarchy_rds_3: build/libgeometry.a build/libgui.a src/demo_hierarchy_rds_3.cpp src/rds_3_agent.cpp src/rds_3_agent.hpp src/rds_3.cpp src/rds_3.hpp src/RVO.hpp src/RVO.cpp
	g++ $(FLAGS) src/demo_hierarchy_rds_3.cpp src/rds_3_agent.cpp src/rds_3.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_hierarchy_rds_3

build/demo_hierarchy: build/libgeometry.a build/libgui.a src/demo_hierarchy.cpp src/rds_2.cpp src/rds_2.hpp src/rds_2_agent.cpp src/rds_2_agent.hpp
	g++ $(FLAGS) src/demo_hierarchy.cpp src/rds_2_agent.cpp src/rds_2.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/demo_hierarchy

build/blocks: src/Blocks.cpp src/geometry.hpp src/rds_2.cpp src/rds_2.hpp src/rds_2_agent.cpp src/rds_2_agent.hpp build/libgui.a build/libgeometry.a
	g++ $(FLAGS) src/Blocks.cpp src/rds_2.cpp src/rds_2_agent.cpp -Lbuild -lgui -lSDL2 -lRVO -lgeometry -o build/blocks

check_rvo: build/check_rvo

build/check_rvo: src/check_rvo.cpp src/RVO.hpp src/RVO.cpp src/geometry.hpp build/libgui.a
	g++ $(FLAGS) src/check_rvo.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -o build/check_rvo

#demo_rvo: build/demo_rvo

#build/demo_rvo: src/demo_rvo.cpp src/simulation.hpp src/simulation.cpp src/RVO.hpp src/RVO.cpp src/simulate_rvo_rds.hpp src/simulate_rvo_rds.cpp src/rds_2.cpp src/rds_2.hpp build/libgui.a build/libgeometry.a build/librds_wrap.a
#	g++ $(FLAGS) src/demo_rvo.cpp src/simulation.cpp src/RVO.cpp src/simulate_rvo_rds.cpp src/rds_2.cpp -Lbuild -lrds_wrap -lgeometry -lgui -lSDL2 -o build/demo_rvo

#demo_simu: build/demo_simu

#build/demo_simu: src/demo_simu.cpp src/simulator.cpp src/simulator.hpp src/orca_style.cpp src/orca_style.hpp build/librds_wrap.a
#	g++ $(FLAGS) src/demo_simu.cpp src/simulator.cpp src/orca_style.cpp -Lbuild -lrds_wrap -lgui -lSDL2 -o build/demo_simu

#demo_rds: build/demo_rds  # target just for convenience

#build/demo_rds: src/demo_rds.cpp build/librds_wrap.a build/libgui.a
#	g++ $(FLAGS) src/demo_rds.cpp -Lbuild -lrds_wrap -lgui -lSDL2 -o build/demo_rds

demo_geo: build/demo_geo # target just for convenience

build/demo_geo: src/demo_geo.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/demo_geo.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/demo_geo

## TESTS (NOT READY TO BUILD)
##
##TEST_LIB = -lgtest -lgtest_main -lpthread
##
##test_geometry: build/test_geometry # target just for convenience
##
##build/test_geometry: test/geometry_test.cpp build/libgeometry.a
##	g++ $(FLAGS) test/geometry_test.cpp -Lbuild -lgeometry $(TEST_LIB) -o build/test_geometry

# LIBRARIES

# rds_wrap: build/librds_wrap.a # target just for convenience

# build/librds_wrap.a: build/rds/.hi src/rds_wrap.cpp src/rds_wrap.hpp build/librds_core.a
# 	g++ $(FLAGS) -c src/rds_wrap.cpp -o build/rds_wrap.o
# 	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
# 	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
# 	ar rvs build/librds_wrap.a build/rds_wrap.o build/rds_core.o build/distance_minimizer.o
# 	cp src/rds_wrap.hpp build/rds
# 	cp src/collision_point.hpp build/rds
# 	cp src/differential_drive_kinematics.hpp build/rds
# 	cp src/rds_core.hpp build/rds
# 	cp src/geometry.hpp build/rds
# 	cp src/capsule.hpp build/rds

rds_core: build/librds_core.a # target just for convenience

build/librds_core.a: build/.hi src/rds_core.cpp src/rds_core.hpp src/differential_drive_kinematics.hpp src/collision_point.hpp build/libgeometry.a
	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds_core.a build/rds_core.o build/distance_minimizer.o

gui: build/libgui.a # target just for convenience

build/libgui.a: build/rds/.hi src/geometry.hpp src/gui.cpp src/gui.hpp src/window.cpp src/window.hpp src/capsule.hpp src/capsule.cpp
	g++ $(FLAGS) -c src/gui.cpp -o build/gui.o 
	g++ $(FLAGS) -c src/window.cpp -o build/window.o
	g++ $(FLAGS) -c src/capsule.cpp -o build/capsule.o
	ar rvs build/libgui.a build/gui.o build/window.o build/capsule.o
	cp src/geometry.hpp build/rds
	cp src/gui.hpp build/rds
	cp src/window.hpp build/rds
	cp src/capsule.hpp build/rds
	
geometry: build/libgeometry.a # target just for convenience

build/libgeometry.a: build/.hi src/geometry.hpp src/distance_minimizer.cpp src/distance_minimizer.hpp src/capsule.hpp src/capsule.cpp
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	g++ $(FLAGS) -c src/capsule.cpp -o build/capsule.o
	ar rvs build/libgeometry.a build/distance_minimizer.o build/capsule.o

# HOUSE KEEPING

clean:
	rm -rf build

build/rds/.hi : build/.hi
	if ! [ -d build/rds ]; then \
		mkdir build/rds; \
	fi
	touch .hi
	mv .hi build/rds/.hi

build/.hi: # targets that depend on this will create the build folder (unless existing) before executing their rule
	if ! [ -d build ]; then \
		mkdir build; \
	fi
	touch .hi
	mv .hi build/.hi