all: geometry gui demo_geo rds_core rds_wrap demo_rds demo_simu demo_rvo# list only what is ready to build
	@echo "\033[92mDone with building RDS.\033[0m"

FLAGS = -std=c++11 -g -Wall

# DEMOS

check_rvo: build/check_rvo

build/check_rvo: src/check_rvo.cpp src/RVO.hpp src/RVO.cpp src/geometry.hpp build/libgui.a
	g++ $(FLAGS) src/check_rvo.cpp src/RVO.cpp -Lbuild -lgui -lSDL2 -o build/check_rvo

demo_rvo: build/demo_rvo

build/demo_rvo: src/demo_rvo.cpp src/simulation.hpp src/simulation.cpp src/RVO.hpp src/RVO.cpp src/simulate_rvo_rds.hpp src/simulate_rvo_rds.cpp build/libgui.a build/libgeometry.a build/librds_wrap.a
	g++ $(FLAGS) src/demo_rvo.cpp src/simulation.cpp src/RVO.cpp src/simulate_rvo_rds.cpp -Lbuild -lrds_wrap -lgeometry -lgui -lSDL2 -o build/demo_rvo

demo_simu: build/demo_simu

build/demo_simu: src/demo_simu.cpp src/simulator.cpp src/simulator.hpp src/orca_style.cpp src/orca_style.hpp build/librds_wrap.a
	g++ $(FLAGS) src/demo_simu.cpp src/simulator.cpp src/orca_style.cpp -Lbuild -lrds_wrap -lgui -lSDL2 -o build/demo_simu

demo_rds: build/demo_rds  # target just for convenience

build/demo_rds: src/demo_rds.cpp build/librds_wrap.a build/libgui.a
	g++ $(FLAGS) src/demo_rds.cpp -Lbuild -lrds_wrap -lgui -lSDL2 -o build/demo_rds

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

rds_wrap: build/librds_wrap.a # target just for convenience

build/librds_wrap.a: build/rds/.hi src/rds_wrap.cpp src/rds_wrap.hpp build/librds_core.a
	g++ $(FLAGS) -c src/rds_wrap.cpp -o build/rds_wrap.o
	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds_wrap.a build/rds_wrap.o build/rds_core.o build/distance_minimizer.o
	cp src/rds_wrap.hpp build/rds
	cp src/collision_point.hpp build/rds
	cp src/differential_drive_kinematics.hpp build/rds
	cp src/rds_core.hpp build/rds
	cp src/geometry.hpp build/rds

rds_core: build/librds_core.a # target just for convenience

build/librds_core.a: build/.hi src/rds_core.cpp src/rds_core.hpp src/differential_drive_kinematics.hpp src/collision_point.hpp build/libgeometry.a
	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds_core.a build/rds_core.o build/distance_minimizer.o

gui: build/libgui.a # target just for convenience

build/libgui.a: build/rds/.hi src/geometry.hpp src/gui.cpp src/gui.hpp src/window.cpp src/window.hpp
	g++ $(FLAGS) -c src/gui.cpp -o build/gui.o 
	g++ $(FLAGS) -c src/window.cpp -o build/window.o
	ar rvs build/libgui.a build/gui.o build/window.o
	cp src/geometry.hpp build/rds
	cp src/gui.hpp build/rds
	cp src/window.hpp build/rds

geometry: build/libgeometry.a # target just for convenience

build/libgeometry.a: build/.hi src/geometry.hpp src/distance_minimizer.cpp src/distance_minimizer.hpp
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/libgeometry.a build/distance_minimizer.o

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