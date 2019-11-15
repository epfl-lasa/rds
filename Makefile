all: geometry gui demo_geo # list only what is ready to build

FLAGS = -std=c++11 -g -Wall


# DEMOS

demo_rds: build/demo_rds  # target just for convenience

build/demo_rds: src/demo_rds.cpp build/librds_wrap.a build/libgui.a
	g++ $(FLAGS) src/demo_rds.cpp -Lbuild -lrds_wrap -lgui -lSDL2 -o build/demo_rds

demo_geo: build/demo_geo # target just for convenience

build/demo_geo: src/demo_geo.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/demo_geo.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/demo_geo

# TESTS

TEST_LIB = -lgtest -lgtest_main -lpthread

test_geometry: build/test_geometry # target just for convenience

build/test_geometry: test/geometry_test.cpp build/libgeometry.a
	g++ $(FLAGS) test/geometry_test.cpp -Lbuild -lgeometry $(TEST_LIB) -o build/test_geometry


# ROS NODES

ROS_LIB = -Wl,-rpath,/opt/ros/kinetic/lib -L/opt/ros/kinetic/lib \
-lroscpp -lrosconsole -lroscpp_serialization -lrostime \
-I/opt/ros/kinetic/include

ROS_LIB_2 = -Wl,-rpath,/opt/ros/kinetic/lib -L/opt/ros/kinetic/lib \
-lroscpp -lrosconsole -lroscpp_serialization -lrostime -lvelodyne_rawdata -lroslib \
-I/opt/ros/kinetic/include -I/usr/include/pcl-1.7 -I/usr/include/eigen3/

lidar_to_lrf_node: build/lidar_to_lrf_node

build/lidar_to_lrf_node: src/lidar_to_lrf_node.cpp
	g++ $(FLAGS) src/lidar_to_lrf_node.cpp $(ROS_LIB_2) -o build/lidar_to_lrf_node

gui_node: build/gui_node # target just for convenience

build/gui_node: src/gui_node.cpp build/libgui.a
	g++ $(FLAGS) src/gui_node.cpp -Lbuild -lgui -lSDL2 $(ROS_LIB) -o build/gui_node

rds_node: build/rds_node # target just for convenience

build/rds_node: src/rds_node.cpp build/librds.a
	g++ $(FLAGS) src/rds_node.cpp -Lbuild -lrds $(ROS_LIB) -o build/rds_node

nominal_command_node: build/nominal_command_node # target just for convenience

build/nominal_command_node: src/nominal_command_node.cpp
	g++ $(FLAGS) src/nominal_command_node.cpp $(ROS_LIB) -o build/nominal_command_node

# not ready:

gui_client_node: build/gui_client_node # target just for convenience

build/gui_client_node: src/gui_client.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/gui_client.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/gui_client_node

gui_server_node: build/gui_server_node # target just for convenience

build/gui_server_node: build/.hi src/gui_server.cpp src/geometry.hpp
	g++ $(FLAGS) src/gui_server.cpp -o build/gui_server_node


# LIBRARIES

rds_wrap: build/librds_wrap.a # target just for convenience

build/librds_wrap.a: src/rds_wrap.cpp src/rds_wrap.hpp build/librds_core.a
	g++ $(FLAGS) -c src/rds_wrap.cpp -o build/rds_wrap.o
	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds_wrap.a build/rds_wrap.o build/rds_core.o build/distance_minimizer.o

rds_core: build/librds_core.a # target just for convenience

build/librds_core.a: src/rds_core.cpp src/rds_core.hpp src/differential_drive_kinematics.hpp src/collision_point.hpp build/libgeometry.a
	g++ $(FLAGS) -c src/rds_core.cpp -o build/rds_core.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds_core.a build/rds_core.o build/distance_minimizer.o

rds: build/librds.a # target just for convenience

build/librds.a: src/rds.cpp src/rds.hpp build/libgeometry.a
	g++ $(FLAGS) -c src/rds.cpp -o build/rds.o
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/librds.a build/rds.o build/distance_minimizer.o

gui: build/libgui.a # target just for convenience

build/libgui.a: build/.hi src/geometry.hpp src/gui.cpp src/gui.hpp src/window.cpp src/window.hpp
	g++ $(FLAGS) -c src/gui.cpp -o build/gui.o 
	g++ $(FLAGS) -c src/window.cpp -o build/window.o
	ar rvs build/libgui.a build/gui.o build/window.o

geometry: build/libgeometry.a # target just for convenience

build/libgeometry.a: build/.hi src/geometry.hpp src/distance_minimizer.cpp src/distance_minimizer.hpp
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/libgeometry.a build/distance_minimizer.o

# HOUSE KEEPING

clean:
	rm -rf build

build/.hi: # targets that depend on this will create the build folder (unless existing) before executing their rule
	if ! [ -d build ]; then \
		mkdir build; \
	fi
	touch .hi
	mv .hi build/.hi