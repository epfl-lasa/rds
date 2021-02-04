cmake_minimum_required(VERSION 3.0.2)
project(user_pkg_example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rds_ros
  rds_network_ros
  roscpp
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES user_pkg_example
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(holonomic_nominal_command_node src/holonomic_nominal_command_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(holonomic_nominal_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(holonomic_nominal_command_node
  ${catkin_LIBRARIES}
)