cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nasa_msgs nav_msgs actionlib tf move_base_msgs visualization_msgs
	geometry_msgs tf cv_bridge)



find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
#find_library(CGAL_LIBRARY CGAL /usr/local)
#find_package(CGAL REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIRS})

#include_directories(/home/mowgli/Downloads/boost_1_53_0)
#include_directories(${CGAL_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build library files
add_library(local_planner_com_lib  src/lib/map_tools.cpp
                                            src/lib/general.cpp)

target_link_libraries(local_planner_com_lib ${catkin_LIBRARIES})
## test
include(LocalPlanTests.cmake OPTIONAL)

## Test Node
add_executable(local_path_test src/nodes/local_path_test.cpp)
target_link_libraries(local_path_test local_planner_com_lib ${catkin_LIBRARIES})

## main local_planner node
add_library(local_path_node_lib src/lib/localplan.cpp src/lib/localctrlr.cpp)
target_link_libraries(local_path_node_lib local_planner_com_lib  ${catkin_LIBRARIES})

add_executable(local_path_node src/nodes/local_path_node.cpp)
target_link_libraries(local_path_node local_path_node_lib local_planner_com_lib ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
