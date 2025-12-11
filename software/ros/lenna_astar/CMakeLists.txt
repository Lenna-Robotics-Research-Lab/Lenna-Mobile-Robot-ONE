cmake_minimum_required(VERSION 2.8.3)
project(lenna_astar)

# Use C++11 explicitly
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs nav_msgs tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astar_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(astar_lib scripts/a_star.cpp)  # Note: change to src/ if file is there
target_link_libraries(astar_lib ${catkin_LIBRARIES})

add_executable(astar_node scripts/astar_node.cpp)
target_link_libraries(astar_node astar_lib ${catkin_LIBRARIES})
