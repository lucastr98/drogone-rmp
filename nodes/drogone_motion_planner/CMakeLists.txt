cmake_minimum_required(VERSION 2.8.3)
project(drogone_motion_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# add_definitions(-std=c++11)
add_definitions(-std=c++17 -Wall -Wextra)

cs_add_executable(rmp_planner_node
  src/rmp_planner.cpp
  src/rmp_planner_node.cpp
)

cs_install()
cs_export()
