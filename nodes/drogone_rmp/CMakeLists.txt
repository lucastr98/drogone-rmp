cmake_minimum_required(VERSION 2.8.3)
project(drogone_rmp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# add_definitions(-std=c++11)

add_definitions(-std=c++17 -Wall -Wextra)

cs_add_executable(rmp_node
  src/rmp_node.cpp
  src/rmp.cpp
)

cs_add_executable(rmpcpp_example_node
  src/rmpcpp_example_node.cpp
  src/rmpcpp_example.cpp
)

cs_install()
cs_export()
