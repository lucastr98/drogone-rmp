cmake_minimum_required(VERSION 2.8.3)
project(drogone_dummy_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

cs_add_executable(dummy_detector_node
  src/dummy_detector.cpp
  src/dummy_detector_node.cpp
)

cs_install()
cs_export()
