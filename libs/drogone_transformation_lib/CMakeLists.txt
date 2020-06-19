cmake_minimum_required(VERSION 2.8.3)
project(drogone_transformation_lib)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra)

cs_add_library(${PROJECT_NAME}
  src/world_to_camera.cpp
)

cs_install()
cs_export()
