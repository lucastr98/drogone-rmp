cmake_minimum_required(VERSION 2.8.3)
project(drogone_simulation)

find_package(catkin_simple REQUIRED COMPONENTS)

catkin_simple(ALL_DEPS_REQUIRED)

#add executable
cs_add_executable( check_for_catch
  src/check_for_catch.cpp)
cs_add_executable( back_to_position_hold_server
  src/back_to_position_hold_server.cpp)
cs_add_executable( visualization
  src/visualization.cpp)
