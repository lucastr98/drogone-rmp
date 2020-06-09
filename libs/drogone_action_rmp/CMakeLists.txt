cmake_minimum_required(VERSION 2.8.3)
project(drogone_action_rmp)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

############
# BINARIES #
############

#add executable

cs_install()
cs_export()
