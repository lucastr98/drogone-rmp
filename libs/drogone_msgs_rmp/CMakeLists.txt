cmake_minimum_required(VERSION 2.8.3)
project(drogone_msgs_rmp)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########
cs_install()
cs_export()
