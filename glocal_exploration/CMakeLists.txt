cmake_minimum_required(VERSION 2.8.3)
project(glocal_exploration)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/planning/state_machine.cpp
        src/planning/local_planner/rh_rrt_star.cpp
        )

##########
# Export #
##########

cs_install()
cs_export()