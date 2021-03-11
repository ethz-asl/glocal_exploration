cmake_minimum_required(VERSION 2.8.3)
project(glocal_exploration)

add_definitions(-std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/state/state_machine.cpp
        src/state/communicator.cpp
        src/state/region_of_interest.cpp
        src/mapping/map_base.cpp
        src/planning/local/rh_rrt_star.cpp
        src/planning/local/lidar_model.cpp
        src/planning/global/submap_frontier_evaluator.cpp
        src/planning/global/skeleton/skeleton_a_star.cpp
)

##########
# Export #
##########

cs_install()
cs_export()