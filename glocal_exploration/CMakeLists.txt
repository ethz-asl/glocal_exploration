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
        src/state/state_machine.cpp
        src/state/communicator.cpp
        src/state/region_of_interest.cpp
        src/planning/local/rh_rrt_star.cpp
        src/planning/local/lidar_model.cpp
        src/planning/global/submap_frontier_evaluator.cpp
        )

##########
# Export #
##########

cs_install()
cs_export()