cmake_minimum_required(VERSION 2.8.3)
project(glocal_exploration_ros)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/planners/glocal_planner.cpp
        )

###############
# Executables #
###############

cs_add_executable(glocal_planner_node
        src/ros_nodes/glocal_planner_node.cpp
        )
target_link_libraries(glocal_planner_node ${PROJECT_NAME})

