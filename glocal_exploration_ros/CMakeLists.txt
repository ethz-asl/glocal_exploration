cmake_minimum_required(VERSION 2.8.3)
project(glocal_exploration_ros)

add_definitions(-std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/glocal_system.cpp
        src/mapping/voxblox_map.cpp
        src/mapping/voxgraph_map.cpp
        src/mapping/voxgraph_local_area.cpp
        src/mapping/voxgraph_spatial_hash.cpp
        src/planning/global/skeleton_planner.cpp
        src/conversions/ros_component_factory.cpp
        src/visualization/rh_rrt_star_visualizer.cpp
        src/visualization/skeleton_visualizer.cpp
)

###############
# Executables #
###############

cs_add_executable(glocal_system_node
    app/glocal_system_node.cpp)
target_link_libraries(glocal_system_node ${PROJECT_NAME})

cs_add_executable(voxblox_evaluation_node
        app/experiments/voxblox_evaluation_node.cpp)
target_link_libraries(voxblox_evaluation_node ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()

