cmake_minimum_required(VERSION 2.8.3)
project(nbveplanner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

cs_add_library(${PROJECT_NAME}
        src/nbvep.cpp
        src/tree.cpp
        src/rrt.cpp
        src/history.cpp
        src/voxblox_manager.cpp)


cs_add_executable(nbvePlanner src/nbv_exp_planner_node.cpp)
target_link_libraries(nbvePlanner ${PROJECT_NAME})

cs_install()
cs_export()
