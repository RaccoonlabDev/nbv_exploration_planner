cmake_minimum_required(VERSION 2.8.3)
project(frontier_clustering)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)# -DEIGEN_USE_BLAS)

# Blas library
#find_package( BLAS REQUIRED )

#include_directories(${BLAS_INCLUDE_DIR})


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/frontier.cpp
        src/frontier_clustering.cpp
        )

cs_add_executable(frontierClustering
        src/frontier_clustering_node.cpp
        )
target_link_libraries(frontierClustering ${PROJECT_NAME})# ${BLAS_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
