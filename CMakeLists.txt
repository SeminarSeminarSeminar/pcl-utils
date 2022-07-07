CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

PROJECT(pcl_project)
MESSAGE(STATUS ${CMAKE_PROJECT_NAME})

# Set flags
SET(COMPILE_FLAGS "-std=c++14")

# Dependent packages
FIND_PACKAGE(PCL 1.8 REQUIRED)

# Header directories(-I)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

# Library directories(-L)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

# Definitions(-D)
ADD_DEFINITIONS(${COMPILE_FLAGS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Binary
ADD_EXECUTABLE(concat src/concat.cpp)
ADD_EXECUTABLE(filter_radius src/filter_radius.cpp)
ADD_EXECUTABLE(filter_stat src/filter_stat.cpp)
ADD_EXECUTABLE(filter_voxel src/filter_voxel.cpp)
ADD_EXECUTABLE(seg_region src/seg_region.cpp)
ADD_EXECUTABLE(gen_graph src/gen_graph.cpp)

# Linker options(-l)
TARGET_LINK_LIBRARIES(concat ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(filter_radius ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(filter_stat ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(filter_voxel ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(seg_region ${PCL_LIBRARIES})
TARGET_LINK_LIBRARIES(gen_graph ${PCL_LIBRARIES})

