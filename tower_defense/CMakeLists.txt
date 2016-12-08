PROJECT(tower_defense)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(tower_defense
    src/tower_defense_main.cpp)
TARGET_LINK_LIBRARIES(tower_defense ${libs} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
