cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(xrslam_ros LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    )

include_directories( ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(xrslam_ros
  ${CMAKE_CURRENT_SOURCE_DIR}/src/xrslam_node.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../xrslam-interface/include
)

target_link_libraries(xrslam_ros
    ${catkin_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/libxrslam.so
)
