cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-interface)

include(SuperBuildDepends)

set(XRSLAM_INTERFACE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INTERFACE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(xrslam-interface
  ${XRSLAM_INTERFACE_INCLUDE_DIR}/XRSLAM.h
  ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMInternal.cpp
  ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMManager.cpp
)
target_link_libraries(xrslam-interface
  PUBLIC
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
)
target_include_directories(xrslam-interface
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INTERFACE_INCLUDE_DIR}>
    ${XRSLAM_INTERFACE_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_INTERFACE_SOURCE_DIR}
)

set(XRSLAM_INTERFACE_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
