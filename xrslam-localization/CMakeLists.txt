cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-localization)

include(SuperBuildDepends)
include_directories(${CMAKE_SOURCE_DIR}/xrslam-interface/include)

set(XRSLAM_LOCALIZATION_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_LOCALIZATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(xrslam-localization
  ${XRSLAM_LOCALIZATION_INCLUDE_DIR}/XRGlobalLocalizer.h
  ${XRSLAM_LOCALIZATION_SOURCE_DIR}/XRGlobalLocalizerInternal.cpp
  ${XRSLAM_LOCALIZATION_SOURCE_DIR}/XRGlobalLocalizerManager.cpp
)
target_link_libraries(xrslam-localization
  PUBLIC
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
)
target_include_directories(xrslam-localization
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_LOCALIZATION_INCLUDE_DIR}>
    ${XRSLAM_LOCALIZATION_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_LOCALIZATION_SOURCE_DIR}
)

set(XRSLAM_LOCALIZATION_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
