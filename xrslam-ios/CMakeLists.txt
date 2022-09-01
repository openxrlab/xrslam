cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-ios)

option(IOS_DEVELOPMENT_TEAM "Team ID for iOS app development" "XXXXXXXXXX")
option(APP_IDENTIFIER_PREFIX "Common prefix for all iOS app identifiers" "xrslam")

if(TARGET xrslam-extra-opencv-image AND TARGET xrslam-extra-yaml-config)
  add_subdirectory(visualizer)
endif()
