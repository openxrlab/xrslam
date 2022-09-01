cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-pc LANGUAGES CXX)

add_subdirectory(player)

if(NOT XRSLAM_PC_HEADLESS_ONLY)
  add_subdirectory(lightvis)
endif()
