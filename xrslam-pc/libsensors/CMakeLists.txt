cmake_minimum_required(VERSION 3.14.0 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(libsensors LANGUAGES CXX)

include(CMakeDependentOption)
include(SuperBuildDepends)


if(NOT IOS AND NOT ANDROID)
  superbuild_extern(ffmpeg)
endif()

add_library(sensors
  ${CMAKE_CURRENT_SOURCE_DIR}/include/libsensors.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/libsensors.cpp
)

target_include_directories(sensors
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(sensors
  PRIVATE
    options::modern-cpp
)

if(TARGET depends::ffmpeg)
  target_link_libraries(sensors
    PUBLIC
      depends::ffmpeg
  )
endif()
