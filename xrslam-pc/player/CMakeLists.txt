cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-pc-player LANGUAGES CXX)

include(SuperBuildDepends)
superbuild_depend(argparse)

add_executable(xrslam-pc-player
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/opencv_painter.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/euroc_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/euroc_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/async_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/async_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/tum_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/tum_dataset_reader.cpp
)

if(XRSLAM_PC_HEADLESS_ONLY)
target_compile_definitions(xrslam-pc-player
  PRIVATE
    XRSLAM_PC_HEADLESS_ONLY
)
endif()

target_include_directories(xrslam-pc-player
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/
)

target_link_libraries(xrslam-pc-player
  PRIVATE
    xrslam
    depends::argparse
)

if(NOT XRSLAM_PC_HEADLESS_ONLY)
target_link_libraries(xrslam-pc-player
  PRIVATE
    depends::glbinding
    depends::nuklear
    lightvis
)
endif()
