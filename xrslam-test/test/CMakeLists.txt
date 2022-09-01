cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-test-test LANGUAGES CXX)

include_directories(test_feature_track PRIVATE ${CMAKE_SOURCE_DIR}/xrslam/src)
include(SuperBuildDepends)
superbuild_depend(googletest)

function(xr_add_test target)
    set(options)
    set(multiValueArgs LINKS SRCS)
    cmake_parse_arguments(_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${target} ${_TEST_SRCS})
    target_link_libraries(${target} ${_TEST_LINKS} depends::googletest)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endfunction()

xr_add_test(test_version SRCS src/test_main.cpp src/test_version.cpp)
target_link_libraries(test_version xrslam)

xr_add_test(test_se3_cost_function SRCS src/test_main.cpp src/test_se3_cost_function.cpp)
target_link_libraries(test_se3_cost_function xrslam)

xr_add_test(test_feature_track SRCS src/test_main.cpp src/test_feature_track.cpp)
target_link_libraries(test_feature_track xrslam-extra-opencv-image xrslam-extra-yaml-config)
