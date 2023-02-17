cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-interface)

include(SuperBuildDepends)

set(XRSLAM_INTERFACE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INTERFACE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

IF (XRSLAM_PC)
    message(STATUS "xrslam will build as static library")
    add_library(xrslam STATIC
        ${XRSLAM_INTERFACE_INCLUDE_DIR}/XRSLAM.h
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMInternal.cpp
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMManager.cpp)
ELSE()
    message(STATUS "xrslam will build as shared library")
    add_library(xrslam SHARED
        ${XRSLAM_INTERFACE_INCLUDE_DIR}/XRSLAM.h
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMInternal.cpp
        ${XRSLAM_INTERFACE_SOURCE_DIR}/XRSLAMManager.cpp)
ENDIF()

target_link_libraries(xrslam
  PUBLIC
    xrslam-extra-opencv-image
    xrslam-extra-yaml-config
)
target_include_directories(xrslam
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INTERFACE_INCLUDE_DIR}>
    ${XRSLAM_INTERFACE_INCLUDE_DIR}
  PRIVATE
    ${XRSLAM_INTERFACE_SOURCE_DIR}
)

if(XRSLAM_PC)
add_custom_command(TARGET xrslam POST_BUILD
                            COMMAND ar x $<TARGET_FILE:xrslam>
                            COMMAND ar x $<TARGET_FILE:xrslam-extra-opencv-image>
                            COMMAND ar x $<TARGET_FILE:xrslam-extra-yaml-config>
                            COMMAND ar x $<TARGET_FILE:xrslam-core>
                            COMMAND ar x $<TARGET_LINKER_FILE:yaml-cpp>
                            COMMAND ar cr $<TARGET_FILE:xrslam> *.o
                            COMMAND ranlib $<TARGET_FILE:xrslam>
                            COMMAND rm *.o
                            )
endif()

set(XRSLAM_INTERFACE_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
