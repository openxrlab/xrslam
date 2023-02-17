cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-library VERSION 0.4.0 LANGUAGES CXX)

configure_file(cmake/version.h.in include/xrslam/version.h)

set(XRSLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(XRSLAM_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(XRSLAM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(XRSLAM_BINARY_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
set(XRSLAM_TEST_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

set(PUBLIC_HEADERS
  ${XRSLAM_INCLUDE_DIR}/xrslam/inspection.h
  ${XRSLAM_INCLUDE_DIR}/xrslam/xrslam.h
  ${XRSLAM_BINARY_INCLUDE_DIR}/xrslam/version.h
)

set(PUBLIC_SOURCES
)

set(PRIVATE_HEADERS
  ${XRSLAM_SOURCE_DIR}/xrslam/common.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/detail.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/feature_tracker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/frontend_worker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/initializer.h
  ${XRSLAM_SOURCE_DIR}/xrslam/core/sliding_window_tracker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/cost_function_validator.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/marginalization_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/preintegration_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/quaternion_parameterization.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/reprojection_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/ceres/rotation_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/marginalization_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegration_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegrator.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/reprojection_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/rotation_factor.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/solver.h
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/state.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/essential.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/homography.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/lie_algebra.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/stereo.h
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/wahba.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/frame.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/map.h
  ${XRSLAM_SOURCE_DIR}/xrslam/map/track.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/debug.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/identifiable.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/poisson_disk_filter.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/random.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/ransac.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/tag.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/unique_timer.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/worker.h
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/logger.h
  ${XRSLAM_SOURCE_DIR}/xrslam/ar/virtual_object_manager.h
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/base64.h
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/base64_convert.h
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/localizer.h
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/httplib.h
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/json.h
)

set(PRIVATE_SOURCES
  ${XRSLAM_SOURCE_DIR}/xrslam/config.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/detail.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/feature_tracker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/frontend_worker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/initializer.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/core/sliding_window_tracker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/preintegrator.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/estimation/solver.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/essential.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/homography.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/lie_algebra.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/geometry/stereo.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/inspection.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/frame.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/map.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/map/track.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/xrslam.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/debug.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/utility/worker.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/localizer/localizer.cpp
  ${XRSLAM_SOURCE_DIR}/xrslam/ar/virtual_object_manager.cpp
)

add_compile_options(-fPIC)

add_library(xrslam-core
  ${PUBLIC_HEADERS}
  ${PUBLIC_SOURCES}
  ${PRIVATE_HEADERS}
  ${PRIVATE_SOURCES}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(xrslam-core
    PRIVATE
      -Qunused-arguments
      -mllvm
      -inline-threshold=5000
  )
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_definitions(xrslam-core
    PUBLIC
      _ENABLE_EXTENDED_ALIGNED_STORAGE
    PRIVATE
      _USE_MATH_DEFINES
      _CRT_SECURE_NO_WARNINGS
  )
endif()

target_include_directories(xrslam-core
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/include>
    $<BUILD_INTERFACE:${XRSLAM_INCLUDE_DIR}>
    ${XRSLAM_BINARY_INCLUDE_DIR}
    ${XRSLAM_SOURCE_DIR}
  PRIVATE
    ${XRSLAM_SOURCE_DIR}
)

target_link_libraries(xrslam-core
PUBLIC
  options::modern-cpp
  XRPrimer::xrprimer
  ${OpenCV_LIBS}
)

set(XRSLAM_LIBRARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
