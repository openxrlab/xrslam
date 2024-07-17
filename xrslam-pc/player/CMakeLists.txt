cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(xrslam-pc-player LANGUAGES CXX)

include(SuperBuildDepends)
superbuild_depend(argparse)

add_executable(xrslam-pc-player
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/euroc_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/euroc_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/async_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/async_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/tum_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/tum_dataset_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/sensors_dataset_reader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/sensors_dataset_reader.cpp
)


if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
    find_package(OpenGL REQUIRED)
    add_definitions(${OPENGL_DEFINITIONS})
    find_package(X11 REQUIRED)
    set(LIBS ${GLFW3_LIBRARY} X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL dl pthread freetype ${ASSIMP_LIBRARY})
    set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")
elseif(APPLE)
    include_directories(/System/Library/Frameworks)
    find_library(COCOA_LIBRARY Cocoa)
    find_library(OpenGL_LIBRARY OpenGL)
    find_library(IOKit_LIBRARY IOKit)
    find_library(CoreVideo_LIBRARY CoreVideo)
    mark_as_advanced(COCOA_LIBRARY OpenGL_LIBRARY)
    set(APPLE_LIBS ${COCOA_LIBRARY} ${IOKit_LIBRARY} ${OpenGL_LIBRARY} ${CoreVideo_LIBRARY})
    set(LIBS ${APPLE_LIBS})
endif()

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
    sensors
    depends::argparse
    xrslam-liteviz
    ${LIBS}
)
