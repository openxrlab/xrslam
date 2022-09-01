cmake_minimum_required(VERSION 3.14.0 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(lightvis VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

superbuild_depend(glfw)
superbuild_depend(glbinding)
superbuild_depend(nuklear)

add_library(lightvis
  ${CMAKE_CURRENT_SOURCE_DIR}/include/image.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/lightvis.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/shader.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/image.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis_font_roboto.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lightvis_font_roboto.cpp
)

target_include_directories(lightvis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(lightvis
  PUBLIC
    depends::glbinding
    XRPrimer::xrprimer
  PRIVATE
    depends::glfw
    depends::nuklear
)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_definitions(lightvis
    PRIVATE
      _USE_MATH_DEFINES
  )
endif()
