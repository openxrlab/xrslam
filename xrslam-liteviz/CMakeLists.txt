cmake_minimum_required(VERSION 3.11)
project(xrslam-liteviz LANGUAGES CXX)

add_compile_options(-fPIC)

include(SuperBuildDepends)
superbuild_depend(argparse)
superbuild_depend(glfw)

find_package(GLEW REQUIRED)
message(STATUS "Found GLEW in ${GLEW_INCLUDE_DIRS}")

# find_package(ASSIMP REQUIRED)  
# include_directories(${ASSIMP_INCLUDE_DIRS})
# message(STATUS "Found ASSIMP included at ${ASSIMP_INCLUDE_DIRS}")  

# 包含必要的目录
include_directories(
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
)

# 查找源文件
file(GLOB IMGUI_SOURCE
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui/*.h
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui/*.cpp
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.*
  ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3.*
)

file(GLOB VIEWER_SOURCE
  ${CMAKE_CURRENT_SOURCE_DIR}/viewer.cpp
)

# 添加库
add_library(xrslam-liteviz STATIC
  ${IMGUI_SOURCE}
  ${VIEWER_SOURCE}
  ${OPENGL_LIBRARIES}
)

# 链接依赖库
target_link_libraries(xrslam-liteviz
  PRIVATE
    options::modern-cpp
    XRPrimer::xrprimer
    glfw
    GLEW::GLEW
    assimp
)

# 设置包含目录
target_include_directories(xrslam-liteviz
INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
)