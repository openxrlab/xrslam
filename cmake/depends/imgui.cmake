include(FetchContent)

if(NOT TARGET depends::imgui)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::imgui expects options::modern-cpp")
  endif()

  FetchContent_Declare(
    depends-imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.0
  )
  FetchContent_MakeAvailable(depends-imgui)

  add_library(depends_imgui STATIC
    ${depends-imgui_SOURCE_DIR}/imgui.cpp
    ${depends-imgui_SOURCE_DIR}/imgui_draw.cpp
    ${depends-imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${depends-imgui_SOURCE_DIR}/imgui_tables.cpp
    ${depends-imgui_SOURCE_DIR}/imgui_demo.cpp
    ${depends-imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${depends-imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )

  target_include_directories(depends_imgui PUBLIC
    ${depends-imgui_SOURCE_DIR}
  )

  target_link_libraries(depends_imgui PUBLIC options::modern-cpp)

  add_library(depends::imgui ALIAS depends_imgui)

  set(depends-imgui-source-dir ${depends-imgui_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-imgui-source-dir)
endif()