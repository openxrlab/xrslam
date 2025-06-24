if(NOT TARGET depends::glfw)
  FetchContent_Declare(
    depends-glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3
  )
  FetchContent_GetProperties(depends-glfw)
  if(NOT depends-glfw_POPULATED)
    message(STATUS "Fetching GLFW sources")
    FetchContent_Populate(depends-glfw)
    message(STATUS "Fetching GLFW sources - done")
  endif()
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory(${depends-glfw_SOURCE_DIR} ${depends-glfw_BINARY_DIR})
  add_library(depends::glfw INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::glfw INTERFACE glfw)
  set(depends-glfw-source-dir ${depends-glfw_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-glfw-binary-dir ${depends-glfw_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-glfw-source-dir)
  mark_as_advanced(depends-glfw-binary-dir)
endif()