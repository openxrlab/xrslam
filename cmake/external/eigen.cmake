if(NOT TARGET depends::eigen)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::eigen expects options::modern-cpp")
  endif()

  include(FetchContent)
  FetchContent_Declare(
    depends-eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
  )
  FetchContent_GetProperties(depends-eigen)
  if(NOT depends-eigen_POPULATED)
    message(STATUS "Fetching eigen sources")
    FetchContent_Populate(depends-eigen)
    message(STATUS "Fetching eigen sources - done")
  endif()

  add_library(depends::eigen INTERFACE IMPORTED GLOBAL)

  target_include_directories(depends::eigen INTERFACE
    ${depends-eigen_SOURCE_DIR})

  target_link_libraries(depends::eigen INTERFACE options::modern-cpp)

  set(depends-eigen-source-dir ${depends-eigen_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-eigen-source-dir)
endif()
