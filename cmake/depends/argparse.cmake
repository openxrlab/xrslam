if(NOT TARGET depends::argparse)
  if(NOT TARGET options::cpp17)
    message(FATAL_ERROR "depends::argparse expects options::cpp17")
  endif()
  FetchContent_Declare(
    depends-argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG        v1.3
  )
  FetchContent_GetProperties(depends-argparse)
  if(NOT depends-argparse_POPULATED)
    message(STATUS "Fetching argparse sources")
    FetchContent_Populate(depends-argparse)
    message(STATUS "Fetching argparse sources - done")
  endif()
  set(ARGPARSE_BUILD_TESTS NO CACHE BOOL "" FORCE)
  add_subdirectory(${depends-argparse_SOURCE_DIR} ${depends-argparse_BINARY_DIR})
  add_library(depends::argparse INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::argparse INTERFACE argparse::argparse options::cpp17)
  set(depends-argparse-source-dir ${depends-argparse_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-argparse-source-dir)
endif()
