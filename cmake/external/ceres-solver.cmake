if(NOT TARGET depends::ceres-solver)
  if(NOT TARGET options::cpp17)
    message(FATAL_ERROR "depends::ceres expects options::cpp17")
  endif()

  FetchContent_Declare(
    depends-ceres-solver
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG        2.0.0
  )
  FetchContent_GetProperties(depends-ceres-solver)
  if(NOT depends-ceres-solver_POPULATED)
    message(STATUS "Fetching ceres-solver sources")
    FetchContent_Populate(depends-ceres-solver)
    message(STATUS "Fetching ceres-solver sources - done")
  endif()

  # Ceres config
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(CERES_BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(CERES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(MINIGLOG OFF CACHE BOOL "" FORCE)
  set(CERES_THREADING_MODEL "CXX_THREADS" CACHE STRING "" FORCE)

  add_subdirectory(${depends-ceres-solver_SOURCE_DIR} ${depends-ceres-solver_BINARY_DIR} EXCLUDE_FROM_ALL)

  add_library(depends::ceres-solver INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::ceres-solver INTERFACE ceres options::cpp17)

  set(depends-ceres-solver-source-dir ${depends-ceres-solver_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-ceres-solver-source-dir)
endif()