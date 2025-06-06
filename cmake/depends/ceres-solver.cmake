if(NOT TARGET depends::ceres-solver)
  if(NOT TARGET depends::eigen)
    message(FATAL_ERROR "depends::ceres-solver expects depends::eigen")
  endif()
  FetchContent_Declare(
    depends-ceres-solver
    GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
    GIT_TAG        e809cf0c2879f521078b4c9e6329390b42ecf722 # 1.14.0 with std::function fix.
  )
  FetchContent_GetProperties(depends-ceres-solver)
  if(NOT depends-ceres-solver_POPULATED)
    message(STATUS "Fetching ceres-solver sources")
    FetchContent_Populate(depends-ceres-solver)
    message(STATUS "Fetching ceres-solver sources - done")
  endif()
  # TODO: clean up parameters and avoid pollution.
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(MINIGLOG ON CACHE BOOL "" FORCE)
  set(GFLAGS OFF CACHE BOOL "" FORCE)
  set(CXX11 ON CACHE BOOL "" FORCE)
  set(CXX11_THREADS ON CACHE BOOL "" FORCE)
  set(TBB OFF CACHE BOOL "" FORCE)
  set(OPENMP OFF CACHE BOOL "" FORCE)
  set(LAPACK ON CACHE BOOL "" FORCE)
  set(MINIGLOG_MAX_LOG_LEVEL 0 CACHE STRING "" FORCE)
  set(EIGEN_PREFER_EXPORTED_EIGEN_CMAKE_CONFIGURATION OFF CACHE BOOL "" FORCE)
  set(EIGEN_INCLUDE_DIR ${depends-eigen-source-dir} CACHE PATH "" FORCE)
  add_subdirectory(${depends-ceres-solver_SOURCE_DIR} ${depends-ceres-solver_BINARY_DIR})
  add_library(depends::ceres-solver INTERFACE IMPORTED GLOBAL)
  target_include_directories(depends::ceres-solver
    INTERFACE
      ${depends-ceres-solver_BINARY_DIR}/config
      ${depends-ceres-solver_SOURCE_DIR}/internal/ceres/miniglog
      ${depends-ceres-solver_SOURCE_DIR}/include
  )
  target_link_libraries(depends::ceres-solver INTERFACE ceres depends::eigen)
  set(depends-ceres-solver-source-dir ${depends-ceres-solver_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-ceres-solver-binary-dir ${depends-ceres-solver_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-ceres-solver-source-dir)
  mark_as_advanced(depends-ceres-solver-binary-dir)
endif()
