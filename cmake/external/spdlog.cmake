if(NOT TARGET depends::spdlog)
  if(NOT TARGET options::cpp17)
    message(FATAL_ERROR "depends::spdlog expects options::cpp17")
  endif()
  include(FetchContent)
  FetchContent_Declare(
    depends-spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.11.0
  )
  FetchContent_GetProperties(depends-spdlog)
  if(NOT depends-spdlog_POPULATED)
    message(STATUS "Fetching spdlog sources")
    FetchContent_Populate(depends-spdlog)
    message(STATUS "Fetching spdlog sources - done")
  endif()

  set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
  set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)

  add_subdirectory(${depends-spdlog_SOURCE_DIR} ${depends-spdlog_BINARY_DIR})

  set_target_properties(spdlog PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(depends::spdlog INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::spdlog INTERFACE spdlog::spdlog options::cpp17)
  set(depends-spdlog-source-dir ${depends-spdlog_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-spdlog-source-dir)
endif()
