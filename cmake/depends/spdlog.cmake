if(NOT TARGET depends::spdlog)
  if(NOT TARGET options::modern-cpp)
    message(FATAL_ERROR "depends::spdlog expects options::modern-cpp")
  endif()
  FetchContent_Declare(
    depends-spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.3.1
  )
  FetchContent_GetProperties(depends-spdlog)
  if(NOT depends-spdlog_POPULATED)
    message(STATUS "Fetching spdlog sources")
    FetchContent_Populate(depends-spdlog)
    message(STATUS "Fetching spdlog sources - done")
  endif()
  set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "..." FORCE)
  add_subdirectory(${depends-spdlog_SOURCE_DIR} ${depends-spdlog_BINARY_DIR})
  add_library(depends::spdlog INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::spdlog INTERFACE spdlog::spdlog options::modern-cpp)
endif()
