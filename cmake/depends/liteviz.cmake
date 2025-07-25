if(NOT TARGET depends::liteviz)
  if(NOT TARGET options::cpp17)
    message(FATAL_ERROR "depends::liteviz expects options::cpp17")
  endif()
  FetchContent_Declare(
    depends-liteviz
    GIT_REPOSITORY https://github.com/panxkun/liteviz.git
    GIT_TAG        6aaf8295f09d7c6d6dd5697c5d9b71a4c09ca7e8
  )
  FetchContent_GetProperties(depends-liteviz)
  if(NOT depends-liteviz_POPULATED)
    message(STATUS "Fetching liteviz sources")
    FetchContent_Populate(depends-liteviz)
    message(STATUS "Fetching liteviz sources - done")
  endif()
  set(liteviz_BUILD_TESTS NO CACHE BOOL "" FORCE)
  add_subdirectory(${depends-liteviz_SOURCE_DIR} ${depends-liteviz_BINARY_DIR})
  add_library(depends::liteviz INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::liteviz INTERFACE liteviz-core options::cpp17)
  set(depends-liteviz-source-dir ${depends-liteviz_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-liteviz-source-dir)
endif()
