if(NOT TARGET depends::googletest)
  FetchContent_Declare(
      depends-googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        703bd9caab50b139428cea1aaff9974ebee5742e # release-1.10.0
  )
  FetchContent_GetProperties(depends-googletest)
  if(NOT depends-googletest_POPULATED)
      message(STATUS "Fetching googletest sources")
      FetchContent_Populate(depends-googletest)
      message(STATUS "Fetching googletest sources - done")
  endif()
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  set(INSTALL_GTEST     OFF)

  # set(LIBRARY_OUTPUT_PATH  ${depends-googletest_BINARY_DIR})

  add_subdirectory(${depends-googletest_SOURCE_DIR} ${depends-googletest_BINARY_DIR})
  add_library(depends::googletest INTERFACE IMPORTED GLOBAL)
  target_link_libraries(depends::googletest INTERFACE gtest)
  set(depends-googletest-source-dir ${depends-googletest_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  set(depends-googletest-binary-dir ${depends-googletest_BINARY_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-googletest-source-dir)
  mark_as_advanced(depends-googletest-binary-dir)
endif()
