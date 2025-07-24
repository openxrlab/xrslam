if(NOT TARGET depends::glad)
  include(FetchContent)

  FetchContent_Declare(
    depends-glad
    GIT_REPOSITORY https://github.com/panxkun/glad-opengl-core43.git
    GIT_TAG        master
  )
  FetchContent_GetProperties(depends-glad)
  if(NOT depends-glad_POPULATED)
    message(STATUS "Fetching GLAD sources")
    FetchContent_Populate(depends-glad)
    message(STATUS "Fetching GLAD sources - done")
  endif()

  add_library(depends_glad STATIC
    ${depends-glad_SOURCE_DIR}/src/glad.c
  )

  target_include_directories(depends_glad PUBLIC
    ${depends-glad_SOURCE_DIR}/include
  )

  add_library(depends::glad ALIAS depends_glad)

  set(depends-glad-source-dir ${depends-glad_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-glad-source-dir)
endif()
