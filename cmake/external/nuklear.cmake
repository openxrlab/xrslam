if(NOT TARGET depends::nuklear)
  FetchContent_Declare(
    depends-nuklear
    GIT_REPOSITORY https://github.com/vurtun/nuklear.git
    GIT_TAG        509c75b
  )
  FetchContent_GetProperties(depends-nuklear)
  if(NOT depends-nuklear_POPULATED)
    message(STATUS "Fetching nuklear sources")
    FetchContent_Populate(depends-nuklear)
    message(STATUS "Fetching nuklear sources - done")
  endif()
  add_library(depends::nuklear INTERFACE IMPORTED GLOBAL)
  target_compile_definitions(depends::nuklear INTERFACE
    NK_INCLUDE_FIXED_TYPES
    NK_INCLUDE_DEFAULT_ALLOCATOR
    NK_INCLUDE_STANDARD_IO
    NK_INCLUDE_STANDARD_VARARGS
    NK_INCLUDE_VERTEX_BUFFER_OUTPUT
    NK_INCLUDE_FONT_BAKING
    NK_INCLUDE_DEFAULT_FONT
    NK_KEYSTATE_BASED_INPUT
  )
  target_include_directories(depends::nuklear INTERFACE ${depends-nuklear_SOURCE_DIR})
  set(depends-nuklear-source-dir ${depends-nuklear_SOURCE_DIR} CACHE INTERNAL "" FORCE)
  mark_as_advanced(depends-nuklear-source-dir)
endif()
