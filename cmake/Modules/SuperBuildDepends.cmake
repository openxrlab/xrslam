include(CMakeParseArguments)
include(FetchContent)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

if(NOT COMMAND superbuild_option)
  function(superbuild_option option_name)
    include("${CMAKE_SOURCE_DIR}/cmake/options/${option_name}.cmake")
  endfunction()
endif()

if(NOT COMMAND superbuild_depend)
  function(superbuild_depend depend_name)
    include("${CMAKE_SOURCE_DIR}/cmake/depends/${depend_name}.cmake")
  endfunction()
endif()

if(NOT COMMAND superbuild_extern)
  function(superbuild_extern extern_name)
    set(platform PLATFORM)
    cmake_parse_arguments(SUPERBUILD_EXTERN "" "${platform}" "" ${ARGN})
    if(NOT DEFINED SUPERBUILD_EXTERN_PLATFORM OR SUPERBUILD_EXTERN_PLATFORM STREQUAL AUTO)
      if(IOS)
        set(SUPERBUILD_EXTERN_PLATFORM IOS)
      elseif(ANDROID)
        set(SUPERBUILD_EXTERN_PLATFORM ANDROID)
      else()
        set(SUPERBUILD_EXTERN_PLATFORM SYSTEM)
      endif()
    endif()
    set(SUPERBUILD_EXTERN_SYSTEM_MODULE "${CMAKE_SOURCE_DIR}/cmake/external/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_IOS_MODULE "${CMAKE_SOURCE_DIR}/cmake/external/ios/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_ANDROID_MODULE "${CMAKE_SOURCE_DIR}/cmake/external/android/${extern_name}.cmake")
    if(EXISTS "${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
      include("${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
    else()
      include("${SUPERBUILD_EXTERN_SYSTEM_MODULE}")
    endif()
  endfunction()
endif()