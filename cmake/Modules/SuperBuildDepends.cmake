# MIT License

# Copyright (c) 2019 jinyu li

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

include(CMakeParseArguments)
include(FetchContent)

FetchContent_Declare(
  super-build-depends
  GIT_REPOSITORY https://github.com/itsuhane/cmake-superbuild-depends.git
  GIT_TAG        master
)
FetchContent_GetProperties(super-build-depends)
if(NOT super-build-depends_POPULATED)
  message(STATUS "Fetching SuperBuildDepends scripts")
  FetchContent_Populate(super-build-depends)
  message(STATUS "Fetching SuperBuildDepends scripts - done")
endif()

list(APPEND CMAKE_MODULE_PATH "${super-build-depends_SOURCE_DIR}/cmake/Modules")

if(NOT COMMAND superbuild_option)
  function(superbuild_option option_name)
    include("${super-build-depends_SOURCE_DIR}/options/${option_name}.cmake")
  endfunction()
endif()

if(NOT COMMAND superbuild_depend)
  function(superbuild_depend depend_name)
    include("${super-build-depends_SOURCE_DIR}/depends/${depend_name}.cmake")
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
    set(SUPERBUILD_EXTERN_SYSTEM_MODULE "${super-build-depends_SOURCE_DIR}/externs/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_IOS_MODULE "${super-build-depends_SOURCE_DIR}/externs/ios/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_ANDROID_MODULE "${super-build-depends_SOURCE_DIR}/externs/android/${extern_name}.cmake")
    if(EXISTS "${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
      include("${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
    else()
      include("${SUPERBUILD_EXTERN_SYSTEM_MODULE}")
    endif()
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
    set(SUPERBUILD_EXTERN_SYSTEM_MODULE "${super-build-depends_SOURCE_DIR}/externs/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_IOS_MODULE "${super-build-depends_SOURCE_DIR}/externs/ios/${extern_name}.cmake")
    set(SUPERBUILD_EXTERN_ANDROID_MODULE "${super-build-depends_SOURCE_DIR}/externs/android/${extern_name}.cmake")
    if(EXISTS "${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
      include("${SUPERBUILD_EXTERN_${SUPERBUILD_EXTERN_PLATFORM}_MODULE}")
    else()
      include("${SUPERBUILD_EXTERN_SYSTEM_MODULE}")
    endif()
  endfunction()
endif()
