if(NOT TARGET depends::opencv)
  FetchContent_Declare(
    depends-opencv
    URL     https://github.com/opencv/opencv/releases/download/4.0.1/opencv-4.0.1-ios-framework.zip
    URL_MD5 35ebe10de1089f6b1e1cce04d822f740
  )
  FetchContent_GetProperties(depends-opencv)
  if(NOT depends-opencv_POPULATED)
    message(STATUS "Fetching precompiled OpenCV framework")
    FetchContent_Populate(depends-opencv)
    message(STATUS "Fetching precompiled OpenCV framework - done")
    message(STATUS "Configuring OpenCV framework")
    file(COPY ${depends-opencv_SOURCE_DIR}/ DESTINATION ${depends-opencv_BINARY_DIR}/opencv2.framework)
    message(STATUS "Configuring OpenCV framework - done")
  endif()
  find_library(OpenCV_LIBS NAMES opencv2 PATHS ${depends-opencv_BINARY_DIR}/ NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  add_library(depends::opencv INTERFACE IMPORTED GLOBAL)
  target_compile_options(depends::opencv INTERFACE -framework opencv2 -F${depends-opencv_BINARY_DIR} $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-command-line-argument>)
  target_link_libraries(depends::opencv INTERFACE "-framework opencv2")
  target_link_options(depends::opencv INTERFACE "-F${depends-opencv_BINARY_DIR}")
endif()
