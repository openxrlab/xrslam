if(NOT TARGET options::cpp11)
  add_library(options::cpp11 INTERFACE IMPORTED GLOBAL)
  target_compile_features(options::cpp11 INTERFACE cxx_std_11)
endif()
