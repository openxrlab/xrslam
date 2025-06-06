if(NOT TARGET options::cpp14)
  add_library(options::cpp14 INTERFACE IMPORTED GLOBAL)
  target_compile_features(options::cpp14 INTERFACE cxx_std_14)
endif()
