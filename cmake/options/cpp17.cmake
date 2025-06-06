if(NOT TARGET options::cpp17)
  add_library(options::cpp17 INTERFACE IMPORTED GLOBAL)
  target_compile_features(options::cpp17 INTERFACE cxx_std_17)
endif()
