add_library(jarvis_vision 
  "jarvis.cpp"
  "classification.cpp"
  "cloud_io.cpp"
  "cloud_pipeline.cpp"
  "colorize.cpp"
  "model_recognition.cpp"
  "plane_extraction.cpp"
  "simple_visualizer.cpp"
  "steady_timer.cpp"
  )

target_include_directories(jarvis_vision PUBLIC "${CMAKE_SOURCE_DIR}/include")

# The following command lists some features introduced in C++14. 
# Thereby, CMake induces which must compile the library using a C++ standard >= 14
target_compile_features(jarvis_vision PUBLIC
  cxx_generic_lambdas
  cxx_lambda_init_captures
  cxx_return_type_deduction
  )

configure_jarvis_target(jarvis_vision)

target_include_directories(jarvis_vision SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_compile_definitions(jarvis_vision PUBLIC ${PCL_DEFINITIONS})

target_link_libraries(jarvis_vision PRIVATE
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
