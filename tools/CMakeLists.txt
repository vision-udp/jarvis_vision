# ======================
# cv_openni
# ======================

add_jarvis_executable(cv_openni
  "cv_openni.cpp"
  )
target_include_directories(cv_openni SYSTEM PRIVATE 
  ${OpenCV_INCLUDE_DIRS}
  )
target_link_libraries(cv_openni PRIVATE
  ${OpenCV_LIBS}
  )

# ======================
# pcl_openni
# ======================

add_jarvis_executable(pcl_openni
  "pcl_openni.cpp"
  )
target_include_directories(pcl_openni SYSTEM PRIVATE 
  ${PCL_INCLUDE_DIRS}
  )
target_link_libraries(pcl_openni PRIVATE
  ${PCL_LIBRARIES}
  )
target_compile_definitions(pcl_openni PRIVATE
  ${PCL_DEFINITIONS})
