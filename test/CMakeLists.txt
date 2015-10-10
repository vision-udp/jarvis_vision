function(add_unittest TEST_NAME)
  set(TARGET_ID "${TEST_NAME}_test")
  add_jarvis_executable(${TARGET_ID} ${ARGN})
  target_link_libraries(${TARGET_ID} gtest)
  target_disable_global_constructor_warning(${TARGET_ID}) 
  add_test(NAME ${TEST_NAME} COMMAND ${TARGET_ID})
endfunction()

add_subdirectory(core)
