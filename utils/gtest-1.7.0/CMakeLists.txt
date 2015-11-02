add_library(gtest "src/gtest-all.cc" "src/gtest_main.cc")
target_link_libraries(gtest PRIVATE ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(gtest PRIVATE "include" ".")
set_target_properties(gtest PROPERTIES CXX_STANDARD 14)

target_include_directories(gtest SYSTEM INTERFACE
	"${CMAKE_CURRENT_SOURCE_DIR}/include")
