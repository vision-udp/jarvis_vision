cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)
project("Jarvis-Vision" CXX C) # C is needed for FindThreads

# ---------------------------------------
# Initial config
# ---------------------------------------

if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Debug")
  set(CMAKE_BUILD_TYPE "Debug")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(Threads) # Needed for gtest
find_package(Doxygen)
find_package(PCL 1.7.2 REQUIRED COMPONENTS
  common io visualization features filters sample_consensus
  search segmentation)
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---------------------------------------
# Helper functions
# ---------------------------------------

function(target_enable_all_warnings TARGET_ID)
  target_compile_options(${TARGET_ID} PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:Clang>>:
    -Weverything
    -Wno-c++98-compat
    -Wno-c++98-compat-pedantic
    -Wno-padded
  >)
endfunction()

function(target_disable_global_constructor_warning TARGET_ID)
  target_compile_options(${TARGET_ID} PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:Clang>>:
    -Wno-global-constructors
  >)
endfunction()

function(configure_jarvis_target TARGET_ID)
  target_enable_all_warnings(${TARGET_ID})
  set_property(TARGET ${TARGET_ID} PROPERTY CXX_EXTENSIONS OFF)
endfunction()

function(add_jarvis_executable TARGET_ID)
  add_executable(${TARGET_ID} ${ARGN})
  target_link_libraries(${TARGET_ID} PRIVATE jarvis_vision)
  target_include_directories(${TARGET_ID} PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(${TARGET_ID} PRIVATE ${Boost_LIBRARIES})
  configure_jarvis_target(${TARGET_ID})
endfunction()

# ---------------------------------------
# Add utilities
# ---------------------------------------

add_subdirectory("utils")

# ---------------------------------------
# Add the library
# ---------------------------------------

add_subdirectory("src")

# ---------------------------------------
# Add the unit tests
# ---------------------------------------

enable_testing()
add_subdirectory("test")

# ---------------------------------------
# Add documentation
# ---------------------------------------

add_subdirectory("doc")

# ---------------------------------------
# Add tools
# ---------------------------------------

add_subdirectory("tools")

# ---------------------------------------
# Add example
# ---------------------------------------

add_subdirectory("examples")
