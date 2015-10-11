cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)
project("Jarvis-Vision" CXX C) # C is needed for FindThreads

# ---------------------------------------
# Initial config                        
# --------------------------------------- 

if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Debug")
  set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(Threads) # Needed for gtest
find_package(Doxygen)

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
  target_link_libraries(${TARGET_ID} jarvis_vision)
  configure_jarvis_target(${TARGET_ID})
endfunction()

# ---------------------------------------
# Add utilities
# ---------------------------------------

add_subdirectory("utils")

# ---------------------------------------
# Add the Quotient-Filter library  
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