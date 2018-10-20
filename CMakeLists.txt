cmake_minimum_required(VERSION 3.1.0)

project(shift)
set (shift_VERSION_MAJOR 1)
set (shift_VERSION_MINOR 0)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message(STATUS "Generating in ${CMAKE_BUILD_TYPE} mode.")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${CMAKE_BINARY_DIR})
include_directories(include)

add_subdirectory(assets)
add_subdirectory(src)
