cmake_minimum_required(VERSION 3.10.0)

project(shift)
set(VERSION_MAJOR 1 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message(STATUS "Generating in ${CMAKE_BUILD_TYPE} mode.")

set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0" CACHE STRING "")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${CMAKE_BINARY_DIR})
include_directories(include)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/cmake")

find_package(assimp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(graphviz REQUIRED)
find_package(Qhull REQUIRED)
find_package(cblas)

add_subdirectory(external)
add_subdirectory(assets)
add_subdirectory(src)
