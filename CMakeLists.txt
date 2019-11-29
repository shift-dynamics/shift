cmake_minimum_required(VERSION 3.10.0)

project(shift)
set(VERSION_MAJOR 2 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

# Get the current working branch
execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Get the latest abbreviated commit hash of the working branch
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
  ${CMAKE_SOURCE_DIR}/src/version.h.in
  ${CMAKE_BINARY_DIR}/version.h
)

include(CTest)

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
find_package(Threads REQUIRED)
find_package(cblas)

add_subdirectory(external)
add_subdirectory(assets)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(doc)
