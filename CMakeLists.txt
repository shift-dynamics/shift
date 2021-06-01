cmake_minimum_required(VERSION 3.11)
cmake_policy(SET CMP0048 NEW)
project(shift
    VERSION 2.0.0
    DESCRIPTION "A Dynamics Engine for Automation and Robotics"
    LANGUAGES C CXX
)

option(SHIFT_BUILD_GRAPHICS "Build GUI" OFF)
option(SHIFT_BUILD_TESTS "Build test suite" ON)
option(SHIFT_BUILD_DOC "Build documentation" ON)
option(SHIFT_LOG_LEVEL "Logging level")

message(STATUS
  "${PROJECT_NAME} "
  "v${shift_VERSION_MAJOR}.${shift_VERSION_MINOR}.${shift_VERSION_PATCH}"
)

execute_process(
  COMMAND git rev-parse --abbrev-ref HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE SHIFT_GIT_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE SHIFT_GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

configure_file(
    "${CMAKE_SOURCE_DIR}/src/config.h.in"
    "${CMAKE_BINARY_DIR}/include/shift/config.h"
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    if(NOT SHIFT_LOG_LEVEL)
        set(SHIFT_LOG_LEVEL 4) # DEBUG LOGGING
    endif()
else()
    if(NOT SHIFT_LOG_LEVEL)
        set(SHIFT_LOG_LEVEL 3) # INFO LOGGING
    endif()
endif()

message(STATUS
    "Building ${PROJECT_NAME} in ${CMAKE_BUILD_TYPE} mode "
    "and SHIFT_LOG_LEVEL: ${SHIFT_LOG_LEVEL}")

set(CMAKE_CXX_FLAGS "-DSHIFT_LOG_LEVEL=${SHIFT_LOG_LEVEL}")
set(CMAKE_CXX_FLAGS_DEBUG "-ggdb3 -O0" CACHE STRING "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_SOURCE_DIR}/cmake")
include_directories(${CMAKE_BINARY_DIR}/include)

find_package(assimp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(graphviz REQUIRED)
find_package(Qhull REQUIRED)
find_package(Threads REQUIRED)
find_package(cblas)

include(FetchContent)
FetchContent_Declare(
    ksl
    GIT_REPOSITORY git@github.com:shift-dynamics/ksl.git
    GIT_TAG master
)
FetchContent_MakeAvailable(ksl)

include(ExternalProject)
ExternalProject_Add(
    argagg
    GIT_REPOSITORY https://github.com/vietjtnguyen/argagg.git
    GIT_TAG master
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

if(SHIFT_BUILD_GRAPHICS)
    add_subdirectory(assets)
endif()

add_subdirectory(src)

if(SHIFT_BUILD_DOCS)
    add_subdirectory(doc)
endif()

if(SHIFT_BUILD_TESTS)
    FetchContent_Declare(
      shift-models
      GIT_REPOSITORY git@github.com:shift-dynamics/shift-models.git
      GIT_TAG master
    )
    FetchContent_MakeAvailable(shift-models)
    include(CTest)
    add_subdirectory(test)
endif()
