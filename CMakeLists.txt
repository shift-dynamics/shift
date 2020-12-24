cmake_minimum_required(VERSION 3.11)
project(shift)

option(SHIFT_BUILD_TESTS "Build test suite" ON)
option(SHIFT_BUILD_DOC "Build documentation" ON)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/VersionInfo.cmake)
shift_set_version_info(2 0 0)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message(STATUS "Building ${PROJECT_NAME} in ${CMAKE_BUILD_TYPE} mode.")

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

FetchContent_Declare(
    argagg
    GIT_REPOSITORY https://github.com/vietjtnguyen/argagg.git
    GIT_TAG master
)
FetchContent_MakeAvailable(argagg)

add_subdirectory(assets)
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
