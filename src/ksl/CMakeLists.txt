cmake_minimum_required(VERSION 2.8.11)
project(ksl)

find_package(cblas REQUIRED)
find_library(M_LIB m REQUIRED)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

add_definitions(-ggdb)
add_definitions(-std=c11)
add_definitions(-O3)
add_definitions(-Wall)
add_definitions(-D_GNU_SOURCE=1)
add_definitions(-pipe)
add_definitions(-fPIC)
add_definitions(-msse)
add_definitions(-msse2)
add_definitions(-msse3)
add_definitions(-ffast-math)
add_definitions(-fassociative-math)
add_definitions(-ftree-vectorizer-verbose=7)
add_definitions(-fopt-info)

if(APPLE)
    add_definitions(-framework accelerate)
endif(APPLE)

include_directories(
  ${CBLAS_INCLUDE_DIRS}
)

add_library(ksl STATIC ksl.c)

add_library(bksl STATIC bksl.c)

add_executable(ksl_test ksl_test.c)
target_link_libraries(ksl_test ksl bksl ${M_LIB} ${CBLAS_LIBRARIES})
