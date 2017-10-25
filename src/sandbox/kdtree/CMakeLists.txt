cmake_minimum_required(VERSION 2.8.11)
project(elevationmap_test)


find_package(cblas REQUIRED)
add_executable(kdtree_test kdtree_test.c)
target_link_libraries(
  kdtree_test
  ${CBLAS_LIBRARIES})
include_directories(
  ${CBLAS_INCLUDE_DIRS})
add_definitions(-ggdb)
find_library( M_LIB m REQUIRED )
include_directories(../../ksl)
include_directories(../../core)
add_library(factor STATIC ../../core/factor.c)
add_definitions(-ggdb)
add_library(kdtree STATIC ../../core/kdtree.c)
add_definitions(-ggdb)
target_link_libraries(kdtree_test ${M_LIB} kdtree factor)
