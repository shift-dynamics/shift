cmake_minimum_required(VERSION 2.8.11)
project(gjk_test)


find_package(cblas REQUIRED)
add_executable(gjk main.c)
target_link_libraries(
  gjk
  ${CBLAS_LIBRARIES})
include_directories(
  ${CBLAS_INCLUDE_DIRS})
add_definitions(-ggdb)
find_library( M_LIB m REQUIRED )
include_directories(../../ksl)
include_directories(../../core)
add_library(kinematics STATIC ../../core/kinematics.c)
add_definitions(-ggdb)
target_link_libraries(gjk ${M_LIB} kinematics)
