cmake_minimum_required(VERSION 2.8.11)
project(pca_test)
message( ${CMAKE_C_COMPILER} )
message( ${CMAKE_CXX_COMPILER} )

set(VERSION_MAJOR 1 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

list(INSERT CMAKE_MODULE_PATH 0 "${pca_test_SOURCE_DIR}/../../../cmake")

find_package(cblas REQUIRED)
find_library(M_LIB m REQUIRED)
find_package(Qhull REQUIRED)
message( "using libraries: " ${LIBS} )

## set output directory ##
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/ )

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
  ${QHULL_INCLUDE_DIRS}
)

### build static library ###
add_executable(pca_test
  ${pca_test_SOURCE_DIR}/main.c
  ${pca_test_SOURCE_DIR}/../../core/pca.c
  ${pca_test_SOURCE_DIR}/../../core/kinematics.c
  ${pca_test_SOURCE_DIR}/../../core/ellipsoid.c
  ${pca_test_SOURCE_DIR}/../../core/convexhull.c
  ${pca_test_SOURCE_DIR}/../../core/factor.c
)

target_link_libraries(pca_test
  ${LIBS}
  stdc++
  ${M_LIB}
  ${CBLAS_LIBRARIES}
  ${QHULL_LIBRARIES}
)
