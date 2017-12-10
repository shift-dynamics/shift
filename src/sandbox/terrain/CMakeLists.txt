cmake_minimum_required(VERSION 2.8.11)
project(elevationmap_test)

list(INSERT CMAKE_MODULE_PATH 0 "${elevationmap_test_SOURCE_DIR}/../../../cmake")

find_package(jansson REQUIRED)
find_package(cblas REQUIRED)
find_library(M_LIB m REQUIRED)
find_package(ImageMagick REQUIRED)
find_package(ImageMagick COMPONENTS MagickWand REQUIRED)
set(ENV{GDAL_ROOT} ${CMAKE_SOURCE_DIR}/../../third_party/gdal/gdal)
set(ENV{GDAL_DIR} ${CMAKE_SOURCE_DIR}/../../third_party/gdal/gdal/apps)
find_package(GDAL REQUIRED)


add_executable(gen_elevationmap main.c)

add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)
add_definitions(-ggdb)

add_library(
  kinematics
  STATIC
  ../../core/kinematics.c)

add_library(
  elevationmap
  STATIC
  ../../core/terrain.c
  ../../core/stack.c)

target_link_libraries(
  gen_elevationmap
  ${ImageMagick_LIBRARIES}
  ${ImageMagick_MagickWand_LIBRARIES}
  ${JANSSON_LIBRARIES}
  ${M_LIB}
  ${CBLAS_LIBRARIES}
  ${GDAL_LIBRARIES}
  elevationmap
  kinematics)

include_directories(
  ${ImageMagick_INCLUDE_DIRS}
  ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${JANSSON_INCLUDE_DIRS}
  ${CBLAS_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
  ../../ksl
  ../../core)
