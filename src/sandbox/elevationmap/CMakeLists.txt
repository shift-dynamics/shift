cmake_minimum_required(VERSION 2.8.11)
project(elevationmap_test)


find_package(ImageMagick REQUIRED)
find_package(ImageMagick COMPONENTS MagickWand REQUIRED)
find_package(cblas REQUIRED)
add_executable(gen_elevationmap main.c)
target_link_libraries(
  gen_elevationmap
  ${ImageMagick_LIBRARIES}
  ${ImageMagick_MagickWand_LIBRARIES}
  ${CBLAS_LIBRARIES})
include_directories(
  ${ImageMagick_INCLUDE_DIRS} ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${CBLAS_INCLUDE_DIRS})
add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)
add_definitions(-ggdb)
find_library( M_LIB m REQUIRED )
include_directories(../../ksl)
include_directories(../../core)
add_library(kinematics STATIC ../../core/kinematics.c)
add_definitions(-ggdb)
add_library(elevationmap STATIC ../../core/elevationmap.c)
add_definitions(-ggdb)
target_link_libraries(gen_elevationmap ${M_LIB} elevationmap kinematics)
