cmake_minimum_required(VERSION 2.8.11)
project(shift)

set(VERSION_MAJOR 0 CACHE STRING "Project major version number.")
set(VERSION_MINOR 1 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

if (DEFINED _SHIFT_VERBOSE)
  add_definitions(-D_SHIFT_VERBOSE=${_SHIFT_VERBOSE})
else()
  add_definitions(-D_SHIFT_VERBOSE=0)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(jansson REQUIRED)
find_package(cblas REQUIRED)
find_library(M_LIB m REQUIRED)
find_package(graphviz REQUIRED)
find_package(assimp REQUIRED)
find_package(Qhull REQUIRED)
find_package(ImageMagick REQUIRED)
find_package(ImageMagick COMPONENTS MagickWand REQUIRED)
add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)


set(LIBS
  ${LIBS}
  ${ASSIMP_LIBRARIES}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${JANSSON_LIBRARIES}
  ${M_LIB}
  ${CBLAS_LIBRARIES}
  ${ImageMagick_LIBRARIES}
  ${ImageMagick_MagickWand_LIBRARIES}
  ${QHULL_LIBRARIES}
)

include_directories(
  ${GRAPHVIZ_INCLUDE_DIR}
  ${JANSSON_INCLUDE_DIRS}
  ${CBLAS_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
  ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
)

if(APPLE)
  list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/qt5/5.7.0/lib/cmake/Qt5Core")
endif(APPLE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

if(APPLE)
  add_definitions(-framework accelerate)
endif(APPLE)

include(pandocology)

add_document(
  TARGET about
  OUTPUT_FILE about.html
  SOURCES readme.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/html"
)

add_document(
  TARGET license
  OUTPUT_FILE license.html
  SOURCES license.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/html"
)

include_directories(src)
include_directories(third_party/shift-core/src)

add_subdirectory(third_party)
add_subdirectory(src)
add_subdirectory(doc)

#file(COPY
#  ${CMAKE_CURRENT_LIST_DIR}/third_party/shift-core/external/shift-models
#  DESTINATION
#  ${CMAKE_CURRENT_BINARY_DIR})
