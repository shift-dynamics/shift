cmake_minimum_required(VERSION 3.1)
project(shift
    VERSION 0.1.0
    DESCRIPTION "A Dynamics Engine for Automation and Robotics"
    LANGUAGES C CXX
)

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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

if(APPLE)
  add_definitions(-framework accelerate)
endif(APPLE)

add_definitions(-D_GNU_SOURCE)

include(pandocology)

add_document(
  TARGET about
  OUTPUT_FILE about.html
  SOURCES readme.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/gui/html"
)

add_document(
  TARGET license
  OUTPUT_FILE license.html
  SOURCES license.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/gui/html"
)

include_directories(src/core)
include_directories(src/gui)

add_subdirectory(src)
add_subdirectory(doc)
