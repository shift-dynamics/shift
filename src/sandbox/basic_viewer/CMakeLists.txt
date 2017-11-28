cmake_minimum_required(VERSION 2.8.11)
project(viewer)
message( ${CMAKE_C_COMPILER} )
message( ${CMAKE_CXX_COMPILER} )

set(VERSION_MAJOR 1 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

set(CMAKE_MODULE_PATH "${viewer_SOURCE_DIR}/../../../cmake")

## find Jansson and math libraries ##
list(INSERT CMAKE_MODULE_PATH 0 "${star2_SOURCE_DIR}/cmake")
if(APPLE)
  list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/qt5/5.7.0/lib/cmake/Qt5Core")
endif(APPLE)
find_package(jansson REQUIRED)
find_package(cblas REQUIRED)
find_library(M_LIB m REQUIRED)
find_package(graphviz REQUIRED)
find_library(stdc++ REQUIRED)
find_package(assimp REQUIRED)
find_package(Qhull REQUIRED)
find_package(ImageMagick REQUIRED)
find_package(ImageMagick COMPONENTS MagickWand REQUIRED)


set( LIBS
    ${ASSIMP_LIBRARIES}
    ${GRAPHVIZ_GVC_LIBRARY}
    ${GRAPHVIZ_CGRAPH_LIBRARY}
    ${LIBS}
    ${JANSSON_LIBRARIES}
    ${M_LIB}
    ${CBLAS_LIBRARIES}
    ${ImageMagick_LIBRARIES}
    ${ImageMagick_MagickWand_LIBRARIES}
    ${QHULL_LIBRARIES})
message( "using libraries: " ${LIBS} )
message( "qhull: " ${QHULL_LIBRARIES})

# find_library( LAPACK lapack)
# find_library( PTHREAD pthread )

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
add_definitions(${GRAPHVIZ_COMPILE_FLAGS})
add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)

if(APPLE)
  add_definitions(-framework accelerate)
endif(APPLE)
message(${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY})
# message(${GRAPHVIZ_INCLUDE_DIR})

include_directories(
  ${GRAPHVIZ_INCLUDE_DIR}
  ${JANSSON_INCLUDE_DIRS}
  ${CBLAS_INCLUDE_DIRS}
  ${viewer_SOURCE_DIR}/../../core
  ${viewer_SOURCE_DIR}/../../ode
  ${viewer_SOURCE_DIR}/../../gui
  ${viewer_SOURCE_DIR}/../../
  ${ASSIMP_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
  ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
)


set(LIBS
  ${ASSIMP_LIBRARIES}
  ${LIBS}
  ${JANSSON_LIBRARIES}
  ${M_LIB}
  ${CBLAS_LIBRARIES})
message(${LIBS})

## set output directory ##
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

add_definitions(-ggdb) # for debug statements, comment this out for production code
add_definitions(-std=c11)
add_definitions(-O3)
add_definitions(-Wall)
add_definitions(-D_GNU_SOURCE=1)
add_definitions(-pipe)
add_definitions(-fPIC)


### build static library ###
add_library(star2_lib STATIC
  ${viewer_SOURCE_DIR}/../../core/assembly.c
  ${viewer_SOURCE_DIR}/../../core/body.c
  ${viewer_SOURCE_DIR}/../../core/camera.c
  ${viewer_SOURCE_DIR}/../../core/chordconstraint.c
  ${viewer_SOURCE_DIR}/../../core/context.c
  ${viewer_SOURCE_DIR}/../../core/convexhull.c
  ${viewer_SOURCE_DIR}/../../core/equation.c
  ${viewer_SOURCE_DIR}/../../core/factor.c
  ${viewer_SOURCE_DIR}/../../core/frame.c
  ${viewer_SOURCE_DIR}/../../core/geometry.c
  ${viewer_SOURCE_DIR}/../../core/graph.c
  ${viewer_SOURCE_DIR}/../../core/inertia.c
  ${viewer_SOURCE_DIR}/../../core/io.c
  ${viewer_SOURCE_DIR}/../../core/jacobian.c
  ${viewer_SOURCE_DIR}/../../core/joint.c
  ${viewer_SOURCE_DIR}/../../core/kinematics.c
  ${viewer_SOURCE_DIR}/../../core/light.c
  ${viewer_SOURCE_DIR}/../../core/loader.c
  ${viewer_SOURCE_DIR}/../../core/material.c
  ${viewer_SOURCE_DIR}/../../core/memoize.c
  ${viewer_SOURCE_DIR}/../../core/mobility.c
  ${viewer_SOURCE_DIR}/../../core/newtonraphson.c
  ${viewer_SOURCE_DIR}/../../core/queue.c
  ${viewer_SOURCE_DIR}/../../core/restraint.c
  ${viewer_SOURCE_DIR}/../../core/sensor.c
  ${viewer_SOURCE_DIR}/../../core/solve.c
  ${viewer_SOURCE_DIR}/../../core/stack.c
  ${viewer_SOURCE_DIR}/../../core/substructure.c
  ${viewer_SOURCE_DIR}/../../core/terrain.c
  ${viewer_SOURCE_DIR}/../../core/topology.c
  ${viewer_SOURCE_DIR}/../../core/utility.c
  ${viewer_SOURCE_DIR}/../../force_modules/service.c
  ${viewer_SOURCE_DIR}/../../force_modules/tire.c
  ${viewer_SOURCE_DIR}/../../force_modules/tire/pac2002.c
  ${viewer_SOURCE_DIR}/../../force_modules/tire/fiala.c
  ${viewer_SOURCE_DIR}/../../force_modules/stiffness.c
  ${viewer_SOURCE_DIR}/../../force_modules/damping.c
  ${viewer_SOURCE_DIR}/../../force_modules/forcemodule.c
  ${viewer_SOURCE_DIR}/../../force_modules/pid.c
  ${viewer_SOURCE_DIR}/../../ode/numericalintegrator.c
  ${viewer_SOURCE_DIR}/../../ode/ode113.c
  ${viewer_SOURCE_DIR}/../../ode/ode113b.c
)

add_library(ode113_lib STATIC
  ${viewer_SOURCE_DIR}/../../ode/ode113.c
)


### build basic opengl framework ###
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
set(
  GUI_SOURCES
  ${viewer_SOURCE_DIR}/src/main.cpp
  ${viewer_SOURCE_DIR}/src/basicviewer.cpp
  ${viewer_SOURCE_DIR}/src/basicrenderwindow.cpp
  ${viewer_SOURCE_DIR}/../../gui/openglwindow.cpp
  ${viewer_SOURCE_DIR}/../../gui/orbitalcamera.cpp
  ${viewer_SOURCE_DIR}/src/resources.qrc
)

# qt5_use_modules does not set all the necessary flags to build the
# program
remove_definitions(-std=c11)
add_definitions(-march=x86-64)
add_definitions(-mtune=generic)
add_definitions(-fstack-protector-strong)
add_definitions(-D_REENTRANT)
add_definitions(-DQT_NO_DEBUG)
add_definitions(-DQT_OPENGL_LIB)
add_definitions(-DQT_WIDGETS_LIB)
add_definitions(-DQT_GUI_LIB)
add_definitions(-DQT_CORE_LIB)
add_definitions(--param=ssp-buffer-size=4)
add_definitions(-W)
add_definitions(-lstdc++)
# add_definitions(--sort-common)
# add_definitions(--as-needed)
# add_definitions(-z)
# add_definitions(relro)

add_executable(viewer ${GUI_SOURCES})
qt5_use_modules(viewer Core Widgets OpenGL Gui)
target_link_libraries(viewer
  star2_lib
  ${OPENGL_LIBRARIES}
  stdc++
  ${CBLAS_LIBRARIES}
  ${LIBS}
)
