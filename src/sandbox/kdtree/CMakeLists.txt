cmake_minimum_required(VERSION 2.8.11)
project(kdtree)


## find Jansson and math libraries ##
list(INSERT CMAKE_MODULE_PATH 0 "${kdtree_SOURCE_DIR}/cmake")
if(APPLE)
  list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/qt5/5.7.0/lib/cmake/Qt5Core")
endif(APPLE)
find_library(M_LIB m REQUIRED)
find_library(stdc++ REQUIRED)
find_package(assimp REQUIRED)
find_package(Qhull REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(cblas REQUIRED)
find_package(Threads REQUIRED)


set(LIBS
  ${LIBS}
  ${ASSIMP_LIBRARIES}
  ${M_LIB}
  ${CBLAS_LIBRARIES}
  ${QHULL_LIBRARIES})
message("using libraries: " ${LIBS})

## set output directory ##
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/ )

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


include_directories(
  ${CBLAS_INCLUDE_DIRS})
add_definitions(-ggdb)
find_library( M_LIB m REQUIRED )
include_directories(../../ksl)
include_directories(../../core)
add_definitions(-ggdb)
add_definitions(-O3)
add_definitions(-D_GNU_SOURCE=1)

include_directories(
    ${kdtree_SOURCE_DIR}/../../ksl
    ${kdtree_SOURCE_DIR}/../../core
    ${kdtree_SOURCE_DIR}/src/gui
    ${kdtree_SOURCE_DIR}/src
    ${ASSIMP_INCLUDE_DIRS}
    ${QHULL_INCLUDE_DIRS}
)

### build static library ###
add_library(kdtree_lib STATIC
    ${kdtree_SOURCE_DIR}/../../core/factor.c
    ${kdtree_SOURCE_DIR}/../../core/kdtree.c
    ${kdtree_SOURCE_DIR}/../../core/kinematics.c
    ${kdtree_SOURCE_DIR}/../../core/geometry.c
)

# add_definitions(-pedantic)
add_definitions(-Wall)
add_definitions(-Wextra)

add_executable(kdtree_nntest ${kdtree_SOURCE_DIR}/src/kdtree_nntest.c)
target_link_libraries(
  kdtree_nntest
  ${M_LIB}
  kdtree_lib
  Threads::Threads
  ${CBLAS_LIBRARIES})

add_executable(kdtree_raytest ${kdtree_SOURCE_DIR}/src/kdtree_raytest.c)
target_link_libraries(
  kdtree_raytest
  ${M_LIB}
  kdtree_lib
  Threads::Threads
  ${CBLAS_LIBRARIES})

### build graphical user interface ###
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
set(GUI_SOURCES
  ${kdtree_SOURCE_DIR}/src/gui/shift_gui.cpp
  ${kdtree_SOURCE_DIR}/src/gui/workspace.cpp
  ${kdtree_SOURCE_DIR}/src/gui/openglwindow.cpp
  ${kdtree_SOURCE_DIR}/src/gui/renderwindow.cpp
  ${kdtree_SOURCE_DIR}/src/gui/geometry.cpp
  ${kdtree_SOURCE_DIR}/src/gui/bodies.cpp
  ${kdtree_SOURCE_DIR}/src/gui/frames.cpp
  ${kdtree_SOURCE_DIR}/src/gui/joints.cpp
  ${kdtree_SOURCE_DIR}/src/gui/grid.cpp
  ${kdtree_SOURCE_DIR}/src/gui/lights.cpp
  ${kdtree_SOURCE_DIR}/src/gui/terrain.cpp
  ${kdtree_SOURCE_DIR}/src/gui/flatshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/frameshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/simpleshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/phongshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/shaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/shadowshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/terrainshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/terrainshadowshaderprogram.cpp
  ${kdtree_SOURCE_DIR}/src/gui/orbitalcamera.cpp
  ${kdtree_SOURCE_DIR}/src/gui/resources.qrc
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
add_definitions(-pedantic)
add_definitions(-Wall)
add_definitions(-Wextra)

add_executable(shift ${GUI_SOURCES})
qt5_use_modules(shift Core Widgets OpenGL Gui Svg)
target_link_libraries(shift
  shift_lib
  ode113_lib
  ${OPENGL_LIBRARIES}
  stdc++
  ${CBLAS_LIBRARIES}
  ${LIBS}
)
