cmake_minimum_required(VERSION 2.8.11)
project(basic_framework)
# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang)
message( ${CMAKE_C_COMPILER} )
message( ${CMAKE_CXX_COMPILER} )

set(VERSION_MAJOR 1 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

set(CMAKE_MODULE_PATH "${basic_framework_SOURCE_DIR}/../../../cmake")

## find Jansson and math libraries ##
find_package(jansson REQUIRED)
find_package(cblas REQUIRED)

find_library(M_LIB m REQUIRED)
find_library(stdc++ REQUIRED)
find_package(assimp REQUIRED)
# find_package( clapack REQUIRED )
# find_package( graphviz REQUIRED )

set(LIBS
  # ${GRAPHVIZ_GVC_LIBRARY}
  # ${GRAPHVIZ_CGRAPH_LIBRARY}
    ${ASSIMP_LIBRARIES}
    ${LIBS}
    ${JANSSON_LIBRARIES}
    ${M_LIB}
    ${CBLAS_LIBRARIES})
message(${LIBS})

# find_library( LAPACK lapack)
# find_library( PTHREAD pthread )

## set output directory ##
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

add_definitions(-ggdb) # for debug statements, comment this out for production code
add_definitions(-std=c11)
add_definitions(-O3)
add_definitions(-Wall)
add_definitions(-D_GNU_SOURCE=1)
add_definitions(-pipe)
add_definitions(-fPIC)
# add_definitions(${GRAPHVIZ_COMPILE_FLAGS})

# if(APPLE)
#     add_definitions(-framework accelerate)
# endif(APPLE)
# message(${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY})
# message(${GRAPHVIZ_INCLUDE_DIR})

include_directories(
    ${GRAPHVIZ_INCLUDE_DIR}
    ${JANSSON_INCLUDE_DIRS}
    ${CBLAS_INCLUDE_DIRS}
    ${basic_framework_SOURCE_DIR}/../../mbdl/core
    ${basic_framework_SOURCE_DIR}/../../mbdl/gui
    ${basic_framework_SOURCE_DIR}/../../mbdl
    ${ASSIMP_INCLUDE_DIRS}
    # ${mbd_SOURCE_DIR}/src/hsl/hsl_ma48-3.3.0/example/C
    # ${mbd_SOURCE_DIR}/src/hsl/hsl_ma48-3.3.0/src)
)

### build static library ###
add_library(mbdl_lib STATIC
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/context.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/kinematics.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/utility.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/loader.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/queue.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/stack.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/topology.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/joint.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/graph.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/geometry.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/camera.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/jacobian.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/factor.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/solve.c
    ${basic_framework_SOURCE_DIR}/../../mbdl/core/equation.c
    # ${mbdl_SOURCE_DIR}/src/mbdl/core/newtonRaphson.c
    # ${mbdl_SOURCE_DIR}/src/mbdl/core/mobility.c
    # ${mbdl_SOURCE_DIR}/src/mbdl/core/writer.c
    # ${mbdl_SOURCE_DIR}/src/mbdl/core/service.c
)


### build command line test applications ###
# add_executable(mbdl_cli
# ${mbdl_SOURCE_DIR}/src/mbdl/application/mbdl_cli.c)
# target_link_libraries(mbdl_cli mbdl_lib ${LIBS})


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
    ${basic_framework_SOURCE_DIR}/main.cpp
    ${basic_framework_SOURCE_DIR}/framework.cpp
    ${basic_framework_SOURCE_DIR}/openglwindow.cpp
    ${basic_framework_SOURCE_DIR}/renderwindow.cpp
    ${basic_framework_SOURCE_DIR}/resources.qrc
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

add_executable(basic_framework ${GUI_SOURCES})
qt5_use_modules(basic_framework Core Widgets OpenGL Gui)
target_link_libraries(basic_framework
    mbdl_lib
    ${OPENGL_LIBRARIES}
    stdc++
    ${CBLAS_LIBRARIES}
    ${LIBS}
)
