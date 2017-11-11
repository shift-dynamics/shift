cmake_minimum_required(VERSION 2.8.11)
project(star2)
message(${CMAKE_C_COMPILER})
message(${CMAKE_CXX_COMPILER})

# set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake )
set(VERSION_MAJOR 2 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

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


set(LIBS
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
message("using libraries: " ${LIBS})
message("qhull: " ${QHULL_LIBRARIES})

# find_library( LAPACK lapack)
# find_library( PTHREAD pthread )

## set output directory ##
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

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
  ${star2_SOURCE_DIR}/src/core
  ${star2_SOURCE_DIR}/src/ode
  ${star2_SOURCE_DIR}/src/gui
  ${star2_SOURCE_DIR}/src
  ${ASSIMP_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
  ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
)

add_library(ksl STATIC
  ${star2_SOURCE_DIR}/src/ksl/ksl.c
)

add_library(bksl STATIC
  ${star2_SOURCE_DIR}/src/ksl/bksl.c
)

### build static library ###
add_library(star2_lib STATIC
    ${star2_SOURCE_DIR}/src/core/assembly.c
    ${star2_SOURCE_DIR}/src/core/body.c
    ${star2_SOURCE_DIR}/src/core/camera.c
    ${star2_SOURCE_DIR}/src/core/chordconstraint.c
    ${star2_SOURCE_DIR}/src/core/context.c
    ${star2_SOURCE_DIR}/src/core/convexhull.c
    ${star2_SOURCE_DIR}/src/core/equation.c
    ${star2_SOURCE_DIR}/src/core/factor.c
    ${star2_SOURCE_DIR}/src/core/frame.c
    ${star2_SOURCE_DIR}/src/core/graph.c
    ${star2_SOURCE_DIR}/src/core/inertia.c
    ${star2_SOURCE_DIR}/src/core/io.c
    ${star2_SOURCE_DIR}/src/core/jacobian.c
    ${star2_SOURCE_DIR}/src/core/joint.c
    ${star2_SOURCE_DIR}/src/core/kinematics.c
    ${star2_SOURCE_DIR}/src/core/light.c
    ${star2_SOURCE_DIR}/src/core/material.c
    ${star2_SOURCE_DIR}/src/core/memoize.c
    ${star2_SOURCE_DIR}/src/core/mobility.c
    ${star2_SOURCE_DIR}/src/core/newtonraphson.c
    ${star2_SOURCE_DIR}/src/core/queue.c
    ${star2_SOURCE_DIR}/src/core/restraint.c
    ${star2_SOURCE_DIR}/src/core/sensor.c
    ${star2_SOURCE_DIR}/src/core/solve.c
    ${star2_SOURCE_DIR}/src/core/stack.c
    ${star2_SOURCE_DIR}/src/core/substructure.c
    ${star2_SOURCE_DIR}/src/core/terrain.c
    ${star2_SOURCE_DIR}/src/core/topology.c
    ${star2_SOURCE_DIR}/src/core/utility.c
    ${star2_SOURCE_DIR}/src/force_modules/service.c
    ${star2_SOURCE_DIR}/src/force_modules/tire.c
    ${star2_SOURCE_DIR}/src/force_modules/tire/pac2002.c
    ${star2_SOURCE_DIR}/src/force_modules/tire/fiala.c
    ${star2_SOURCE_DIR}/src/force_modules/stiffness.c
    ${star2_SOURCE_DIR}/src/force_modules/damping.c
    ${star2_SOURCE_DIR}/src/force_modules/forcemodule.c
    ${star2_SOURCE_DIR}/src/force_modules/pid.c
    ${star2_SOURCE_DIR}/src/ode/numericalintegrator.c
    ${star2_SOURCE_DIR}/src/ode/ode113.c
    ${star2_SOURCE_DIR}/src/ode/ode113b.c
    # ${star2_SOURCE_DIR}/src/core/writer.c
)

add_library(ode113_lib STATIC
  ${star2_SOURCE_DIR}/src/ode/ode113.c
)

# add_definitions(-pedantic)
add_definitions(-Wall)
add_definitions(-Wextra)

### build command line test applications ###
add_executable(star2_cli ${star2_SOURCE_DIR}/src/application/star2_cli.c)
target_link_libraries(star2_cli star2_lib ode113_lib ksl ${LIBS})

add_executable(7r_robot ${star2_SOURCE_DIR}/src/application/7r_robot.c)
target_link_libraries(7r_robot star2_lib ode113_lib ${LIBS})

## build ksl test functions
add_executable(ksl_test ${star2_SOURCE_DIR}/src/ksl/ksl_test.c)
target_link_libraries(ksl_test ksl bksl ${M_LIB} ${CBLAS_LIBRARIES})

### build ode test applications ###
# add_executable(ode113_test
# ${star2_SOURCE_DIR}/src/mbdl/application/ode113_test.c)
# target_link_libraries(ode113_test ode113_lib ${LIBS})

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
    ${star2_SOURCE_DIR}/src/gui/star2_gui.cpp
    ${star2_SOURCE_DIR}/src/gui/workspace.cpp
    ${star2_SOURCE_DIR}/src/gui/openglwindow.cpp
    ${star2_SOURCE_DIR}/src/gui/renderwindow.cpp
    ${star2_SOURCE_DIR}/src/gui/geometry.cpp
    ${star2_SOURCE_DIR}/src/gui/bodies.cpp
    ${star2_SOURCE_DIR}/src/gui/frames.cpp
    ${star2_SOURCE_DIR}/src/gui/joints.cpp
    ${star2_SOURCE_DIR}/src/gui/grid.cpp
    ${star2_SOURCE_DIR}/src/gui/lights.cpp
    ${star2_SOURCE_DIR}/src/gui/terrain.cpp
    ${star2_SOURCE_DIR}/src/gui/flatshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/frameshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/simpleshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/phongshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/shaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/shadowshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/terrainshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/terrainshadowshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/pointcloudshaderprogram.cpp
    ${star2_SOURCE_DIR}/src/gui/pointcloud.cpp
    ${star2_SOURCE_DIR}/src/gui/orbitalcamera.cpp
    ${star2_SOURCE_DIR}/src/gui/resources.qrc
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
# add_definitions(--sort-common)
# add_definitions(--as-needed)
# add_definitions(-z)
# add_definitions(relro)

add_executable(star2 ${GUI_SOURCES})
qt5_use_modules(star2 Core Widgets OpenGL Gui Svg)
target_link_libraries(star2
    star2_lib
    ode113_lib
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    stdc++
    ${CBLAS_LIBRARIES}
    ${LIBS}
)

file(COPY
    ${star2_SOURCE_DIR}/src/models
    DESTINATION
    ${CMAKE_CURRENT_BINARY_DIR})

### build documentation ###
find_package( Doxygen )
option( BUILD_DOCUMENTATION
    "Create and install the HTML based API documentation (requires Doxygen)"
    ${DOXYGEN_FOUND} )

if( BUILD_DOCUMENTATION )
    if( NOT DOXYGEN_FOUND )
        message( FATAL_ERROR "Doxygen is needed to build the documentation." )
    endif()

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    # generate doxygen documentation
    add_custom_target( doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )

    # copy doxyboot.js file to html directory
    # file(COPY
    #  ${star2_SOURCE_DIR}/doc/doxygen-bootstrapped/doxy-boot.js
    #  DESTINATION
    # ${CMAKE_CURRENT_BINARY_DIR}/html)

    # ## install sphinx documentation to /srv/http
    # install(
    #   DIRECTORY
    #   ${star2_SOURCE_DIR}/doc/sphinx/build/html
    #   DESTINATION
    #   /srv)
    #

    ## install doxygen documentation to /srv/http/mbdl
    install(
      DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/html
      DESTINATION
      /srv/http/static/doxygen)

    install(
      DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/html
      DESTINATION
      /share/doc)



endif()
