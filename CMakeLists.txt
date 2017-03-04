cmake_minimum_required(VERSION 2.8.11)
project( star2 )
message( ${CMAKE_C_COMPILER} )
message( ${CMAKE_CXX_COMPILER} )

# set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake )
set( VERSION_MAJOR   2  CACHE STRING "Project major version number." )
set( VERSION_MINOR   0  CACHE STRING "Project minor version number." )
set( VERSION_PATCH   0  CACHE STRING "Project patch version number." )
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

## find Jansson and math libraries ##
list( APPEND CMAKE_MODULE_PATH "${star2_SOURCE_DIR}/cmake" )
if(APPLE)
  list(APPEND CMAKE_MODULE_PATH "/usr/local/Cellar/qt5/5.7.0/lib/cmake/Qt5Core")
endif(APPLE)
find_package( jansson REQUIRED )
find_package( cblas REQUIRED )
# find_package( clapack REQUIRED )
find_library( M_LIB m REQUIRED )
find_package( graphviz REQUIRED )
find_library( stdc++ REQUIRED )
find_package( assimp REQUIRED )

set( LIBS
    ${ASSIMP_LIBRARIES}
    ${GRAPHVIZ_GVC_LIBRARY}
    ${GRAPHVIZ_CGRAPH_LIBRARY}
    ${LIBS}
    ${JANSSON_LIBRARIES}
    ${M_LIB}
    ${CBLAS_LIBRARIES} )
message( ${LIBS} )

# find_library( LAPACK lapack)
# find_library( PTHREAD pthread )

## set output directory ##
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/ )

# add_definitions(-ggdb)
add_definitions(-std=c11)
add_definitions(-O3)
add_definitions(-Wall)
add_definitions(-D_GNU_SOURCE=1)
add_definitions(-pipe)
add_definitions(-fPIC)
add_definitions(${GRAPHVIZ_COMPILE_FLAGS})

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
    # ${mbd_SOURCE_DIR}/src/hsl/hsl_ma48-3.3.0/example/C
    # ${mbd_SOURCE_DIR}/src/hsl/hsl_ma48-3.3.0/src
)

### build static library ###
add_library(star2_lib STATIC
    ${star2_SOURCE_DIR}/src/core/context.c
    ${star2_SOURCE_DIR}/src/core/kinematics.c
    ${star2_SOURCE_DIR}/src/core/utility.c
    ${star2_SOURCE_DIR}/src/core/loader.c
    ${star2_SOURCE_DIR}/src/core/queue.c
    ${star2_SOURCE_DIR}/src/core/stack.c
    ${star2_SOURCE_DIR}/src/core/topology.c
    ${star2_SOURCE_DIR}/src/core/joint.c
    ${star2_SOURCE_DIR}/src/core/graph.c
    ${star2_SOURCE_DIR}/src/core/geometry.c
    ${star2_SOURCE_DIR}/src/core/camera.c
    ${star2_SOURCE_DIR}/src/core/jacobian.c
    ${star2_SOURCE_DIR}/src/core/factor.c
    ${star2_SOURCE_DIR}/src/core/solve.c
    ${star2_SOURCE_DIR}/src/core/inertia.c
    ${star2_SOURCE_DIR}/src/core/equation.c
    ${star2_SOURCE_DIR}/src/core/memoize.c
    ${star2_SOURCE_DIR}/src/core/mobility.c
    ${star2_SOURCE_DIR}/src/core/restraint.c
    ${star2_SOURCE_DIR}/src/core/newtonraphson.c
    ${star2_SOURCE_DIR}/src/core/terrain.c
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
    # ${star2_SOURCE_DIR}/src/mbdl/core/writer.c
)

add_library(ode113_lib STATIC
  ${star2_SOURCE_DIR}/src/ode/ode113.c
)

# add_definitions(-pedantic)
add_definitions(-Wall)
add_definitions(-Wextra)

### build command line test applications ###
add_executable(star2_cli ${star2_SOURCE_DIR}/src/application/star2_cli.c)
target_link_libraries(star2_cli star2_lib ode113_lib ${LIBS})

add_executable(7r_robot ${star2_SOURCE_DIR}/src/application/7r_robot.c)
target_link_libraries(7r_robot star2_lib ode113_lib ${LIBS})

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
    file(COPY
     ${star2_SOURCE_DIR}/doc/doxygen-bootstrapped/doxy-boot.js
     DESTINATION
    ${CMAKE_CURRENT_BINARY_DIR}/html)

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
