cmake_minimum_required(VERSION 2.8.11)
project(shift)
message(${CMAKE_C_COMPILER})
message(${CMAKE_CXX_COMPILER})

# set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} cmake )
set(VERSION_MAJOR 2 CACHE STRING "Project major version number.")
set(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
set(VERSION_PATCH 0 CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

if (DEFINED _SHIFT_VERBOSE)
  add_definitions(-D_SHIFT_VERBOSE=${_SHIFT_VERBOSE})
else()
  add_definitions(-D_SHIFT_VERBOSE=0)
endif()

## find Jansson and math libraries ##
list(INSERT CMAKE_MODULE_PATH 0 "${shift_SOURCE_DIR}/cmake")
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

message(STATUS "ASSIMP_INCLUDE_DIRS" ${ASSIMP_INCLUDE_DIRS})
message(STATUS "ASSIMP_LIBRARIES" ${ASSIMP_LIBRARIES})

# build gdal submodule
add_custom_target(gdal
  COMMAND make -j4
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal
  COMMENT "Building GDAL"
  DEPENDS ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/makefile.vc
  BYPRODUCTS ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/libgdal.la
  VERBATIM)

add_custom_target(gdal_configure
  COMMAND ./configure
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal
  COMMENT "Configuring GDAL"
  BYPRODUCTS ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/makefile.vc
  VERBATIM)

# find gdal submodule
set(ENV{GDAL_ROOT} ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal)
set(ENV{GDAL_DIR} ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/apps)
find_package(GDAL REQUIRED)


# build v-hacd submodule
# add_subdirectory(${CMAKE_SOURCE_DIR}/src/third_party/v-hacd/src)

# build stepcode submodule
# add_subdirectory(${CMAKE_SOURCE_DIR}/src/third_party/stepcode)


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
  ${QHULL_LIBRARIES}
  ${GDAL_LIBRARIES})
message("using libraries: " ${LIBS})
message("qhull: " ${QHULL_LIBRARIES})

# find_library( LAPACK lapack)
# find_library( PTHREAD pthread )

## set output directory ##
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/)

add_definitions(-g3)
add_definitions(-ggdb)
add_definitions(-std=c11)
add_definitions(-O0)
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
# add_definitions(-fsanitize=address)
# add_definitions(-fno-omit-frame-pointer)
#
# set(CMAKE_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

if(APPLE)
  add_definitions(-framework accelerate)
endif(APPLE)
message(${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY})
# message(${GRAPHVIZ_INCLUDE_DIR})

message("GDAL_INCLUDE_DIRS: " ${GDAL_INCLUDE_DIRS})
include_directories(
  ${GRAPHVIZ_INCLUDE_DIR}
  ${JANSSON_INCLUDE_DIRS}
  ${CBLAS_INCLUDE_DIRS}
  ${shift_SOURCE_DIR}/src/core
  ${shift_SOURCE_DIR}/src/ode
  ${shift_SOURCE_DIR}/src/gui
  ${shift_SOURCE_DIR}/src
  ${ASSIMP_INCLUDE_DIRS}
  ${ImageMagick_INCLUDE_DIRS}
  ${ImageMagick_MagickWand_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
)

# add_library(ksl STATIC
#   ${shift_SOURCE_DIR}/src/ksl/ksl.c
# )
#
# add_library(bksl STATIC
#   ${shift_SOURCE_DIR}/src/ksl/bksl.c
# )

### build static library ###
add_library(shift_lib STATIC
  ${shift_SOURCE_DIR}/src/core/assembly.c
  ${shift_SOURCE_DIR}/src/core/body.c
  ${shift_SOURCE_DIR}/src/core/camera.c
  ${shift_SOURCE_DIR}/src/core/chordconstraint.c
  ${shift_SOURCE_DIR}/src/core/context.c
  ${shift_SOURCE_DIR}/src/core/convexhull.c
  ${shift_SOURCE_DIR}/src/core/collisionobject.c
  ${shift_SOURCE_DIR}/src/core/equation.c
  ${shift_SOURCE_DIR}/src/core/factor.c
  ${shift_SOURCE_DIR}/src/core/frame.c
  ${shift_SOURCE_DIR}/src/core/graph.c
  ${shift_SOURCE_DIR}/src/core/inertia.c
  ${shift_SOURCE_DIR}/src/core/io.c
  ${shift_SOURCE_DIR}/src/core/jacobian.c
  ${shift_SOURCE_DIR}/src/core/joint.c
  ${shift_SOURCE_DIR}/src/core/kinematics.c
  ${shift_SOURCE_DIR}/src/core/light.c
  ${shift_SOURCE_DIR}/src/core/material.c
  ${shift_SOURCE_DIR}/src/core/memoize.c
  ${shift_SOURCE_DIR}/src/core/mobility.c
  ${shift_SOURCE_DIR}/src/core/newtonraphson.c
  ${shift_SOURCE_DIR}/src/core/pca.c
  ${shift_SOURCE_DIR}/src/core/queue.c
  ${shift_SOURCE_DIR}/src/core/restraint.c
  ${shift_SOURCE_DIR}/src/core/sensor.c
  ${shift_SOURCE_DIR}/src/core/solve.c
  ${shift_SOURCE_DIR}/src/core/sphere.c
  ${shift_SOURCE_DIR}/src/core/ellipsoid.c
  ${shift_SOURCE_DIR}/src/core/stack.c
  ${shift_SOURCE_DIR}/src/core/substructure.c
  ${shift_SOURCE_DIR}/src/core/terrain.c
  ${shift_SOURCE_DIR}/src/core/topology.c
  ${shift_SOURCE_DIR}/src/core/utility.c
  ${shift_SOURCE_DIR}/src/force_modules/service.c
  ${shift_SOURCE_DIR}/src/force_modules/tire.c
  ${shift_SOURCE_DIR}/src/force_modules/tire/pac2002.c
  ${shift_SOURCE_DIR}/src/force_modules/tire/fiala.c
  ${shift_SOURCE_DIR}/src/force_modules/stiffness.c
  ${shift_SOURCE_DIR}/src/force_modules/damping.c
  ${shift_SOURCE_DIR}/src/force_modules/forcemodule.c
  ${shift_SOURCE_DIR}/src/force_modules/pid.c
  ${shift_SOURCE_DIR}/src/ode/numericalintegrator.c
  ${shift_SOURCE_DIR}/src/ode/ode113.c
  ${shift_SOURCE_DIR}/src/ode/ode113b.c
)
add_dependencies(shift_lib gdal)

add_library(ode113_lib STATIC
  ${shift_SOURCE_DIR}/src/ode/ode113.c
)

# add_definitions(-pedantic)
add_definitions(-Wall)
add_definitions(-Wextra)

### build command line test applications ###
add_executable(shift_cli ${shift_SOURCE_DIR}/src/application/shift_cli.c)
target_link_libraries(shift_cli shift_lib ode113_lib ${LIBS})
add_dependencies(shift_cli gdal)

message(STATUS "LIBS:" ${LIBS})

add_executable(7r_robot ${shift_SOURCE_DIR}/src/application/7r_robot.c)
target_link_libraries(7r_robot shift_lib ode113_lib ${LIBS})

## build ksl test functions
# add_executable(ksl_test ${shift_SOURCE_DIR}/src/ksl/ksl_test.c)
# target_link_libraries(ksl_test ksl bksl ${M_LIB} ${CBLAS_LIBRARIES})

## convert markdown documents to html for display in Qt application
INCLUDE(pandocology)

add_document(
  TARGET about
  OUTPUT_FILE about.html
  SOURCES readme.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY ${CMAKE_SOURCE_DIR}/src/gui/html
)

add_document(
  TARGET license
  OUTPUT_FILE license.html
  SOURCES license.rst
  PANDOC_DIRECTIVES -t html -s
  PRODUCT_DIRECTORY ${CMAKE_SOURCE_DIR}/src/gui/html
)

### build ode test applications ###
# add_executable(ode113_test
# ${shift_SOURCE_DIR}/src/mbdl/application/ode113_test.c)
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
  ${shift_SOURCE_DIR}/src/gui/shift_gui.cpp
  ${shift_SOURCE_DIR}/src/gui/workspace.cpp
  ${shift_SOURCE_DIR}/src/gui/openglwindow.cpp
  ${shift_SOURCE_DIR}/src/gui/renderwindow.cpp
  ${shift_SOURCE_DIR}/src/gui/geometry.cpp
  ${shift_SOURCE_DIR}/src/gui/bodies.cpp
  ${shift_SOURCE_DIR}/src/gui/frames.cpp
  ${shift_SOURCE_DIR}/src/gui/joints.cpp
  ${shift_SOURCE_DIR}/src/gui/grid.cpp
  ${shift_SOURCE_DIR}/src/gui/lights.cpp
  ${shift_SOURCE_DIR}/src/gui/terrain.cpp
  ${shift_SOURCE_DIR}/src/gui/flatshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/frameshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/simpleshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/phongshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/shaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/shadowshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/terrainshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/normalshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/terrainshadowshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/pointcloudshaderprogram.cpp
  ${shift_SOURCE_DIR}/src/gui/pointcloud.cpp
  ${shift_SOURCE_DIR}/src/gui/orbitalcamera.cpp
  ${shift_SOURCE_DIR}/src/gui/resources.qrc
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
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-g3)
# add_definitions(-fsanitize=address)
# add_definitions(-fno-omit-frame-pointer)
# add_definitions(--sort-common)
# add_definitions(--as-needed)
# add_definitions(-z)
# add_definitions(relro)

add_executable(shift ${GUI_SOURCES})
add_dependencies(shift license about gdal)
qt5_use_modules(shift Core Widgets OpenGL Gui Svg)
target_link_libraries(shift
  shift_lib
  ode113_lib
  ${GLEW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  stdc++
  ${CBLAS_LIBRARIES}
  ${LIBS})

file(COPY
    ${shift_SOURCE_DIR}/src/models
    DESTINATION
    ${CMAKE_CURRENT_BINARY_DIR})

### build documentation ###
find_package(Doxygen)
option(BUILD_DOCUMENTATION
  "Create and install the HTML based API documentation (requires Doxygen, Doxyrest, and Sphinx)"
  ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
  if(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation.")
  endif()

  set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
  set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile)

  configure_file(${doxyfile_in} ${doxyfile} @ONLY)

  # generate doxygen documentation
  add_custom_target(doxygen
    COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)

  set (DOC_NAME "shift documentation")

  set (shift_VERSION_MAJOR 2)
  set (shift_VERSION_MINOR 0)
  set (shift_VERSION_REVISION 0)

  set (shift_VERSION_COMPANY "Kristopher Wehage")
  set (shift_VERSION_YEARS   "2015-2017")

  set (STATIC_RST_DIR ${CMAKE_SOURCE_DIR}/doc/sphinx/doxyrest)
  set (XML_DIR  ${CMAKE_CURRENT_BINARY_DIR}/doc/xml)
  set (RST_DIR  ${CMAKE_SOURCE_DIR}/doc/sphinx/doxyrest)
  set (TMP_DIR  ${CMAKE_CURRENT_BINARY_DIR}/tmp)
  set (HTML_DIR ${CMAKE_SOURCE_DIR}/doc/html)
  set (DOXYREST_FRAME_DIR ${CMAKE_SOURCE_DIR}/doc/doxyrest/frame)

  message(${DOXYREST_FRAME_DIR})

  set (
  	DOXYREST_CMD_LINE
  	${XML_DIR}/index.xml
  	-D g_hasNewLineAfterReturnType
  	-D g_nameDelimiter=::
  	-D g_escapeAsterisks
  	-D g_verbatimToCodeBlock
  	-o ${RST_DIR}/page_index.rst
  	-F ${DOXYREST_FRAME_DIR}
  	-f c_index.rst.in
  	)

  # running doxyrest to convert doxygen to rst format
  add_custom_target(doxyrest
    COMMAND doxyrest ${DOXYREST_CMD_LINE}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/doc
    COMMENT "Generating .rst from Doxygen .xml listing using Doxyrest"
    VERBATIM)

  add_dependencies(doxyrest doxygen)

  # generate sphinx documentation
  add_custom_target(doc
    COMMAND make html
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/doc
    COMMENT "Generating API documentation with Sphinx"
    VERBATIM)

  add_dependencies(doc doxyrest)

  # optionally install doxygen documentation to /share/doc
  install(
    DIRECTORY
    ${CMAKE_SOURCE_DIR}/html
    DESTINATION
    /share/doc)

endif()
