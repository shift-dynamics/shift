find_package(cblas)

set(shift_core_src

  assembly.cc
  base.cc
  context.cc
  factory.cc
  frame.cc
  modelparameters.cc
  jacobian.cc
  substructure.cc
  topology.cc
  units.cc

  parameter/value.cc
  parameter/factoryarray.cc
  parameter/list.cc
  parameter/array.cc
  parameter/arraybase.cc

  behavior/base.cc
  behavior/contactdetection.cc
  behavior/ik_7dof.cc

  body/base.cc
  body/rigid.cc
  body/free.cc
  body/fixed.cc
  body/virtual.cc

  collisiongeometry/base.cc

  constraint/base.cc
  constraint/chord.cc

  forcemodule/base.cc
  forcemodule/contact.cc
  forcemodule/pid.cc
  forcemodule/tire/fiala.cc

  geometry/base.cc
  geometry/ellipsoid.cc
  geometry/mesh.cc
  geometry/pointcloud.cc
  geometry/sphere.cc

  integrator/base.cc
  integrator/ode113.cc

  joint/base.cc
  joint/float.cc
  joint/prismatic.cc
  joint/revolute.cc
  joint/rigid.cc
  joint/spherical.cc

  kdtree/kdtree.cc

  light/base.cc
  light/directional.cc
  light/ies.cc
  light/point.cc
  light/spot.cc

  logger/base.cc
  logger/csv.cc
  logger/hdf5.cc

  sensor/base.cc
  sensor/imu.cc
  sensor/range.cc

  solver/base.cc
  solver/rnca.cc

  terrain/base.cc
  terrain/basic.cc
)

include_directories(${CMAKE_BINARY_DIR}/include)

add_library(shift_core STATIC ${shift_core_src})

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_definitions(
    shift_core
    PRIVATE
    QT_NO_DEBUG
  )
endif()

target_compile_options(
  shift_core
  PRIVATE
  -march=x86-64
  -mtune=generic
  -fstack-protector-strong
  --param=ssp-buffer-size=4
  -W
  -lstdc++
  -Wall
  -Wextra
  -g3
  # -O3
  -std=c++17
  ${GRAPHVIZ_COMPILE_FLAGS}
  -DSHIFT_VERIFY_EQUATIONS_=1
)

target_include_directories(
  shift_core
  PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/ksl/include
  ${JSONCPP_INCLUDE_DIR}
)

target_link_libraries(
  shift_core
  ${JsonCpp_LIBRARY}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${CBLAS_LIBRARIES}
  ksl
)
