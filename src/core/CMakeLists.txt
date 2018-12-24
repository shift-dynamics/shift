set(shift_core_src

  serialize.cc
  assembly.cc
  base.cc
  context.cc
  factory.cc
  frame.cc
  modelparameters.cc
  substructure.cc
  topology.cc
  units.cc

  behaviors/behavior.cc
  behaviors/contactdetection.cc
  behaviors/ik_7dof.cc

  bodies/body.cc
  bodies/rigid.cc
  bodies/free.cc
  bodies/fixed.cc

  collisiongeometry/collisiongeometry.cc

  constraints/constraint.cc

  behaviors/behavior.cc
  behaviors/contactdetection.cc
  behaviors/ik_7dof.cc

  forcemodules/forcemodule.cc
  forcemodules/contact.cc
  forcemodules/pid.cc

  geometry/geometry.cc
  geometry/ellipsoid.cc
  geometry/mesh.cc
  geometry/pointcloud.cc
  geometry/sphere.cc

  integrators/integrator.cc
  integrators/ode113.cc

  joints/joint.cc
  joints/float.cc
  joints/prismatic.cc
  joints/revolute.cc
  joints/rigid.cc
  joints/spherical.cc

  lights/light.cc
  lights/directional.cc
  lights/ies.cc
  lights/point.cc
  lights/spot.cc

  loggers/logger.cc
  loggers/csv.cc
  loggers/hdf5.cc

  sensors/sensor.cc
  sensors/imu.cc
  sensors/range.cc

  solvers/solver.cc
  solvers/rnca.cc

  terrain/terrain.cc
  terrain/basic.cc
)

include_directories(${CMAKE_SOURCE_DIR}/include/shift-core)

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
  -O3
  -std=c++17
)

target_include_directories(
  shift_core
  PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/ksl/include
  ${JSONCPP_INCLUDE_DIR}
)

target_link_libraries(shift_core ${JsonCpp_LIBRARY})
