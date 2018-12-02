find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(shift_core_src
  assembly.cc
  base.cc
  context.cc
  frame.cc
  modelparameters.cc
  serialize.cc
  substructure.cc

  behaviors/behavior.cc
  behaviors/contactdetection.cc
  behaviors/ik_7dof.cc

  bodies/body.cc
  bodies/rigid.cc
  bodies/free.cc

  collisiongeometry/collisiongeometry.cc

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
  joints/rigid.cc
  joints/spherical.cc
  joints/revolute.cc

  lights/light.cc
  lights/directional.cc
  lights/ies.cc
  lights/point.cc
  lights/spot.cc

  sensors/sensor.cc
  sensors/imu.cc
  sensors/range.cc

  solvers/solver.cc
  solvers/rtonca.cc

  terrain/terrain.cc
  terrain/basic.cc
)

include_directories(${CMAKE_SOURCE_DIR}/include/shift-core)

add_library(shift_core STATIC ${shift_core_src})

qt5_use_modules(shift_core Core Widgets Gui)

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
  -std=c++11
)

target_include_directories(
  shift_core
  PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_include_directories(
  shift_core
  PUBLIC
  ${CMAKE_SOURCE_DIR}/external/ksl/include
)
