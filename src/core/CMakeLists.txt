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
  body.cc
  context.cc
  frame.cc
  integrator_ode113.cc
  integratorbase.cc
  joint.cc
  modelparameters.cc
  pointcloud.cc
  sensor.cc
  serialize.cc
  solverbase.cc
  sparsesolver.cc
  substructure.cc
  terrain.cc
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
