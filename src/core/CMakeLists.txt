find_package(cblas)

set(shift_core_src

)

add_library(shift_core STATIC
  assembly.cc
  base.cc
  context.cc
  factory.cc
  frame.cc
  modelparameters.cc
  jacobian.cc
  kdtree.cc
  substructure.cc
  topology.cc
  units.cc
)

add_subdirectory(parameter)
add_subdirectory(behavior)
add_subdirectory(body)
add_subdirectory(collisiongeometry)
add_subdirectory(constraint)
add_subdirectory(forcemodule)
add_subdirectory(geometry)
add_subdirectory(integrator)
add_subdirectory(joint)
add_subdirectory(light)
add_subdirectory(logger)
add_subdirectory(sensor)
add_subdirectory(solver)
add_subdirectory(terrain)

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
  ${CMAKE_BINARY_DIR}/include
  ${JSONCPP_INCLUDE_DIR}
)

target_link_libraries(
  shift_core
  shift_core_parameter
  shift_core_behavior
  shift_core_body
  shift_core_collisiongeometry
  shift_core_constraint
  shift_core_forcemodule
  shift_core_geometry
  shift_core_integrator
  shift_core_joint
  shift_core_light
  shift_core_logger
  shift_core_sensor
  shift_core_solver
  shift_core_terrain
  ${JsonCpp_LIBRARY}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${CBLAS_LIBRARIES}
  ksl
)
