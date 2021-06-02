set(
  SHIFT_CORE_COMPILE_FLAGS
  -march=x86-64
  -mtune=generic
  -fstack-protector-strong
  --param=ssp-buffer-size=4
  -W
  -Wall
  -Wextra
  # -g3
  # -O3
  -std=c++17
  -DSHIFT_VERIFY_EQUATIONS_=1
  ${GRAPHVIZ_COMPILE_FLAGS}
)

set(
  SHIFT_CORE_LIBRARIES
  ${JsonCpp_LIBRARY}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${CBLAS_LIBRARIES}
  ksl
)

add_library(shift_core STATIC
  Assembly.cc
  Context.cc
  ModelParameters.cc
  Jacobian.cc
  KdTree.cc
  Substructure.cc
  utility.cc
  log.cc
  Exception.cc
  Factory.cc
  Units.cc
  Topology.cc
  Frame.cc
  Base.cc
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

message(STATUS ${JsonCpp_INCLUDE_DIR})

set(
  SHIFT_CORE_INCLUDE_DIRS
  ${CMAKE_BINARY_DIR}/include
  ${JsonCpp_INCLUDE_DIR}
)

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
  ${SHIFT_CORE_COMPILE_FLAGS}
)

# These linker options could be used to dynamically link code,
# which I am not doing yet. It builds ok in gcc with these options,
# But clang doesn't like them.
#
# set(
#   SHIFT_CORE_LINK_OPTIONS
#   -Wl,-export-dynamic
# )
#
# target_link_options(
#   shift_core
#   PUBLIC
#   ${SHIFT_CORE_LINK_OPTIONS}
# )

target_include_directories(
  shift_core
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_link_libraries(
  shift_core
  PUBLIC
  shift_core_parameter
  shift_core_body
  shift_core_behavior
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
)

