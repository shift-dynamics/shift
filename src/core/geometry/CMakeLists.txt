add_library(
  shift_core_geometry
  Base.cc
  Ellipsoid.cc
  Mesh.cc
  PointCloud.cc
  Sphere.cc
)

target_include_directories(
  shift_core_geometry
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

find_package(assimp)

target_compile_options(
  shift_core_geometry
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
  ${ASSIMP_INCLUDE_DIRS}
)


target_link_libraries(
  shift_core_geometry
  ${SHIFT_CORE_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)
