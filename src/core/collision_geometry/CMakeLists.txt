add_library(
  shift_core_collision_geometry
  Base.cc
)

target_include_directories(
  shift_core_collision_geometry
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_collision_geometry
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_collision_geometry
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
