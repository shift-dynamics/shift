add_library(
  shift_core_collisiongeometry
  Base.cc
)

target_include_directories(
  shift_core_collisiongeometry
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_collisiongeometry
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_collisiongeometry
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
