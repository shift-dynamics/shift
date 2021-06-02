add_library(
  shift_core_terrain
  Base.cc
  Basic.cc
)

target_include_directories(
  shift_core_terrain
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_terrain
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_terrain
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
