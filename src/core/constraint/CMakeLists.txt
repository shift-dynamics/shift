add_library(
  shift_core_constraint
  Base.cc
  Chord.cc
)

target_include_directories(
  shift_core_constraint
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_constraint
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_constraint
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
