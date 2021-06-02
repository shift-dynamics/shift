add_library(
  shift_core_solver
  Base.cc
  RNCA.cc
)

target_include_directories(
  shift_core_solver
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_solver
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_solver
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
