add_library(
  shift_core_integrator
  Base.cc
  ODE113.cc
)

target_include_directories(
  shift_core_integrator
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_integrator
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_integrator
  ${SHIFT_CORE_LIBRARIES}
)
