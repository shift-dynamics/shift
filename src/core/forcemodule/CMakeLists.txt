add_library(
  shift_core_forcemodule
  Base.cc
  Contact.cc
  PID.cc
)

target_include_directories(
  shift_core_forcemodule
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_forcemodule
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_forcemodule
  ${SHIFT_CORE_LIBRARIES}
)
