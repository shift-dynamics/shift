add_library(
  shift_core_behavior
  Base.cc
  ContactDetection.cc
  IK7DOF.cc
)

target_include_directories(
  shift_core_behavior
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_behavior
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_behavior
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
