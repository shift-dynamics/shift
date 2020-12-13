add_library(
  shift_core_joint
  Base.cc
  Float.cc
  Prismatic.cc
  Revolute.cc
  Rigid.cc
  Spherical.cc
)

target_include_directories(
  shift_core_joint
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_joint
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_joint
  ${SHIFT_CORE_LIBRARIES}
)
