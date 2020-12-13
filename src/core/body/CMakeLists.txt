add_library(
  shift_core_body
  Base.cc
  Rigid.cc
  Free.cc
  Fixed.cc
  Virtual.cc
)

target_include_directories(
  shift_core_body
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_body
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_body
  ${SHIFT_CORE_LIBRARIES}
)
