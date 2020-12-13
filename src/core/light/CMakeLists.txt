add_library(
  shift_core_light
  Base.cc
  Directional.cc
  IES.cc
  Point.cc
  Spot.cc
)

target_include_directories(
  shift_core_light
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_light
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_light
  ${SHIFT_CORE_LIBRARIES}
)
