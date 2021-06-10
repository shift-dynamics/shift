add_library(
  shift_core_sensor
  Base.cc
  IMU.cc
  Range.cc
)

target_include_directories(
  shift_core_sensor
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_sensor
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_sensor
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
