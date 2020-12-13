add_library(
  shift_core_parameter
  Value.cc
  FactoryArray.cc
  List.cc
  Array.cc
  ArrayBase.cc
)

target_include_directories(
  shift_core_parameter
  PUBLIC
  ${SHIFT_CORE_INDLUDE_DIRS}
)

target_compile_options(
  shift_core_parameter
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_parameter
  ${SHIFT_CORE_LIBRARIES}
)
