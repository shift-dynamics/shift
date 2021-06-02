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
  ${SHIFT_CORE_INCLUDE_DIRS}
  ${JsonCpp_INCLUDE_DIR}
)

target_compile_options(
  shift_core_parameter
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
)

target_link_libraries(
  shift_core_parameter
  PUBLIC
  shift_core
  ${SHIFT_CORE_LIBRARIES}
)
