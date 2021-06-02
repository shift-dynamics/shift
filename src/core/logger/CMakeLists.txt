set(HDF5_PREFER_PARALLEL true)
find_package(HDF5 COMPONENTS CXX HL REQUIRED)

add_library(
  shift_core_logger
  Base.cc
  CSV.cc
  HDF5.cc
)

target_include_directories(
  shift_core_logger
  PUBLIC
  ${SHIFT_CORE_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${HDF5_CXX_INCLUDE_DIRS}
)

target_compile_options(
  shift_core_logger
  PRIVATE
  ${SHIFT_CORE_COMPILE_FLAGS}
  ${HDF5_DEFINITIONS}
  ${HDF5_CXX_DEFINITIONS}
  -O
)

target_link_libraries(
  shift_core_logger
  shift_core
  ${SHIFT_CORE_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  ${HDF5_CXX_HL_LIBRARIES}
)
