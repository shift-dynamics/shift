add_library(
  shift_core_sensor
  base.cc
  imu.cc
  range.cc
)

target_include_directories(
  shift_core_sensor
  PUBLIC
  ${CMAKE_BINARY_DIR}/include
  ${JSONCPP_INCLUDE_DIR}
)

target_compile_options(
  shift_core_sensor
  PRIVATE
  -march=x86-64
  -mtune=generic
  -fstack-protector-strong
  --param=ssp-buffer-size=4
  -W
  -lstdc++
  -Wall
  -Wextra
  -g3
  # -O3
  -std=c++17
  ${GRAPHVIZ_COMPILE_FLAGS}
  -DSHIFT_VERIFY_EQUATIONS_=1
)

target_link_libraries(
  shift_core_sensor
  shift_core
  ${JsonCpp_LIBRARY}
  ${GRAPHVIZ_GVC_LIBRARY}
  ${GRAPHVIZ_CGRAPH_LIBRARY}
  ${CMAKE_THREAD_LIBS_INIT}
  ${CBLAS_LIBRARIES}
  ksl
)
