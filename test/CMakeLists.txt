find_package(GTest REQUIRED)

add_executable(shift-test
  main.cc
  test_bitfield.cc
  test_excavator.cc
  # test_kdtree.cc
)

target_include_directories(shift-test PRIVATE
  ${GTEST_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(shift-test
  shift_core
  ${GTEST_BOTH_LIBRARIES}
  dl
)

# target_link_options(
#   shift-test
#   PUBLIC
#   # -Wl,-export-dynamic
# )

target_compile_options(
  shift-test
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
  -std=c++17
)

add_test(NAME shift-test
  COMMAND shift-test
)
