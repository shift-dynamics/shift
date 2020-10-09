find_package(GTest REQUIRED)

add_executable(shift-test
  main.cc
  test_bitfield.cc
  # test_kdtree.cc
)

target_include_directories(shift-test PRIVATE
  ${GTEST_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/external/ksl/include
  ${CMAKE_BINARY_DIR}/include

)

target_link_libraries(shift-test
    shift_core
    ${GTEST_BOTH_LIBRARIES}
)

add_test(NAME shift-test
  COMMAND shift-test
)
