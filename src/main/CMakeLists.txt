add_executable(shift-cli ${CMAKE_CURRENT_SOURCE_DIR}/shift_cli.c)
target_link_libraries(shift-cli shiftcore ${LIBS})

# add_executable(7r-robot ${CMAKE_CURRENT_SOURCE_DIR}/7r_robot.c)
# target_link_libraries(7r-robot shiftcore ${LIBS})
