file(GLOB_RECURSE
    shift_includes RELATIVE
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

foreach(include ${shift_includes})
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${include}"
    "${CMAKE_BINARY_DIR}/include/shift/${include}"
    COPYONLY
  )
endforeach()

add_subdirectory(core)

if(SHIFT_BUILD_GRAPHICS)
add_subdirectory(gui)
endif()

add_subdirectory(main)
