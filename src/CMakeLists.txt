file(GLOB_RECURSE shift_includes RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)
foreach(include ${shift_includes})
  message(STATUS ${include})
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/${include}"
    "${CMAKE_BINARY_DIR}/include/shift/${include}"
    COPYONLY
  )
endforeach()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/version.h.in"
  "${CMAKE_BINARY_DIR}/include/shift/version.h"
)

add_subdirectory(core)
add_subdirectory(gui)
add_subdirectory(main)
