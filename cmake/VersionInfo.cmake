macro(shift_set_version_info)

    message(STATUS "${PROJECT_NAME} v${ARGV0}.${ARGV1}.${ARGV2}")

    set(SHIFT_VERSION_MAJOR ${ARGV0} CACHE STRING "Project major version number.")
    set(SHIFT_VERSION_MINOR ${ARGV1} CACHE STRING "Project minor version number.")
    set(SHIFT_VERSION_PATCH ${ARGV2} CACHE STRING "Project patch version number.")
    mark_as_advanced(SHIFT_VERSION_MAJOR SHIFT_VERSION_MINOR SHIFT_VERSION_PATCH)
    
    execute_process(
      COMMAND git rev-parse --abbrev-ref HEAD
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE SHIFT_GIT_BRANCH
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
    execute_process(
      COMMAND git log -1 --format=%h
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      OUTPUT_VARIABLE SHIFT_GIT_COMMIT_HASH
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    configure_file(
        "${CMAKE_SOURCE_DIR}/src/config.h.in"
        "${CMAKE_BINARY_DIR}/include/shift/config.h"
    )

endmacro()
