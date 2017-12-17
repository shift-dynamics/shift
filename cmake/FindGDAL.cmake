find_path(GDAL_INCLUDE_DIR gdal.h
  HINTS
    ENV GDAL_DIR
    ENV GDAL_ROOT
  PATH_SUFFIXES
     include/gdal
     include/GDAL
)

if(UNIX)
  find_program(GDAL_CONFIG gdal-config
      HINTS
        ENV GDAL_DIR
        ENV GDAL_ROOT
  )

  if(GDAL_CONFIG)
    exec_program(
      ${GDAL_CONFIG} ARGS --libs
    OUTPUT_VARIABLE GDAL_LIBRARY_LA_FILE)

    # read libtools generate la meta data file to find location of shared libraries
    if(EXISTS GDAL_LIBRARY_LA_FILE)
      FILE(READ ${GDAL_LIBRARY_LA_FILE} GDAL_LIBRARY_LA_CONTENTS)
      STRING(
        REGEX REPLACE ";" "\\\\;"
        GDAL_LIBRARY_LA_CONTENTS
        ${GDAL_LIBRARY_LA_CONTENTS})
      STRING(REGEX REPLACE "\n" ";"
        GDAL_LIBRARY_LA_CONTENTS
        ${GDAL_LIBRARY_LA_CONTENTS})
      foreach(LINE ${GDAL_LIBRARY_LA_CONTENTS})
        string(REGEX MATCHALL "dlname=[^ ]+" DLNAME ${LINE})
        if(DLNAME)
          string(REPLACE "dlname=" "" DLNAME ${DLNAME})
          string(REPLACE "'" "" DLNAME ${DLNAME})
          message(STATUS "DLNAME: " ${DLNAME})
        endif()
      endforeach()
      string(REGEX MATCH "(([^\\/]*\\/)*)" GDAL_LIBRARY_PATH ${GDAL_LIBRARY_LA_FILE})
      # the code below reads the autotools --cflags variable to find all include
      # directories for gdal, which requires that gdal already be configured
      # and built. explicitly adding output of this command below
      # exec_program(
      #   ${GDAL_CONFIG} ARGS --cflags
      #   OUTPUT_VARIABLE GDAL_CFLAGS)
      # if(GDAL_CFLAGS)
      #   string(REGEX MATCHALL "-I[^ ]+" _gdal_dashI ${GDAL_CFLAGS})
      #   string(REPLACE "-I" ";" GDAL_INCLUDE_DIR ${_gdal_dashI})
      # endif()
    else()
      set(GDAL_LIBRARY_PATH ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/.libs)
    endif()
  endif()
endif()

find_library(GDAL_LIBRARY
  NAMES gdal
  PATHS ${GDAL_LIBRARY_PATH})

message(STATUS "GDAL_CFLAGS: " ${GDAL_CFLAGS})
message(STATUS "GDAL_LIBRARY_PATH: " ${GDAL_LIBRARY_PATH})
message(STATUS "GDAL_LIBRARY: " ${GDAL_LIBRARY})


set(GDAL_INCLUDE_DIR
  ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/port                  ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/gcore                 ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/alg                   ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/ogr                   ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/ogr/ogrsf_frmts       ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/frmts/vrt
  ${CMAKE_SOURCE_DIR}/src/third_party/gdal/gdal/apps)

foreach(INCLUDE_DIR ${GDAL_INCLUDE_DIR})
  message(STATUS ${INCLUDE_DIR})
endforeach()

# message(STATUS "GDAL_INCLUDE_DIR: " ${GDAL_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  GDAL
  DEFAULT_MSG
  GDAL_LIBRARY
  GDAL_INCLUDE_DIR)

set(GDAL_LIBRARIES ${GDAL_LIBRARY})
set(GDAL_INCLUDE_DIRS ${GDAL_INCLUDE_DIR})
