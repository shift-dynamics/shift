find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(shift_gui_src
  workspace.cc
  workspace.ui
  vulkanwindow.cc
)

include_directories(${CMAKE_SOURCE_DIR}/include/shift-gui)

add_library(shift_gui ${shift_gui_src})

qt5_use_modules(shift_gui Core Widgets Gui)

if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  target_compile_definitions(
    shift_gui
    PRIVATE
    QT_NO_DEBUG
  )
endif()

target_compile_options(
  shift_gui
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
  -O3
  -std=c++11
)

target_include_directories(shift_gui PUBLIC ${CMAKE_SOURCE_DIR}/include)
