cmake_minimum_required(VERSION 3.1.0)

project(vulkanwindow)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(vulkan_window_src
  src/vulkanwindow.cc
  src/main.cc
)

add_executable(vulkanwindow ${vulkan_window_src})

qt5_use_modules(vulkanwindow Core Widgets Gui)

target_compile_definitions(
  vulkanwindow
  PRIVATE
  QT_NO_DEBUG
)

target_compile_options(
  vulkanwindow
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
)

