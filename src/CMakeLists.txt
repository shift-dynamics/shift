find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(vulkan_window_src
  workspace.cc
  vulkanwindow.cc
  mainwindow.ui
)

add_library(vulkanwindow ${vulkan_window_src})

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
  -std=c++11
)

target_include_directories(vulkanwindow PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(shift-vulkan main.cc)

target_link_libraries(shift-vulkan vulkanwindow assets)

qt5_use_modules(shift-vulkan Core Widgets Gui)

target_compile_options(
  shift-vulkan
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

target_include_directories(shift-vulkan PUBLIC ${CMAKE_SOURCE_DIR}/include)
