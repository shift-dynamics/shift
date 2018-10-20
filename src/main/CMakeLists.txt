find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(shift shift.cc)

target_link_libraries(shift assets shift_gui shift_core)

qt5_use_modules(shift Core Widgets Gui)

target_compile_options(
  shift
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

target_include_directories(shift PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(shift-cli shift_cli.cc)
target_link_libraries(shift-cli shift_core)
