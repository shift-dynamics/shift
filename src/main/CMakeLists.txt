find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(${CMAKE_SOURCE_DIR}/include/shift-gui)
include_directories(${CMAKE_SOURCE_DIR}/include/shift-core)

add_executable(shift-cli shift_cli.cc)

find_library(shift_core REQUIRED)
find_library(ksl REQUIRED)

message(STATUS "JsonCpp_LIBRARY ${JsonCpp_LIBRARY}")

target_link_libraries(shift-cli shift_core ksl ${JsonCpp_LIBRARY})

add_executable(shift shift.cc)

qt5_use_modules(shift Core Widgets Gui)

find_library(geometry REQUIRED)
find_library(html REQUIRED)
find_library(icons REQUIRED)
find_library(shaders REQUIRED)
find_library(textures REQUIRED)
set(assets geometry html icons shaders textures)

target_link_libraries(shift ${assets} shift_gui shift_core)

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
  -std=c++17
)

target_include_directories(shift PUBLIC ${CMAKE_SOURCE_DIR}/include)
