include_directories(${CMAKE_BINARY_DIR}/include)

add_executable(shift-cli shift_cli.cc)

target_link_libraries(shift-cli
  shift_core
  ksl
  ${JsonCpp_LIBRARY}
)

target_compile_options(
  shift-cli
  PRIVATE
  -march=x86-64
  -mtune=generic
  -fstack-protector-strong
  --param=ssp-buffer-size=4
  -W
  -Wall
  -Wextra
  -g3
  -fpic
  -O3
  -std=c++17
)


ExternalProject_Get_Property(argagg SOURCE_DIR)

target_include_directories(
  shift-cli
  PUBLIC
  ${CMAKE_BINARY_DIR}/include
  ${SOURCE_DIR}/include/
)

if(SHIFT_BUILD_GRAPHICS)
  find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTORCC ON)

  add_executable(shift shift.cc)

  target_link_libraries(shift Qt5::Core Qt5::Widgets Qt5::Gui)

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

  target_include_directories(
    shift
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
  )
endif()
