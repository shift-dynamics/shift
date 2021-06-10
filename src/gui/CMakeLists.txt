find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(shift_gui_src
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/Workspace.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/Camera.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/OrbitalCamera.cc
    # ${CMAKE_CURRENT_SOURCE_DIR}/Mesh.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/Shader.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/TriangleRenderer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/VulkanWindow.cc
)
add_library(shift_gui STATIC ${shift_gui_src})

target_link_libraries(shift_gui Qt5::Core Qt5::Widgets Qt5::Gui)

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

target_include_directories(shift_gui PUBLIC ${CMAKE_BINARY_DIR}/include)
