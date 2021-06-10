set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)

remove_definitions(-std=c11)
add_definitions(-march=x86-64)
add_definitions(-mtune=generic)
add_definitions(-fstack-protector-strong)
add_definitions(-D_REENTRANT)
add_definitions(-DQT_NO_DEBUG)
add_definitions(-DQT_OPENGL_LIB)
add_definitions(-DQT_WIDGETS_LIB)
add_definitions(-DQT_GUI_LIB)
add_definitions(-DQT_CORE_LIB)
add_definitions(--param=ssp-buffer-size=4)
add_definitions(-W)
add_definitions(-lstdc++)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-g3)

add_executable(
  shift
  ${CMAKE_CURRENT_LIST_DIR}/shift_gui.cpp
  ${CMAKE_CURRENT_LIST_DIR}/workspace.cpp
  ${CMAKE_CURRENT_LIST_DIR}/openglwindow.cpp
  ${CMAKE_CURRENT_LIST_DIR}/renderwindow.cpp
  ${CMAKE_CURRENT_LIST_DIR}/geometry.cpp
  ${CMAKE_CURRENT_LIST_DIR}/bodies.cpp
  ${CMAKE_CURRENT_LIST_DIR}/frames.cpp
  ${CMAKE_CURRENT_LIST_DIR}/joints.cpp
  ${CMAKE_CURRENT_LIST_DIR}/grid.cpp
  ${CMAKE_CURRENT_LIST_DIR}/lights.cpp
  ${CMAKE_CURRENT_LIST_DIR}/terrain.cpp
  ${CMAKE_CURRENT_LIST_DIR}/floor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/floorshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/floorshadowshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/flatshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/frameshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/simpleshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/phongshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/shaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/shadowshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/terrainshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/normalshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/terrainshadowshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/pointcloudshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/pointcloud.cpp
  ${CMAKE_CURRENT_LIST_DIR}/orbitalcamera.cpp
  ${CMAKE_CURRENT_LIST_DIR}/switch.cpp
  ${CMAKE_CURRENT_LIST_DIR}/playbackslider.cpp
  ${CMAKE_CURRENT_LIST_DIR}/skybox.cpp
  ${CMAKE_CURRENT_LIST_DIR}/skyboxshaderprogram.cpp
  ${CMAKE_CURRENT_LIST_DIR}/resources.qrc
)

add_dependencies(shift shiftcore)
add_dependencies(shift license about)
qt5_use_modules(shift Core Widgets OpenGL Gui Svg)

target_link_libraries(shift
  shiftcore
  stdc++
  ${GLEW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${CBLAS_LIBRARIES}
  ${LIBS}
)
