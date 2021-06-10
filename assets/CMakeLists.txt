set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Core REQUIRED)

add_subdirectory(geometry)
add_subdirectory(html)
add_subdirectory(icons)
add_subdirectory(shaders)
add_subdirectory(textures)
