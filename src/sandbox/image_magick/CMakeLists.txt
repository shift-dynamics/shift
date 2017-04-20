cmake_minimum_required(VERSION 2.8.11)
project(height_map_generator)


find_package(ImageMagick REQUIRED)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
add_executable(height_map_gen height_map_gen.cpp)
target_link_libraries(height_map_gen ${ImageMagick_LIBRARIES} ${ImageMagick_Magick++_LIBRARIES})
include_directories(${ImageMagick_INCLUDE_DIRS} ${ImageMagick_Magick++_INCLUDE_DIRS})
add_definitions(-DMAGICKCORE_HDRI_ENABLE=1)
add_definitions(-DMAGICKCORE_QUANTUM_DEPTH=16)
