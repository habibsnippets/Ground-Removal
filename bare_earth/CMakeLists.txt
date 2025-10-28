cmake_minimum_required(VERSION 3.0.0)
project(ground_segmentation)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bare_earth bare_earth.cpp)
target_link_libraries(bare_earth ${PCL_LIBRARIES})