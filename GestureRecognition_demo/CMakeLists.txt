cmake_minimum_required(VERSION 3.1)
# set c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# name of project
project(Demo_GR)
# needed package
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
# select executable
add_executable(Demo_GR gesture.cpp)
# add library
# add target libraries
target_link_libraries(Demo_GR ${OpenCV_LIBS})
target_link_libraries(Demo_GR ${realsense2_LIBRARY})
