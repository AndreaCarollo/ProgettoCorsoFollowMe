cmake_minimum_required(VERSION 3.1)
# set c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# name of project
project(People)
# needed package
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
# select executable
add_executable(People peopledect.cpp)
add_library(MyLib MyLib.cpp)
# add library
# add target libraries
target_link_libraries(People ${OpenCV_LIBS})
target_link_libraries(People ${realsense2_LIBRARY})
target_link_libraries(People MyLib)
