cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project( test_realsense )

find_package( OpenCV REQUIRED )
find_package( realsense2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR} )

add_executable( test_realsense main.cpp )

target_link_libraries( test_realsense ${OpenCV_LIBS} )
target_link_libraries( test_realsense ${realsense2_LIBRARY})