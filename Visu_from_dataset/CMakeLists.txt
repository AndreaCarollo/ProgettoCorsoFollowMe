cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project( visu_test )

find_package( OpenCV REQUIRED )
find_package( realsense2 REQUIRED )
find_package( PCL 1.2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(  ${PCL_DEFINITIONS} )

add_executable( visu_test main.cpp acq_lib.cpp )

target_link_libraries( visu_test ${OpenCV_LIBS} )
target_link_libraries( visu_test ${realsense2_LIBRARY})
target_link_libraries( visu_test ${PCL_LIBRARIES} )