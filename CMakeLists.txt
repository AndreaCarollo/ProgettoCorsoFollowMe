cmake_minimum_required(VERSION 3.1)
## set c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

## name of project
project(People)

## needed package
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package( PCL 1.2 REQUIRED )

## include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(  ${PCL_DEFINITIONS} )

## select executable
add_executable(People peopledect.cpp)

## add library
add_library(MyLib MyLib.cpp)
# add_library(AcqLib ./AcquisitionLib/acq_lib.cpp)

## add target libraries
target_link_libraries(People ${OpenCV_LIBS})
target_link_libraries(People ${realsense2_LIBRARY})
# target_link_libraries(People ${PCL_LIBRARIES} )
target_link_libraries(People MyLib)
# target_link_libraries(People AcqLib)
