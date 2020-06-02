cmake_minimum_required(VERSION 3.1)

# Source path:
set(SOURCE_DIR ${CMAKE_HOME_DIRECTORY}/src)
# List of all files in src with name ending in '.cpp':
file(GLOB LIB_SOURCES ${SOURCE_DIR}/lib/*.cpp)

## set c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


## name of project
project(DemoDect)

add_library(followmelib STATIC ${LIB_SOURCES})

target_link_libraries(followmelib m)

## needed package
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
# find_package(PCL 1.2 REQUIRED )

## include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
# include_directories( ${PCL_INCLUDE_DIRS} )
# link_directories( ${PCL_LIBRARY_DIRS} )
# add_definitions(  ${PCL_DEFINITIONS} )

## select executable
add_executable(Demo_dect ${SOURCE_DIR}/Demo_Detection.cpp)

## add target libraries
target_link_libraries(Demo_dect followmelib)
target_link_libraries(Demo_dect ${OpenCV_LIBS})
