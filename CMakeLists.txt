cmake_minimum_required(VERSION 3.5)

# Source path:
set(SOURCE_DIR ${CMAKE_HOME_DIRECTORY}/src)
# List of all files in src with name ending in '.cpp':
file(GLOB LIB_SOURCES ${SOURCE_DIR}/lib/*.cpp)

# TARGETS
# static library containing everything in src
#     first argument is the name of the target
#     second argument is the type of the library (STATIC of SHARED)
#     third argument is a list of source files
add_library(followmelib STATIC ${LIB_SOURCES})
target_link_libraries(followmelib m)

set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED TRUE )

project( followme )

find_package( OpenCV REQUIRED )
find_package( realsense2 REQUIRED )
find_package( PCL 1.3 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(  ${PCL_DEFINITIONS} )

add_executable( parse ${SOURCE_DIR}/main.cpp )
target_link_libraries( parse followmelib)
target_link_libraries( parse ${OpenCV_LIBS} )
target_link_libraries( parse ${realsense2_LIBRARY})
target_link_libraries( parse ${PCL_LIBRARIES} )
