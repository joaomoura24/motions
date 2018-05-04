cmake_minimum_required(VERSION 2.8.3)

project(motions)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roslib
  roscpp
  kdl_parser
  serial_arm_lib
  hardware
)

#find_package(orocos_kdl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES paramProj spiral raster
  #CATKIN_DEPENDS roscpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# Define a library target called raster
add_library(raster src/raster.cpp)
target_link_libraries(raster ${catkin_LIBRARIES})

# Define a library target called spiral
add_library(spiral src/spiral.cpp)
target_link_libraries(spiral ${catkin_LIBRARIES})

# Define a library target called paramProj
add_library(paramProj src/paramProj.cpp)
target_link_libraries(paramProj ${catkin_LIBRARIES})

# Add executable target motions:
add_executable(motions src/motions.cpp)
target_link_libraries(motions ${catkin_LIBRARIES} spiral raster)
