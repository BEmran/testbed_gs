cmake_minimum_required(VERSION 2.8.3)
project(testbed_gs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)

## Specify additional locations of header files
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

add_executable(testbed_tf_basic src/testbed_tf_basic.cpp)
target_link_libraries(testbed_tf_basic  ${catkin_LIBRARIES})
