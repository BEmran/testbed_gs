cmake_minimum_required(VERSION 2.8.3)
project(testbed_gs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  message_generation
  genmsg
)

## Declare srv to be generated:
add_service_files(
  FILES
  cmd_angle.srv
  cmd_du.srv
)

## Declare msg to be generated:
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp rospy tf std_msgs geometry_msgs message_generation
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_station src/ground_station.cpp)
target_link_libraries(ground_station  ${catkin_LIBRARIES})
add_dependencies(ground_station testbed_gs_gencpp)

add_executable(testbed_tf_basic src/testbed_tf_basic.cpp)
target_link_libraries(testbed_tf_basic  ${catkin_LIBRARIES})

add_executable(testbed_tf_6dof src/testbed_tf_6dof.cpp)
target_link_libraries(testbed_tf_6dof  ${catkin_LIBRARIES})

add_executable(testbed_tf_encoder src/testbed_tf_encoder.cpp)
target_link_libraries(testbed_tf_encoder  ${catkin_LIBRARIES})

add_executable(du_cmd src/du_cmd.cpp)
target_link_libraries(du_cmd  ${catkin_LIBRARIES})
