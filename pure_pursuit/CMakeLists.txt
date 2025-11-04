cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  morai_msgs
  sensor_msgs
  nav_msgs
  roslib
)

## I have custom message
## add_message_files(FILES .msg)

## codiinfo(my message) depend on
## generate_messages (DEPENDENCIES std_msgs)

## catkin package(what I refer in "find_package")
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime tf2 tf2_ros tf2_geometry_msgs morai_msgs sensor_msgs nav_msgs roslib
)

## include directories (add here when you have extra libraries)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## =======================================cpp files========================================
## codi_tf node
add_executable(steerangle src/steerangle.cpp)
target_link_libraries(steerangle ${catkin_LIBRARIES})


  