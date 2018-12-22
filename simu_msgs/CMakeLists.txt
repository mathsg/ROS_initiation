cmake_minimum_required(VERSION 2.8.3)
project(simu_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  RobotPose.msg
  vectorList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs sensor_msgs
  DEPENDS
)
