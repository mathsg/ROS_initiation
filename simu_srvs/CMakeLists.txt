cmake_minimum_required(VERSION 2.8.3)
project(simu_srvs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation geometry_msgs simu_msgs sensor_msgs
)


add_service_files(
  FILES

)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  simu_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs simu_msgs sensor_msgs
  DEPENDS
)
