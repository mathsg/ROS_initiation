cmake_minimum_required(VERSION 2.8.3)
project(path_follower)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  simu_msgs
  roscpp
  std_msgs
  sensor_msgs
  simu_srvs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs simu_msgs geometry_msgs sensor_msgs simu_srvs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)
