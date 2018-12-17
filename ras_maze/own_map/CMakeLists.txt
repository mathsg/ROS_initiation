cmake_minimum_required(VERSION 2.8.3)
project(own_map)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs kdl_conversions simu_msgs geometry_msgs sensor_msgs simu_srvs)
find_package(Boost REQUIRED COMPONENTS random)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs simu_msgs geometry_msgs sensor_msgs simu_srvs
  INCLUDE_DIRS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
