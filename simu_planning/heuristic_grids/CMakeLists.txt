cmake_minimum_required(VERSION 2.8.3)
project(heuristic_grids)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  simu_msgs
  simu_srvs
  cv_bridge
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs simu_msgs simu_srvs cv_bridge
)

find_package(OpenCV REQUIRED)


include_directories(
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)
