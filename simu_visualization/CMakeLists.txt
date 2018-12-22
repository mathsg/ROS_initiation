cmake_minimum_required(VERSION 2.8.3)
project(simu_visualization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  nav_msgs
  sensor_msgs
  simu_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf nav_msgs sensor_msgs simu_msgs
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(robot_visualization src/robot_visualization.cpp)
target_link_libraries(robot_visualization ${catkin_LIBRARIES})
add_dependencies(robot_visualization ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(map_visualization src/map_visualization.cpp)
target_link_libraries(map_visualization ${catkin_LIBRARIES})
add_dependencies(map_visualization ${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
