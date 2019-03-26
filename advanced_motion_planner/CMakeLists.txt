cmake_minimum_required(VERSION 2.8.3)
project(advanced_motion_planner)
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  pcl_conversions
  ackermann_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  amp
  src/advanced_motion_planner.cpp
  src/motion_computer.cpp
  src/laserscan_to_pointcloud.cpp
)

target_link_libraries(amp
  ${catkin_LIBRARIES}
)
