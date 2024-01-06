cmake_minimum_required(VERSION 2.8.3)
project(laser_simulator)
SET(CMAKE_BUILD_TYPE Release) 
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -use_fast_math")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  DEPENDS EIGEN3
  CATKIN_DEPENDS roscpp pcl_ros
)

include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  laser_sim_node
  src/laser_sim_node.cpp
)

target_link_libraries( 
  laser_sim_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
