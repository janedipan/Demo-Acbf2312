cmake_minimum_required(VERSION 2.8.12)
project(dynamic_simulator)

add_definitions(-std=c++14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_finders")

#From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo or Debug)
set(CMAKE_BUILD_TYPE Release) 

find_package(CASADI REQUIRED)
include_directories( ${CASADI_INCLUDE_DIR} )

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})


find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	rviz_visual_tools
	message_generation
)

add_message_files(
  FILES
  CoeffPoly.msg
  PieceWisePolTraj.msg
  DynTraj.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)
