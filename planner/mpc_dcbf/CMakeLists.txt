cmake_minimum_required(VERSION 3.0.2)
project(mpc_dcbf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(casadi REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
#  LIBRARIES local_planner
#  DEPENDS system_lib
)

include_directories(
  SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(global_path_by_rviz 
  src/global_path_by_rviz.cpp
)

target_link_libraries(global_path_by_rviz
    ${catkin_LIBRARIES}
)

add_executable(global_path_pub 
  src/global_path_publisher.cpp
)

target_link_libraries(global_path_pub
    ${catkin_LIBRARIES}
)

add_executable(mpc_node_c
  src/mpc_node.cpp
  src/mpc_cbf.cpp
)

target_link_libraries(mpc_node_c
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  casadi
)


