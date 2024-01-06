cmake_minimum_required(VERSION 3.0.2)
project(mpc_dcbf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_planner
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_path_by_rviz src/global_path_by_rviz.cpp)

target_link_libraries(global_path_by_rviz
    ${catkin_LIBRARIES}
)

add_executable(global_path_pub src/global_path_publisher.cpp)

target_link_libraries(global_path_pub
    ${catkin_LIBRARIES}
)


