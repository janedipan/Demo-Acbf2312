cmake_minimum_required(VERSION 2.8.3)
project(traj_planner)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(nlopt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  message_generation
  dynamic_simulator
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_planner
 CATKIN_DEPENDS plan_env dynamic_simulator message_runtime
#  DEPENDS system_lib
)

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${nlopt_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_executable( dynamic_planner_node
    example/dynamic_planner_node.cpp
    src/theta_astar.cpp
    src/bspline_opt.cpp
    src/plan_manager.cpp
    src/dynamic_planner_fsm.cpp
)
target_link_libraries( dynamic_planner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  nlopt
)

add_executable( obs_Manager_node
    example/obs_manager_node.cpp
)
target_link_libraries( obs_Manager_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable( global_path_by_adsm
    example/global_path_by_adsm.cpp
    src/theta_astar.cpp
    src/bspline_opt.cpp
    src/plan_manager.cpp
)
target_link_libraries( global_path_by_adsm
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  nlopt
)