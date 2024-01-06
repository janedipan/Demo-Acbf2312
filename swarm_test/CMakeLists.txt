cmake_minimum_required(VERSION 3.0.2)
project(swarm_test)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  pcl_ros
  dynamic_simulator
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES swarm_test
 CATKIN_DEPENDS roscpp rospy dynamic_simulator
 DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (odom_fixed_node src/odom_fixed.cpp )
target_link_libraries(odom_fixed_node ${catkin_LIBRARIES})

add_executable (pointcloud_fixed_node src/pointcloud_fixed.cpp )
target_link_libraries(pointcloud_fixed_node ${catkin_LIBRARIES})

add_executable (corridor_visualization_node src/corridor_visualization.cpp )
target_link_libraries(corridor_visualization_node ${catkin_LIBRARIES})

add_executable (odom_traj_visualization_node src/odom_traj_visualization.cpp )
target_link_libraries(odom_traj_visualization_node ${catkin_LIBRARIES})

add_executable (start_trigger_node src/start_trigger.cpp )
target_link_libraries(start_trigger_node ${catkin_LIBRARIES})

add_executable (data_processor_node src/data_processor.cpp )
target_link_libraries(data_processor_node ${catkin_LIBRARIES})

