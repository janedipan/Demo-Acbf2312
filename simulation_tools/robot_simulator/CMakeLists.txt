cmake_minimum_required(VERSION 2.8.3)
project(robot_simulator)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (scout_simulator_node src/scout_simulator.cpp )
target_link_libraries(scout_simulator_node ${catkin_LIBRARIES})

add_executable (hunter_simulator_node src/hunter_simulator.cpp )
target_link_libraries(hunter_simulator_node ${catkin_LIBRARIES})

add_executable (scout_visualizer_node src/scout_visualizer.cpp )
target_link_libraries(scout_visualizer_node ${catkin_LIBRARIES})

add_executable (hunter_visualizer_node src/hunter_visualizer.cpp )
target_link_libraries(hunter_visualizer_node ${catkin_LIBRARIES})