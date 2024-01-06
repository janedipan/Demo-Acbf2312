cmake_minimum_required(VERSION 3.10.0)
project(mapping)

find_package(catkin REQUIRED COMPONENTS
  # message_generation
  std_msgs
  roscpp
  rospy
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  # airsim_ros_pkgs
)
message("==============================================")
message(${catkin_INCLUDE_DIRS})
# add_message_files(
#   FILES
#   GimbalAngleEulerCmd.msg
#   GimbalAngleQuatCmd.msg
# )

# add_service_files(
#   FILES
#   SetGPSPosition.srv
#   Takeoff.srv
#   TakeoffGroup.srv
#   Land.srv
#   LandGroup.srv
#   Reset.srv
#   SetLocalPosition.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   nav_msgs
# )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapping
  # CATKIN_DEPENDS message_generation   
  CATKIN_DEPENDS std_msgs roscpp rospy sensor_msgs nav_msgs
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( mapping 
    src/mapping.cpp 
    src/raycast.cpp
    src/localMap_adaptor.cpp
)

target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(localMap_node
  src/localMap_node.cpp
  src/localMap_adaptor.cpp
  src/mapping.cpp
  src/raycast.cpp
)

target_link_libraries(localMap_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  mapping
)