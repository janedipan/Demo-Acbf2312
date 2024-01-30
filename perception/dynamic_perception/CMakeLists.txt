cmake_minimum_required(VERSION 3.0.2)
project(dynamic_perception)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
  autoware_msgs
  # obsdet_msgs
  dynamic_simulator
  message_generation
  message_runtime
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8.1 REQUIRED)
find_package(CASADI REQUIRED)
include_directories( ${CASADI_INCLUDE_DIR} )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

generate_dynamic_reconfigure_options(
  cfg/ri_dbscan.cfg
)

add_message_files(
  FILES
  CloudCluster.msg
  CloudClusterArray.msg
  DetectedObject.msg
  DetectedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  jsk_recognition_msgs
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ri_dbscan
  CATKIN_DEPENDS roscpp std_msgs pcl_ros 
  geometry_msgs
  jsk_recognition_msgs
  message_runtime
  sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIR}
)

link_directories(${OpenCV_LIBRARY_DIRS})


add_executable(ri_dbscan_node
  src/ri_dbscan.cpp
)

add_dependencies(ri_dbscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ri_dbscan_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_executable(l_shape_fitting
  src/lshape_estimator_node.cpp
  src/lshape_estimator.cpp
)

add_dependencies(l_shape_fitting
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(l_shape_fitting
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(obstacle_prediction_node
    src/obstacle_prediction_node.cpp
    src/obstacle_prediction.cpp
    src/Hungarian.cpp)
add_dependencies(obstacle_prediction_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(obstacle_prediction_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_INCLUDE_DIR} casadi)


add_executable(obs_manager_node
  src/obs_manager_node.cpp
)

add_dependencies(obs_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(obs_manager_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
