# Install script for directory: /workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sdk_core/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample/hub/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample/lidar/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample/hub_lvx_file/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample/lidar_lvx_file/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample_cc/hub/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample_cc/lidar/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample_cc/trouble_shooting/cmake_install.cmake")
  include("/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/sample_cc/lidar_utc_sync/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/workspace/proj/dmpl_ws/src/mpl/state_estimation/livox_ros_driver/livox_ros_driver/Livox-SDK/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
