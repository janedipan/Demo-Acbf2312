#include "localMap_adaptor.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localMap_node");
  // ros::NodeHandle nh;
  ros::NodeHandle nh("~");


  LocalMap_manager local_map;
  local_map.initMapManager(nh, 1);
  
  ros::Duration(1.0).sleep();

  ros::spin();
  return 0;
}