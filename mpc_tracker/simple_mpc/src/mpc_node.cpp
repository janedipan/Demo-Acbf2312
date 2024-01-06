#include "mpc_core/mpc_tracker.h"
#include <ros/ros.h>

int main( int argc, char * argv[] ) {
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh;

  mpc_tracker mpc_tracker_;
  mpc_tracker_.initMPC(nh);
  
  ros::Duration(1.0).sleep();

  ros::spin();

  return 0;
}