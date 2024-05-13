#include "mpc_cbf.h"
#include <ros/ros.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "Local_Planner_node");
    ros::NodeHandle nh;

    MPC_PLANNER mpc_planner_;
    mpc_planner_.init_MPC_CBF(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  return 0;
}