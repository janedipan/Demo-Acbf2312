#include "mpc_cbf.h"
#include <ros/ros.h>

int main(int argc, char* argv[]){
  ros::init(argc, argv, "Local_Planner_node");
  ros::NodeHandle nh("~");

  MPC_PLANNER mpc_planner_;
  mpc_planner_.init_MPC_CBF(nh);

  ros::Duration(1.0).sleep();
  while(ros::ok()){
      ros::spin();
  }
  // std::cout<<"--------------[MPC_demo]: end--------------\n";
  // int len1 = mpc_planner_.time_list1.size();
  // std::cout<< "--------------[MPC_demo]: time_size is: "<< len1 << std::endl;
  // std::cout<< "--------------[MPC_demo]: time_list is: "<< std::endl;
  // for(double t: mpc_planner_.time_list1){
  //   std::cout<< t << ",";
  // }
  // std::cout<< std::endl;
  
  // int len2 = mpc_planner_.solver.exit_obs1.size();
  // std::cout<< "--------------[MPC_demo]: exit_obs_size is: "<< len2 << std::endl;
  // std::cout<< "--------------[MPC_demo]: exit_obs_list is: "<< std::endl;
  // for(double t: mpc_planner_.solver.exit_obs1){
  //   std::cout<< t << ",";
  // }
  // std::cout<< std::endl;
  return 0;
}