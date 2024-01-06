// 障碍物管理测试节点

#include "dynamic_perception/obs_manager/obs_manager.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_prediction_node");
  ros::NodeHandle nh("~");

  std::cout << "obs_manager node init" << std::endl;

  Obs_Manager test;
  test.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}