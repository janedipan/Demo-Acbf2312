#include <ros/ros.h>
#include "obs_manager/obs_manager.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "Obs_manager_node");
  ros::NodeHandle nh("~");

  std::cout << "Obs_manager_node init-------" << std::endl;

  // 初始化障碍物管理对象
  std::shared_ptr<Obs_Manager> obs_Manager_;
  obs_Manager_.reset(new Obs_Manager);
  obs_Manager_->init(nh);

  std::cout << "Obs_manager_node init done" << std::endl;

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}