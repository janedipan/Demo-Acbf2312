// 感知测试节点

#include "dynamic_perception/obstacle_prediction.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_prediction_node");
  ros::NodeHandle nh("~");

  std::cout << "obstacle_prediction node init" << std::endl;

  Obstalce_prediction test(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}