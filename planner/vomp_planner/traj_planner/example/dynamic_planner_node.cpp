#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "plan_manage/dynamic_planner_fsm.h"

// Dynamic_local_planner_node 单线程节点
// int main(int argc, char** argv) {
//   ros::init(argc, argv, "Dynamic_local_planner_node");
//   ros::NodeHandle nh("~");

//   std::cout << "Dynamic_local_planner_node init-------" << std::endl;

//   // 初始化fsm对象
//   DynamicReplanFSM hybrid_replan;
//   hybrid_replan.init(nh);

//   std::cout << "Dynamic_local_planner_node init done" << std::endl;

//   ros::Duration(1.0).sleep();
//   ros::spin();

//   return 0;
// }


//当一个节点要接收和处理不同来源的数据，并且这些数据的产生频率也各不相同，
//当我们在一个回调函数里耗费太多时间时，会导致其他回调函数被阻塞，导致数据丢失。这种场合需要给一个节点开辟多个线程，保证数据流的畅通。

// Dynamic_local_planner_node 多线程节点
int main(int argc, char** argv) {
  ros::init(argc, argv, "Dynamic_local_planner_node");
  ros::NodeHandle nh1("~");
  ros::NodeHandle nh2("~");
  ros::NodeHandle nh3("~");

  std::cout << "Dynamic_local_planner_node init-------" << std::endl;

  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;

  nh1.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);
  nh3.setCallbackQueue(&custom_queue3);

  // 初始化fsm对象
  DynamicReplanFSM hybrid_replan;
  hybrid_replan.init(nh1, nh2, nh3);

  std::cout << "Dynamic_local_planner_node init done" << std::endl;

  ros::AsyncSpinner spinner1(1, &custom_queue1);  // 1 thread for the custom_queue1 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for the custom_queue2 // 0 means threads= # of CPU cores
  ros::AsyncSpinner spinner3(1, &custom_queue3);  // 1 thread for the custom_queue3 // 0 means threads= # of CPU cores

  spinner1.start();  // start spinner of the custom queue 1
  spinner2.start();  // start spinner of the custom queue 2
  spinner3.start();  // start spinner of the custom queue 3

  ros::waitForShutdown();

  return 0;
}
