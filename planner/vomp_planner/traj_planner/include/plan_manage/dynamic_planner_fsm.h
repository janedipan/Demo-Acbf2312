
#ifndef _DYNAMIC_PLANNER_FSM_H_
#define _DYNAMIC_PLANNER_FSM_H_

#include <string.h>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>



#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32MultiArray.h>     // 发布参考路径
#include "plan_manage/plan_manager.h"

class DynamicReplanFSM 
{
private:

  int drone_id_;

  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ };

  PlanManager::Ptr planner_manager_;

  /* parameters */
  double no_replan_thresh_, replan_thresh_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double yaw_pos_;                       // 机器人当前yaw

  Eigen::Vector2d start_pt_, start_vel_, start_acc_;  // start state
  Eigen::Vector2d end_pt_, end_vel_;                  // target state

  double start_yaw_;                                  // 记录yaw规划的起点

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, cmd_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, cmdVel_sub_, swamTraj_sub_;
  ros::Publisher mpc_traj_pub;

  bool plan_success;          // 成功规划标志位
  bool use_presetGoal_;         // 是否使用指定的终点

  Eigen::Vector3d user_target;  // 指定的终点位置

  double car_l, car_h, car_w; // 小车外形尺寸

  /* helper functions */
  bool callHybridReplan();        // front-end and back-end method

  void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);

  void waypointCallback(const geometry_msgs::PoseStamped& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

  void pubRefTrajCallback(const ros::TimerEvent& e);
  void pubLMPCRefTrajCallback(const ros::TimerEvent& e);
  void pubMPCRefTrajCallback(const ros::TimerEvent& e);

  bool checkIsCollision();

public:
  DynamicReplanFSM(/* args */) {
  }
  ~DynamicReplanFSM() {
  }

  void init(ros::NodeHandle& nh);

  void init(ros::NodeHandle& nh1, ros::NodeHandle& nh2, ros::NodeHandle& nh3);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
