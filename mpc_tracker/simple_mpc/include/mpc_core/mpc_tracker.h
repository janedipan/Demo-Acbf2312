#ifndef _MPC_TRACKER_H_
#define _MPC_TRACKER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>

#include "mpc_core/mpc_base.h"
#include "mpc_core/mpc_ackman.hpp"
#include "mpc_core/mpc_differential.hpp"
#include "mpc_core/mpc_quadruped.hpp"

class mpc_tracker
{
private:
  int mpc_type_;                              // MPC运动学类型：阿克曼, 差速, 四足
  enum MPC_TYPE { Ackman_MPC, 
                  Differential_MPC,
                  Quadruped_MPC};             // MPC运动学类型枚举

  std::shared_ptr<MPC_base> mpc_solver_;      // MPC父类指针

  // MPC可调参数
  double dT_ = 0.10;                          // 离散时间
  int N_ = 10;                                // 预测步长，需与参考轨迹点数目一致
  double L_ = 0.608;                          // 小车前后轮轴距, 单位m
  std::vector<double> Q = {10, 10, 2.5};      // 小Q矩阵，反映X, Y, Theta的误差权重
  std::vector<double> R = {0.01, 0.01};       // 小R矩阵，反映V，Delta的控制权重

  // MPC约束量
  double max_delta = 0.6;                     // 最大前轮转角(rad)
  double max_speed = 1.0;                     // 最大速度(m/s)
  double min_speed = -1.0;                    // 最小速度(m/s)
  double max_speed_y = 0.5;                   // 最大y轴速度(m/s)

  // 标志位
  bool has_odom = false;                      // 接收到odom标志位
  bool receive_traj_ = false;                 // 接收到参考轨迹标志位

  // MPC输入: 当前状态和参考轨迹
  Eigen::Vector3d current_state;              // 存放当前状态
  Eigen::MatrixXd desired_state;              // 存放参考轨迹

  // ros 接口
	ros::NodeHandle nh_;                        // 保存ros句柄
  ros::Timer cmd_timer_;                      // MPC求解定时器
  ros::Publisher cmd_twist_pub_, predict_pub; // ROS发布者，分别为控制指令发布和预测轨迹发布
  ros::Subscriber odom_sub_, traj_sub_;       // ROS订阅者，分别为里程计接收和参考轨迹接收

  geometry_msgs::Twist cmd_vel;               // 输出cmd_vel消息给小车
  std_msgs::Float32MultiArray ref_traj_msg;   // 接收的参考轨迹消息

  // ROS回调
  void cmdCallback(const ros::TimerEvent &e);                 // 定时器回调函数
  void rcvOdomCallBack(nav_msgs::OdometryPtr msg);            // 里程计接收回调
  void rcvTrajCallBack(std_msgs::Float32MultiArray msg);      // 参考轨迹接收回调
  void drawPredictTraj(std::vector<double>& predict_states);  // 发布预测轨迹topic的函数
  void smooth_yaw(Eigen::MatrixXd& ref_traj);                 // yaw角平滑

public:
  mpc_tracker(){};                        // 构造函数
  ~mpc_tracker(){};                       // 析构函数
  void initMPC(ros::NodeHandle& nh);      // mpc_tracker初始化
};


#endif
