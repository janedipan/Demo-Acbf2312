/* 收集实验数据并输出结果 */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "obs_manager_for_data_process/obs_manager.hpp"

std::shared_ptr<Obs_Manager> obs_Manager_;

std::vector<nav_msgs::Odometry> odom_vector_;   // 存储机器人里程
nav_msgs::Odometry odom_now_;                   // 机器人当前里程

double min_dist_to_obs_ = 1000.0;               // 障碍物最短距离
double mean_vel_;                               // 平均速度
double var_vel_;                                // 速度方差
double traj_length_;                            // 路径长度

void rcvOdomCallBack (const nav_msgs::Odometry& odom_msg) {
  odom_now_ = odom_msg;
  odom_vector_.push_back(odom_msg);

  if (odom_vector_.size() < 2) {  // 累计两个以上的odom后开始计算
    return;
  }

  // STEP 1: 计算路径长度
  traj_length_ = 0.0;
  Eigen::Vector2d last_odom(odom_vector_[0].pose.pose.position.x, 
                            odom_vector_[0].pose.pose.position.y);
  for (int i = 1; i < odom_vector_.size(); i++) {
    Eigen::Vector2d odom_i(odom_vector_[i].pose.pose.position.x, 
                           odom_vector_[i].pose.pose.position.y);
    double dis = (odom_i - last_odom).norm();
    if (dis <= 0.01) {
      dis = 0.0;
    }
    traj_length_ += dis;
    last_odom = odom_i;
  }
  std::cout << "traj_leagth := " << traj_length_ << std::endl;

  // STEP 2: 计算平均速度和速度方差
  mean_vel_ = 0.0;
  std::vector<double> vel_vector_;  // 存储机器人速度
  for (int i = 0; i < odom_vector_.size(); i++) {
    double vel_x_i = odom_vector_[i].twist.twist.linear.x;
    if (abs(vel_x_i) < 0.01) {
      vel_x_i = 0.0;
      continue;
    }
    vel_vector_.push_back(vel_x_i);
    mean_vel_ += abs(vel_x_i);
  }
  mean_vel_ = vel_vector_.size() < 1 ? mean_vel_ : mean_vel_ / vel_vector_.size();
  std::cout << "mean_vel := " << mean_vel_ << std::endl;

  var_vel_ = 0.0;
  for (int i = 0; i < vel_vector_.size(); i++) {
    var_vel_ += pow(vel_vector_[i] - mean_vel_, 2);
  }
  var_vel_ = vel_vector_.size() < 1 ? var_vel_ : var_vel_ / vel_vector_.size();
  std::cout << "var_vel := " << var_vel_ << std::endl;

  // STEP 3: 输出障碍物最近距离
  std::cout << "min_dist_to_obs := " << min_dist_to_obs_ << std::endl;
}

void stateCheckCallback (const ros::TimerEvent& e) {

  ros::Time cur_time = ros::Time::now();
  std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
  std::vector<double> radius_list;            // 获取当前时间的障碍物半径
  obs_Manager_->get_obs_state(cur_time, posVel_list, radius_list);

  Eigen::Vector2d pos_now(odom_now_.pose.pose.position.x,
                          odom_now_.pose.pose.position.y);

  double ROBOT_RADIUS = 0.50;
  for (int i = 0; i < posVel_list.size(); i++) {
    double dist_i_tmp = (pos_now - posVel_list[i].head(2)).norm() - radius_list[i] - ROBOT_RADIUS;
    min_dist_to_obs_ = std::min(min_dist_to_obs_, dist_i_tmp);
  }
}

int main (int argc, char** argv) {

	ros::init (argc, argv, "data_processor_node");
	ros::NodeHandle nh;

  // 初始化障碍物管理对象
  obs_Manager_.reset(new Obs_Manager);
  obs_Manager_->init(nh);

	ros::Subscriber odom_raw_sub  = nh.subscribe("/odometry", 1, rcvOdomCallBack);

  ros::Timer checker_timer = nh.createTimer(ros::Duration(0.05), stateCheckCallback);  // 定时碰撞检查

  ros::spin();
	return 0;
}

