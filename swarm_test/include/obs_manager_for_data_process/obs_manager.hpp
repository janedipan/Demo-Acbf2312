#pragma once
#ifndef _OBS_MANAGER_H
#define _OBS_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <unordered_map>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_converter/ObstacleArrayMsg.h>   // TEB预测轨迹消息类型
#include <std_msgs/Float32MultiArray.h>           // 清华DCBF预测轨迹消息类型
#include "dynamic_simulator/DynTraj.h"            // 动态障碍物预测轨迹消息类型

// 障碍物轨迹类型结构体
struct obstacle_traj      
{
  int Id_;                  // 障碍物id号
  ros::Time start_time;     // 轨迹开始时间
  Eigen::Vector2d bbox_;    // 包围盒
  double circle_R_;         // 障碍物圆半径

  /* 轨迹真值参数: 三叶草轨迹 */
  Eigen::Vector2d scale_;   // 轨迹参数
  Eigen::Vector2d pos_xy_;  // 轨迹参数
  double slower_;           // 轨迹参数
  double offset_;           // 轨迹参数

  /* 预测轨迹参数: 二次多项式 */
  double Time0;              // 预测轨迹开始时间
  double Time1;              // 预测轨迹结束时间
  Eigen::Vector3d coeff_x;   // 二次多项式系数x
  Eigen::Vector3d coeff_y;   // 二次多项式系数y
};


class Obs_Manager
{
public:
  Obs_Manager(){};    // 空的构造和析构
  ~Obs_Manager(){};

  void init(ros::NodeHandle &nh)   // Obs_Manager初始化
  {
    nh.param("obs_manager/use_GroundTruth", is_use_GroundTruth, true);

    // 使用障碍物轨迹真值或障碍物预测轨迹
    if (is_use_GroundTruth) {
      obsTraj_sub = nh.subscribe("/trajs", 100, &Obs_Manager::obsTrajCallback, this);
    } else {
      predicted_Traj_sub = nh.subscribe("/trajs_predicted", 100, &Obs_Manager::predict_Traj_Callback, this);
    }

    obsTraj_pub = nh.advertise<visualization_msgs::MarkerArray>("obs_traj_vis", 1, true);

    dcbfTraj_pub = nh.advertise<std_msgs::Float32MultiArray>("obs_predict_pub", 100, true);

    tebTraj_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("move_base/TebLocalPlannerROS/obstacles", 100, true);

    // show_timer = nh.createTimer(ros::Duration(0.10), &Obs_Manager::showObs_callback, this);

    std::cout << "ObsManager init done !!!" << std::endl;
  }

  // 根据时间返回障碍物的状态: posVel_list: P_x, P_y, V_x, V_y;   radius_lists: 对应的障碍物半径
  void get_obs_state(ros::Time cur_time, std::vector<Eigen::Vector4d>& posVel_list, std::vector<double>& radius_lists)
  {
    if (is_use_GroundTruth)
    {
      // 遍历轨迹容器，根据时间返回可用的障碍物位置
      for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

        double time_out = (cur_time - traj_iter->second.start_time).toSec();

        if(time_out >= 5.0 - 1e-3) {  // 超时的无用障碍物轨迹，删除并跳过
          continue;
        }

        double rel_time = cur_time.toSec() - traj_iter->second.Time0;

        // 根据轨迹参数，计算当前时间的障碍物位置和速度状态
        Eigen::Vector2d tmp_X;
        tmp_X[0] = 1.0 * rel_time / traj_iter->second.slower_ + traj_iter->second.offset_;
        tmp_X[1] = 2.0 * rel_time / traj_iter->second.slower_ + traj_iter->second.offset_;
        Eigen::Vector4d cur_posVel;
        cur_posVel[0] = traj_iter->second.scale_.x() * (sin(tmp_X[0]) + 2.0 * sin(tmp_X[1])) + traj_iter->second.pos_xy_.x();
        cur_posVel[1] = traj_iter->second.scale_.y() * (cos(tmp_X[0]) - 2.0 * cos(tmp_X[1])) + traj_iter->second.pos_xy_.y();
        cur_posVel[2] = traj_iter->second.scale_.x() * (cos(tmp_X[0]) + 4.0 * cos(tmp_X[1])) / traj_iter->second.slower_;
        cur_posVel[3] = traj_iter->second.scale_.y() * (-1.0 * sin(tmp_X[0]) + 4.0 * sin(tmp_X[1])) / traj_iter->second.slower_;
        posVel_list.push_back(cur_posVel);
        radius_lists.push_back(traj_iter->second.circle_R_);
      }
    }
    else
    {
      // 遍历轨迹容器，根据时间返回可用的障碍物位置
      for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

        if ( cur_time.toSec() > traj_iter->second.Time1 - 0.05 ) {  // 超时的无用障碍物轨迹，删除并跳过
          continue;
        }

        Eigen::Vector4d cur_posVel;
        double time_now = cur_time.toSec() - traj_iter->second.Time0;
        cur_posVel[0] = traj_iter->second.coeff_x[0] * pow(time_now/5.0, 2) +
                        traj_iter->second.coeff_x[1] * time_now/5.0 + traj_iter->second.coeff_x[2];
        cur_posVel[1] = traj_iter->second.coeff_y[0] * pow(time_now/5.0, 2) +
                        traj_iter->second.coeff_y[1] * time_now/5.0 + traj_iter->second.coeff_y[2];
        cur_posVel[2] = 2.0 * traj_iter->second.coeff_x[0] * time_now / 25.0 + traj_iter->second.coeff_x[1];
        cur_posVel[3] = 2.0 * traj_iter->second.coeff_y[0] * time_now / 25.0 + traj_iter->second.coeff_y[1];
        posVel_list.push_back(cur_posVel);
        radius_lists.push_back(traj_iter->second.circle_R_);
      }
    }
  }

  int get_obs_size() {

    ros::Time cur_time = ros::Time::now();

    int size_of_obs = 0;

    if (is_use_GroundTruth)
    {
      // 遍历轨迹容器，根据时间返回可用的障碍物位置
      for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

        double time_out = (cur_time - traj_iter->second.start_time).toSec();

        if(time_out >= 5.0 - 1e-3) {  // 超时的无用障碍物轨迹，删除并跳过
          continue;
        }
        size_of_obs += 1;
      }
    }
    else
    {
      // 遍历轨迹容器，根据时间返回可用的障碍物位置
      for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

        if ( cur_time.toSec() > traj_iter->second.Time1 - 0.05 ) {  // 超时的无用障碍物轨迹，删除并跳过
          continue;
        }
        size_of_obs += 1;
      }
    }
    return size_of_obs;
  }

  bool is_collide(Eigen::Vector2d pos, double robot_R, ros::Time cur_time) // 根据当前位置和时间，返回碰撞状态
  {
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径
    get_obs_state(cur_time, posVel_list, radius_list);

    for (int i = 0; i < posVel_list.size(); i++) {  // 遍历障碍物状态容器，查询是否碰撞
      double distance = (pos - posVel_list[i].head(2)).norm();

      if(distance <= radius_list[i] + robot_R) {  // (当前位置与障碍物中心距离) 小于 (障碍物半径 + 机器人半径)
        return true;                              // 返回碰撞!
      }
    }
    return false;
  }

  bool is_VO_unsafe(Eigen::Vector4d posVel, double robot_R, ros::Time cur_time) // 根据当前位置和时间，返回VO安全状态
  {
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径
    get_obs_state(cur_time, posVel_list, radius_list);

    for (int i = 0; i < posVel_list.size(); i++) {  // 遍历障碍物状态容器，查询VO状态
      Eigen::Vector4d rel_dis, rel_vel;
      double radius_vo = radius_list[i] + robot_R;
      rel_dis = posVel.head(2) - posVel_list[i].head(2);
      rel_vel = posVel.tail(2) - posVel_list[i].tail(2);

      Eigen::MatrixXd rjt_rj = rel_dis * rel_dis.transpose();       //计算VO代价的中间变量
      Eigen::MatrixXd rjt_vj = rel_dis * rel_vel.transpose();       //计算VO代价的中间变量
      Eigen::MatrixXd vjt_vj = rel_vel * rel_vel.transpose();       //计算VO代价的中间变量

      double dis_x_vel = rel_dis.transpose() * rel_vel; // dis_x_vel的正负就表示了角度是锐角还是钝角
      double vo = pow(dis_x_vel, 2) / rel_vel.squaredNorm() - rel_dis.squaredNorm() + pow(radius_vo, 2);

      double constant_time = 2.0;   // 允许的最小碰撞时3
      double t_collide = (rjt_rj.norm() - rel_dis.norm() * robot_R) / rjt_vj.norm();   // 碰撞时间： (距离-半径)/速度投影

      if (rel_dis.norm() <= radius_vo) {   // 是否碰撞
        return true;
      }
      if (vo > 0.0 && dis_x_vel < 0.0 && t_collide < constant_time) {  // 是否违反VO
        return true;
      }
    }
    return false;
  }

private:

  bool is_use_GroundTruth;

  ros::Subscriber obsTraj_sub, predicted_Traj_sub;

  ros::Publisher obsTraj_pub, dcbfTraj_pub, tebTraj_pub;

  ros::Timer show_timer;

  std::unordered_map<int, obstacle_traj> obstalce_trajs_;         // 所有障碍物轨迹，哈希表形式存储

  void showObs_callback(const ros::TimerEvent& e)         // 定时器回调，定时更新障碍物显示
  {
    show_Obs_traj();
  }

  void show_Obs_traj() {
    if(obstalce_trajs_.size() == 0) return; // 没有障碍物轨迹，返回

    visualization_msgs::MarkerArray obs_balls_msg;
    visualization_msgs::Marker obs_ball;

    ros::Time time_now = ros::Time::now();

    obs_ball.header.frame_id = "world";
    obs_ball.header.stamp = time_now;
    obs_ball.type = visualization_msgs::Marker::SPHERE_LIST;  // CUBE:立方体, Sphere:球体, Cylinder:圆柱体
    obs_ball.action = visualization_msgs::Marker::ADD;
    obs_ball.id = 0;
    obs_ball.lifetime = ros::Duration(0.20);
    obs_ball.color.a = 0.50f;
    obs_ball.color.r = 0.00f;
    obs_ball.color.g = 0.50f;
    obs_ball.color.b = 0.80f;
    obs_ball.scale.x = 0.5;
    obs_ball.scale.y = 0.5;
    obs_ball.scale.z = 0.5;
    obs_ball.pose.orientation.w = 1.0;

    for (double add_time = 0.0; add_time < 2.0; add_time += 0.1) {
      ros::Time time_index = time_now + ros::Duration(add_time);

      std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
      std::vector<double> radius_list;            // 获取当前时间的障碍物半径

      get_obs_state(time_index, posVel_list, radius_list);

      for (int i = 0; i < posVel_list.size(); i++) {
        geometry_msgs::Point p;
        p.x = posVel_list[i].x();
        p.y = posVel_list[i].y();
        p.z = 0.25;
        obs_ball.points.push_back(p);
      }
      obs_balls_msg.markers.push_back(obs_ball);
    }
    obsTraj_pub.publish(obs_balls_msg);
  }

  void pub_DCBF_traj() {
    if(obstalce_trajs_.size() == 0) return; // 没有障碍物轨迹，返回

    std_msgs::Float32MultiArray dcbf_msgs;
    ros::Time time_now = ros::Time::now();

    int N_ = 25;                        // mpc预测步长，需与清华mpc-dcbf参数保持一致
    double delta_t_ = 0.10;             // mpc离散时间，需与清华mpc-dcbf参数保持一致

    dcbf_msgs.data.resize(5 * N_ * get_obs_size()); // 给障碍物矩阵分配内存

    for (int i = 0; i < N_; i++) {                // 预测第i步
      double add_time = (double)(i * delta_t_);
      ros::Time time_index = time_now + ros::Duration(add_time);
      std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
      std::vector<double> radius_list;            // 获取当前时间的障碍物半径

      get_obs_state(time_index, posVel_list, radius_list);
      for (int j = 0; j < posVel_list.size(); j++) {  // 对于第j个障碍物
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 0 ] = posVel_list[j].x();  // x坐标
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 1 ] = posVel_list[j].y();  // y坐标
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 2 ] = radius_list[j];      // 椭圆半长轴a
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 3 ] = radius_list[j];      // 椭圆半短轴b
        dcbf_msgs.data[ 5 * N_ * j + 5 * i + 4 ] = 0.0;                 // 椭圆方位角theta
      }
    }
    dcbfTraj_pub.publish(dcbf_msgs);
  }

  void pub_TEB_traj() {
    if(obstalce_trajs_.size() == 0) return; // 没有障碍物轨迹，返回

    ros::Time time_index = ros::Time::now();
    std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
    std::vector<double> radius_list;            // 获取当前时间的障碍物半径

    get_obs_state(time_index, posVel_list, radius_list);

    costmap_converter::ObstacleArrayMsg obs_Array;
    obs_Array.header.frame_id = "world";
    obs_Array.header.stamp = time_index;

    for (int i = 0; i < posVel_list.size(); i++) {
      costmap_converter::ObstacleMsg obs_tmp;
      obs_tmp.header.frame_id = "world";
      obs_tmp.header.stamp = time_index;
      obs_tmp.id = i;
      obs_tmp.polygon.points.resize(1);
      obs_tmp.polygon.points[0].x = posVel_list[i].x();
      obs_tmp.polygon.points[0].y = posVel_list[i].y();
      obs_tmp.polygon.points[0].z = 0.0;
      obs_tmp.radius = radius_list[i];

      double yaw = atan2(posVel_list[i][3], posVel_list[i][2]);
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, yaw);
      tf::quaternionTFToMsg(quat, obs_tmp.orientation);

      obs_tmp.velocities.twist.linear.x = posVel_list[i][2];
      obs_tmp.velocities.twist.linear.y = posVel_list[i][3];
      obs_tmp.velocities.twist.linear.z = 0.0;
      obs_tmp.velocities.twist.angular.x = 0.0;
      obs_tmp.velocities.twist.angular.y = 0.0;
      obs_tmp.velocities.twist.angular.z = 0.0;

      obs_Array.obstacles.push_back(obs_tmp);
    }
    tebTraj_pub.publish(obs_Array);
  }

  void obsTrajCallback(const dynamic_simulator::DynTraj& msg)     // 接收障碍物真值轨迹的回调函数
  {
    obstacle_traj tmp_obs;    // 将障碍物消息转为vo_obstacle格式
    tmp_obs.Id_ = msg.id;
    tmp_obs.start_time = msg.header.stamp;
    tmp_obs.Time0 = msg.start_time.stamp.toSec();

    tmp_obs.scale_   << msg.s_num[0], msg.s_num[1];
    tmp_obs.pos_xy_  << msg.s_num[3], msg.s_num[4];
    tmp_obs.slower_ = msg.s_num[6];
    tmp_obs.offset_ = msg.s_num[7];
    tmp_obs.bbox_ << msg.bbox[0], msg.bbox[1];

    Eigen::Vector2d bbox_half = tmp_obs.bbox_ / 2.0;  // 取bbox外接圆半径
    tmp_obs.circle_R_ = bbox_half.norm();

    // 更新障碍物轨迹容器
    auto find_ptr = obstalce_trajs_.find(tmp_obs.Id_);
    if (find_ptr != obstalce_trajs_.end()) {  // 如果障碍物之前已存在，替换哈希表中对应值
      find_ptr->second = tmp_obs;
    }
    else {                                    // 如果障碍物之前不存在，加入到哈希表中
      obstalce_trajs_.insert(std::make_pair(tmp_obs.Id_, tmp_obs));
    }

    // 删除超时的障碍物轨迹
    ros::Time time_now = ros::Time::now(); //当前时刻
    std::vector<std::unordered_map<int, obstacle_traj>::iterator> elements_to_remove;

    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

      double time_out = (time_now - traj_iter->second.start_time).toSec();

      if(time_out >= 3.0 - 1e-3 && traj_iter != obstalce_trajs_.end()) {  // 删除超时的障碍物轨迹
        elements_to_remove.push_back(traj_iter);
      }
    }

    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // std::cout << "obstalce_trajs_.size() := " << obstalce_trajs_.size() << std::endl;

    // 预测轨迹可视化
    // show_Obs_traj();

    // 发布mpc-dcbf的预测轨迹消息
    // pub_DCBF_traj();

    // 发布TEB的预测轨迹消息
    // pub_TEB_traj();
  }

  void predict_Traj_Callback(const dynamic_simulator::DynTraj& msg)     // 接收障碍物预测轨迹的回调函数
  {
    obstacle_traj tmp_obs;    // 将障碍物消息转为vo_obstacle格式
    tmp_obs.Id_ = msg.id;
    tmp_obs.start_time = msg.header.stamp;
    tmp_obs.coeff_x <<  msg.pwp_mean.all_coeff_x[0].data[0],
                        msg.pwp_mean.all_coeff_x[0].data[1],
                        msg.pwp_mean.all_coeff_x[0].data[2];
    tmp_obs.coeff_y <<  msg.pwp_mean.all_coeff_y[0].data[0],
                        msg.pwp_mean.all_coeff_y[0].data[1],
                        msg.pwp_mean.all_coeff_y[0].data[2];
    tmp_obs.Time0 = msg.pwp_mean.times[0];
    tmp_obs.Time1 = msg.pwp_mean.times[1];
    tmp_obs.bbox_ << msg.bbox[0], msg.bbox[1];

    Eigen::Vector2d bbox_half = tmp_obs.bbox_ / 2.0;  // 取bbox外接圆半径
    tmp_obs.circle_R_ = bbox_half.norm();

    // 更新障碍物轨迹容器
    auto find_ptr = obstalce_trajs_.find(tmp_obs.Id_);
    if (find_ptr != obstalce_trajs_.end()) {  // 如果障碍物之前已存在，替换哈希表中对应值
      find_ptr->second = tmp_obs;
    }
    else {                                    // 如果障碍物之前不存在，加入到哈希表中
      obstalce_trajs_.insert(std::make_pair(tmp_obs.Id_, tmp_obs));
    }

    // 删除操作1: 删除超时的障碍物轨迹
    ros::Time time_now = ros::Time::now(); //当前时刻
    std::vector<std::unordered_map<int, obstacle_traj>::iterator> elements_to_remove;

    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

      double time_out = time_now.toSec() - traj_iter->second.Time0;

      if(time_out >= 1.0 - 1e-3 && traj_iter != obstalce_trajs_.end()) {  // 删除超时的障碍物轨迹
        elements_to_remove.push_back(traj_iter);
      }
    }
    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // 删除操作2: 加速度过大的无效障碍物轨迹
    elements_to_remove.clear();
    for (auto traj_iter = obstalce_trajs_.begin(); traj_iter != obstalce_trajs_.end(); traj_iter++) {

      double time_rel = time_now.toSec() - traj_iter->second.Time0;

      double acc_x = 2.0 * traj_iter->second.coeff_x[0] / 25.0;
      double acc_y = 2.0 * traj_iter->second.coeff_y[0] / 25.0;

      // std::cout << "acc_x := " << acc_x << ", acc_y :=" << acc_y << std::endl;

      if (abs(acc_x) > 2.0 || abs(acc_y) > 2.0 ) {
        elements_to_remove.push_back(traj_iter);
      }
    }
    // 删除临时容器中的元素
    for (const auto& elem : elements_to_remove) {
      obstalce_trajs_.erase(elem);
    }

    // std::cout << "obstalce_trajs_.size() := " << obstalce_trajs_.size() << std::endl;

    // 预测轨迹可视化
    // show_Obs_traj();

    // 发布mpc-dcbf的预测轨迹消息
    // pub_DCBF_traj();

    // 发布TEB的预测轨迹消息
    // pub_TEB_traj();
  }

};

#endif