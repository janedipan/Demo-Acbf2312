/* 收集实验数据并输出结果 */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include "obs_manager_for_data_process/obs_manager.hpp"

std::shared_ptr<Obs_Manager> obs_Manager_;

std::vector<nav_msgs::Odometry> odom_vector_;   // 存储机器人里程
nav_msgs::Odometry odom_now_;                   // 机器人当前里程

double min_dist_to_obs_ = 1000.0;               // 障碍物最短距离
double mean_vel_;                               // 平均速度
double mean_ang_;                               // 平均角速度
double var_vel_;                                // 速度方差
double var_ang_;                                // 角速度方差
double traj_length_;                            // 路径长度
double distance_g;
Eigen::VectorXd distance_o;
Eigen::MatrixXd collision_o_;                    // 定义为obs_nx3的矩阵：碰撞符号，第一次碰撞时间，碰撞次数；会在定时器中实时更新000000000000000
int obs_num = 5;
int collision_times = 0;

ros::Time plan_time_start;
ros::Time plan_time_end;
double traj_time_;
Eigen::Vector2d end_pt_;
bool start_sign = false;
bool end_sign   = false;
bool record_sign, record_sign1;

std::vector<std::string> controller_ls = {
    "None",
    "DC",
    "SCBF",
    "DCBF",
    "ACBF",
    "C+ACBF",
    "A+ACBF"
};

// 定义用于表格输出的数据 traj_length, traj_time, mean_vel0, mean_vel1, val_vel0, val_vel1, col_times, min_dist
Eigen::VectorXd output_data = Eigen::VectorXd::Zero(8);

// 定义用于记录障碍物距离的向量数组
std::vector<std::pair<double, double>> distance_to_obs_ls;


std::string controller_name;
int controller_; // DC, CBF, DCBF, ACBF,DC+ACBF, ADSM+ACBF
int scenario_; // 0~5
std::string file_name     = "/home/pj/jane_ws-github/dynamic_avoidance/src/swarm_test/docs/data0715/data.csv";
std::string file_name1     = "/home/pj/jane_ws-github/dynamic_avoidance/src/swarm_test/docs/data0729/data.csv";

void check_collision(Eigen::VectorXd distance_o, Eigen::MatrixXd *collision_o, double t_)
{
  for(int i = 0; i<obs_num; i++)
  {
    if(distance_o[i]<=0){
      // 检测到占用
      if((*collision_o)(i,0) == 0.0){
        // 上一采样时间没占用，更新碰撞时间，增加次数
        (*collision_o)(i,0) = 1.0;
        (*collision_o)(i,1) = t_;
        (*collision_o)(i,2) += 1.0;
      }
      else
      {
        // 上一采样时间有占用
        if(t_-(*collision_o)(i,1) >= 1.0){
          // 超过阈值时间，认为是多次碰撞，更新碰撞时间，增加次数
          (*collision_o)(i,1) = t_;
          (*collision_o)(i,2) += 1.0;
        }
      }
    }
    else{
      // 检测到没占用
      if((*collision_o)(i,0) != 0){
        (*collision_o)(i,0) = 0;
      }
    }
  }
}

void rcvOdomCallBack (const nav_msgs::Odometry& odom_msg) {
  odom_now_ = odom_msg;
  nav_msgs::Odometry odom_last_;
  if(!odom_vector_.empty()) odom_last_ = odom_vector_.back();
  odom_vector_.push_back(odom_msg);

  if (odom_vector_.size() < 2) {  // 累计两个以上的odom后开始计算
    return;
  }

  // STEP 1: 计算路径长度
  // traj_length_ = 0.0;
  // Eigen::Vector2d last_odom(odom_vector_[0].pose.pose.position.x, 
  //                           odom_vector_[0].pose.pose.position.y);
  // for (int i = 1; i < odom_vector_.size(); i++) {
  //   Eigen::Vector2d odom_i(odom_vector_[i].pose.pose.position.x, 
  //                          odom_vector_[i].pose.pose.position.y);
  //   double dis = (odom_i - last_odom).norm();
  //   if (dis <= 0.01) {
  //     dis = 0.0;
  //   }
  //   traj_length_ += dis;
  //   last_odom = odom_i;
  // }
  Eigen::Vector2d lastpos{odom_last_.pose.pose.position.x, odom_last_.pose.pose.position.y};
  Eigen::Vector2d currpos{odom_now_.pose.pose.position.x, odom_now_.pose.pose.position.y};
  double dis = (currpos-lastpos).norm();
  // dis = dis >= 0.005? dis: 0.0;
  // std::cout<< "dis := "<< dis<< std::endl;
  traj_length_ += dis;

  std::cout << "traj_leagth := " << traj_length_ << std::endl;
  // ----------更新导航路长

  // STEP 2: 计算平均速度和速度方差
  mean_vel_ = 0.0;
  mean_ang_ = 0.0;
  std::vector<double> vel_vector_;  // 存储机器人线速度
  std::vector<double> ang_vector_;  // 存储机器人角速度
  for (int i = 0; i < odom_vector_.size(); i++) {
    double vel_x_i = odom_vector_[i].twist.twist.linear.x;
    double ang_x_i = odom_vector_[i].twist.twist.angular.z;
    if (abs(vel_x_i) < 0.01) {
      vel_x_i = 0.0;
      continue;
    }
    vel_vector_.push_back(vel_x_i);
    mean_vel_ += abs(vel_x_i);

    if (abs(ang_x_i) < 0.005) {
      ang_x_i = 0.0;
      continue;
    }    
    ang_vector_.push_back(ang_x_i);
    mean_ang_ += abs(ang_x_i); 
  }
  mean_vel_ = vel_vector_.size() < 1 ? mean_vel_ : mean_vel_ / vel_vector_.size(); // ----------更新平均速度1
  mean_ang_ = ang_vector_.size() < 1 ? mean_ang_ : mean_ang_ / ang_vector_.size(); // ----------更新平均速度2
  // std::cout << "Mean_vel := " << mean_vel_ << std::endl;
  
  // ----------更新速度1方差
  var_vel_ = 0.0;
  for (int i = 0; i < vel_vector_.size(); i++) {
    var_vel_ += pow(vel_vector_[i] - mean_vel_, 2);
  }
  var_vel_ = vel_vector_.size() < 1 ? var_vel_ : var_vel_ / vel_vector_.size();
  std::cout << "Var_vel := " << var_vel_ << std::endl;

  // ----------更新速度2方差
  var_ang_ = 0.0;
  for (int i = 0; i < ang_vector_.size(); i++) {
    var_ang_ += pow(ang_vector_[i] - mean_ang_, 2);
  }  
  var_ang_ = ang_vector_.size() < 1 ? var_ang_ : var_ang_ / ang_vector_.size();
  std::cout << "Var_ang := " << var_ang_ << std::endl;

  // STEP 3: 输出障碍物最近距离, 目标点距离
  for(int i = 0; i<distance_o.size(); i++){
    std::cout << "--- dist_to_obs-num " << i<< " :="<< distance_o[i]-0.4-0.4 << std::endl;
    distance_to_obs_ls.push_back(std::make_pair(traj_time_, distance_o[i]));
  }
  std::cout << "Min_dist_to_obs := " << min_dist_to_obs_ << std::endl;
  std::cout << "Dist_to_goal := " << distance_g << std::endl;

  // STEP 4: 输出导航时间
  if(start_sign){
    // std::cout<< "Travel_time := "<< (plan_time_end-plan_time_start).toSec() << std::endl;
    std::cout<< "Travel_time := "<< traj_time_ << std::endl;
  }

  // STEP 5: 输出机器人控制量
  std::cout<< "Robot's linear|anger velocity := "<< odom_now_.twist.twist.linear.x<< "|"<< odom_now_.twist.twist.angular.z <<std::endl;
  
  // ---------- 设置碰撞检查，统计碰撞次数
  // std::cout<< "collision_times := "<< collision_times<< std::endl;

  std::cout<< "------------- next -------------"<< std::endl;
}

void stateCheckCallback (const ros::TimerEvent& e) {
  Eigen::Vector2d pos_now(odom_now_.pose.pose.position.x,
                          odom_now_.pose.pose.position.y);
  distance_g = (end_pt_-pos_now).norm();
  // 当进入end_sign状态后，plan_time_end将不再更新, 更新plan_time_end导航时间
  if(!end_sign && start_sign){
    plan_time_end = ros::Time::now();
  }
  if(distance_g <= 0.55 && !end_sign && start_sign){
    end_sign = true;
    plan_time_end = ros::Time::now();
  }
  traj_time_ = (plan_time_end-plan_time_start).toSec();
  ros::Time cur_time = ros::Time::now();
  std::vector<Eigen::Vector4d> posVel_list;   // 获取当前时间的障碍物状态
  std::vector<double> radius_list;            // 获取当前时间的障碍物半径
  obs_Manager_->get_obs_state(cur_time, posVel_list, radius_list);
  // ---------- 更新min_dist_to_obs 距离障碍物的最短距离
  distance_o = Eigen::VectorXd::Zero(posVel_list.size());
  double ROBOT_RADIUS = 0.40;
  for (int i = 0; i < posVel_list.size(); i++) {
    // p_r - 0.4 - 0.6; set safety_dis = 0.2
    double dist_i_tmp0 = (pos_now - posVel_list[i].head(2)).norm() - radius_list[i] - ROBOT_RADIUS;
    double dist_i_tmp = (pos_now - posVel_list[i].head(2)).norm();
    if(dist_i_tmp0 <= 0){
      collision_times ++;
    }
    distance_o[i] = dist_i_tmp;
    min_dist_to_obs_ = std::min(min_dist_to_obs_, dist_i_tmp0);
  }
}

void waypointCallback(const geometry_msgs::PoseStamped& msg) 
{
  std::cout<< "----------receive goal----------"<< std::endl;
  if (!start_sign){
    end_pt_<< msg.pose.position.x, msg.pose.position.y; 
    start_sign = true;
    plan_time_start = ros::Time::now();      // 记录开始时间
  }
  else
  {
    return;
  }
}

void writeToCSVFile(Eigen::VectorXd &output_data) {
    // data需要输入绝对路径，std::ofstream::app意味追加模型，默认是刷写
    std::ofstream file;
    file.open(file_name, std::ofstream::app);
    if (file.is_open()) {
      file<< std::endl;
      file<< ",";
      file<< scenario_<< ",";
      file<< controller_name<< ",\t";
      for (int i=0; i<8; i++){
        file<< output_data[i];
        if(i<7) file<< ",\t";
      }
      file.close();
      std::cout << "\033[1;33m----- Data written to file successfully. -----\033[0m" << std::endl;
    } 
    else {
      // 输出错误消息，如果文件无法打开
      std::cerr << "Error: Unable to open file for writing." << std::endl;
    }
}

void writeToCSVFile1(std::vector<std::pair<double, double>> &output_data){
  std::ofstream file;
  file.open(file_name1, std::ofstream::app);
  if(file.is_open()){
    file<< std::endl;
    file<< controller_<< "-traj_time"<< ",";
    for(int i=0; i<output_data.size(); i++){
      file<< output_data[i].first<< ",";
    }

    file<< std::endl;
    file<< controller_<< "-obs_dist"<< ",";
    for(int i=0; i<output_data.size(); i++){
      file<< output_data[i].second<< ",";
    }    
  }
}

int main (int argc, char** argv) {

	ros::init (argc, argv, "data_processor_node");
	ros::NodeHandle nh;
  nh.param("data_processor_node/data/record_sign", record_sign, false);
  nh.param("data_processor_node/data/record_sign1", record_sign1, false);
  nh.param("data_processor_node/scenario", scenario_, 1);
  nh.param("data_processor_node/controller", controller_, 0);
  controller_name = controller_ls[controller_];
  // std::cout<< "Controller_name := "<< controller_name<< std::endl;
  // 初始化障碍物管理对象
  obs_Manager_.reset(new Obs_Manager);
  obs_Manager_->init(nh);
  traj_length_ = 0.0;
	ros::Subscriber odom_raw_sub  = nh.subscribe("/odometry", 1, rcvOdomCallBack);
  ros::Subscriber waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, waypointCallback);
  ros::Timer checker_timer      = nh.createTimer(ros::Duration(0.02), stateCheckCallback);  // 定时碰撞检查
  while (ros::ok())
  {
    ros::spin();
  }
  output_data[0] = traj_length_;
  output_data[1] = traj_time_;
  output_data[2] = mean_vel_;
  output_data[3] = mean_ang_;
  output_data[4] = var_vel_;
  output_data[5] = var_ang_;
  output_data[6] = collision_times;
  output_data[7] = min_dist_to_obs_;
  if(record_sign){
    writeToCSVFile(output_data);
  }

  if(record_sign1){
    writeToCSVFile1(distance_to_obs_ls);
    std::cout<< "record_sign1 successful"<< std::endl;
  }
	return 0;
}

