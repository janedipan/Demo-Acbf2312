#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>

#include "plan_manage/plan_manager.h"

PlanManager planner_manager_;
ros::Subscriber odom_sub;
ros::Subscriber waypoint_sub_;
ros::Publisher mpc_traj_pub_;

nav_msgs::Odometry odom_;
bool is_odom_rcv_, is_target_rcv_;
bool plan_success;
std::vector<Eigen::Vector2d> trajlist;

double start_yaw_;                                  // 记录yaw规划的起点
Eigen::Vector2d start_pt_, start_vel_, start_acc_;  // start state
Eigen::Vector2d end_pt_, end_vel_;                  // target state

double v_max = 1.3;
double step_time;

void odomCallback(const nav_msgs::Odometry& msg) 
{
    if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
    odom_ = msg;
    
    Eigen::Quaterniond orient;
    orient.w() = msg.pose.pose.orientation.w;
    orient.x() = msg.pose.pose.orientation.x;
    orient.y() = msg.pose.pose.orientation.y;
    orient.z() = msg.pose.pose.orientation.z;
    Eigen::Vector3d rot_x = orient.toRotationMatrix().block(0, 0, 3, 1);
    start_yaw_ = atan2(rot_x(1), rot_x(0));

    start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y;
    // 根据yaw角给定一个初始速度-用于路径搜索
    double init_vel = 1.2;
    start_vel_ << init_vel * cos(start_yaw_), init_vel * sin(start_yaw_);    
    start_acc_.setZero();

    is_odom_rcv_ = true;
}

void waypointCallback(const geometry_msgs::PoseStamped& msg) 
{
    end_pt_ << msg.pose.position.x, msg.pose.position.y;
    end_vel_.setZero();

    is_target_rcv_ = true;
}

void globalPathReplan_Callback(const ros::TimerEvent& e) 
{
  if (!is_target_rcv_ || !is_odom_rcv_) 
  {
    return;
  }
  // std::cout<< "get neccessary message"<< std::endl;
  clock_t plan_time_start = clock();                  //开始时间
  // 需要修改getKinoTraj(detal_t的时间值)
  plan_success = planner_manager_.hybridReplanAdsm(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);
  double plan_time_spend = double(clock() - plan_time_start) / CLOCKS_PER_SEC;
  std::cout << "plan_time_spend:= " << plan_time_spend << std::endl;
  
  if(plan_success){
    // std::cout<< "global_path plan failed!"<< std::endl;
    trajlist.clear();
    trajlist = planner_manager_.getGlobalPath(step_time);
  } 
  else{
    std::cout<< "plan failed!" << std::endl;
  }
  
} 

void globalPathPub_Callback(const ros::TimerEvent& e) 
{
  
  // std::vector<Eigen::Vector2d> trajlist = planner_manager_.getGlobalPath(step_time);
  int num = static_cast<int>(trajlist.size());
  if (num == 0) return;
  nav_msgs::Path global_path;
  global_path.header.stamp = ros::Time::now();
  global_path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.orientation.x = 0;
  pose_stamped.pose.orientation.y = 0;
  pose_stamped.pose.orientation.z = 0;
  pose_stamped.pose.orientation.w = 1;

  int idx = 0;
  for(int i=0; i<num; i++){
    pose_stamped.header.seq = idx++;
    pose_stamped.pose.position.x = trajlist[i].x();
    pose_stamped.pose.position.y = trajlist[i].y();
    pose_stamped.pose.position.z = 0.0;

    global_path.poses.push_back(pose_stamped);
  }
  mpc_traj_pub_.publish(global_path);
} 


int main(int argc, char** argv){
    ros::init(argc, argv, "global_path_by_adsm");
    ros::NodeHandle nh("~");
    nh.param("/global_path_by_adsm/step_time", step_time, 0.1);

    planner_manager_.initPlanManage(nh);

    odom_sub      = nh.subscribe("/odometry", 50, odomCallback);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, waypointCallback);
    mpc_traj_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 20);
    
    ros::Timer replan_timer = nh.createTimer(ros::Duration(0.1), globalPathReplan_Callback);      // 定时全局路径搜索
    ros::Timer globbal_path_timer = nh.createTimer(ros::Duration(0.1), globalPathPub_Callback);   // 定时全局路径发布

    ros::Duration(1.0).sleep();
    ROS_WARN("[Test_demo]: ready");

    ros::spin();
    return 0;
}