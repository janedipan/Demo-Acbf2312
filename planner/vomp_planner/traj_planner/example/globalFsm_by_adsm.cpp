#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>

#include "plan_manage/plan_manager.h"

PlanManager planner_manager_;
// ros接口
ros::Subscriber odom_sub;
ros::Subscriber waypoint_sub_;
ros::Publisher mpc_traj_pub_;

bool static_path;
bool is_odom_rcv_, is_target_rcv_, plan_success;
nav_msgs::Odometry odom_;
std::vector<Eigen::Vector2d> trajlist;
std::vector<ros::Time> timelist;
ros::Time start_time_, end_time_;

double cur_yaw_, start_yaw_;                                  // 记录yaw规划的起点
Eigen::Vector2d cur_pt_, cur_vel_;
Eigen::Vector2d start_pt_, start_vel_, start_acc_;  // start state
Eigen::Vector2d end_pt_, end_vel_;                  // target state

double v_max = 1.3;
double step_time, replan_thresh1, replan_thresh2;
double average_time, total_time, plan_num;

// 状态机标志位0-init, 1-wait_target, 2-gen_new_traj, 3-exec_traj, 4-replan_traj
int _fsm_sign;

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
    cur_yaw_ = atan2(rot_x(1), rot_x(0));

    cur_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y;
    // start_pt_ << odom_.pose.pose.position.x, odom_.pose.pose.position.y;
    // 根据yaw角给定一个初始速度-用于路径搜索
    // double init_vel = 1.2;
    double init_vel = msg.twist.twist.linear.x;
    // start_vel_ << init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);    
    cur_vel_ << init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);    
    start_acc_.setZero();

    is_odom_rcv_ = true;
}

void waypointCallback(const geometry_msgs::PoseStamped& msg) 
{
    end_pt_ << msg.pose.position.x, msg.pose.position.y;
    end_vel_.setZero();

    is_target_rcv_ = true;
}

void globalPathPub_Callback(const ros::TimerEvent& e) 
{
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

void maintenance_fsm(const ros::TimerEvent& e)
{

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num==40){
        if(!is_odom_rcv_)   std::cout << "............ no odom ............." << std::endl;
        if(!is_target_rcv_) std::cout << "............ wait for goal ............." << std::endl;
    }

    switch (_fsm_sign)
    {
    // init
    case 0:
    {
    
        if(!is_odom_rcv_) return;
        /* change fsm from 0 to 1 */
        _fsm_sign = 1;
        break;
    }
    // wait_target
    case 1:
    {
        // std::cout<< "............ wait_target .............\n";
        if(!is_target_rcv_) return;
        else{
            // 当前位置与终点小于设定阈值时，停止规划
            Eigen::Vector2d target_pt, odom_now;
            target_pt   << end_pt_[0], end_pt_[1];
            odom_now    << cur_pt_[0], cur_pt_[1];
            if((target_pt - odom_now).norm() < 0.1) {
                is_target_rcv_ = false;
                break;
            }
            else{
                /* change fsm from 1 to 2 */
                _fsm_sign = 2;
            }
            break;
        }
    }
    // gen_new_traj
    case 2:
    {
        std::cout<< "............ gen_new_traj .............\n";
        start_pt_ = cur_pt_;
        double init_vel = v_max*0.8;
        start_vel_<< init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);
        start_acc_.setZero();
        start_yaw_ = cur_yaw_;
        // 初始化路径搜索
        plan_success = planner_manager_.hybridReplanFsm(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);
        if(plan_success){
            /* change fsm from 2 to 3 */
            trajlist = planner_manager_.getGlobalPath(step_time);
            timelist = planner_manager_.get_time_list(step_time);
            start_time_ = timelist.front();
            end_time_   = timelist.back();
            _fsm_sign = 3;
        }
        break;
    }
    // exec_traj
    case 3:
    {
        // 离线轨迹执行
        // std::cout<< "............ exce_traj .............\n";
        ros::Time t_cur = ros::Time::now();
        // ROS_INFO("fsm:time is: %f", t_cur.toSec());
        // 根据时间获取当前全局参考路径的路点
        Eigen::Vector2d pos = planner_manager_.evaluateFrontPose(t_cur, timelist);
        // std::cout<< "............ here0 .............\n";
        // std::cout<< "this traj time is: "<<t_cur.toSec()-start_time_.toSec()<< std::endl;
        if((t_cur-end_time_).toSec() >-1e-2){
            is_target_rcv_ = false;
            /* change fsm from 3 to 1 */
            _fsm_sign = 1;
        }
        else if((end_pt_-pos).norm()<replan_thresh1){ 
            return;
        }
        else if((start_pt_-pos).norm()<replan_thresh2) {
            return;
        }
        else if(static_path){
            return;
        }
        else{
            _fsm_sign = 4;
        }
        break;
    }
    // replan_traj
    case 4:
    {
        // 重规划模式
        // std::cout<< "\033[33m............ replan_traj .............\033[0m\n";
        start_pt_ = cur_pt_;
        // start_vel_ = cur_vel_;
        double init_vel = v_max;
        init_vel = 1.4;
        start_vel_<< init_vel * cos(cur_yaw_), init_vel * sin(cur_yaw_);
        
        start_acc_.setZero();
        start_yaw_ = cur_yaw_;
        // std::cout<< start_vel_<< std::endl;
        plan_success = planner_manager_.hybridReplanFsm(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);     
        if(plan_success){
            trajlist = planner_manager_.getGlobalPath(step_time);
            timelist = planner_manager_.get_time_list(step_time);
            start_time_ = timelist.front();
            end_time_   = timelist.back();
            _fsm_sign = 3;
        }
        else{
            _fsm_sign = 2;
        }
        break;
    }

    }   
}


int main(int argc, char** argv){
    ros::init(argc, argv, "global_fsm_by_adsm");
    ros::NodeHandle nh("~");

    nh.param("obs_manager/step_time", step_time, 0.1);
    nh.param("fsm/thresh_replan", replan_thresh1, -1.0);
    nh.param("fsm/thresh_no_replan", replan_thresh2, -1.0);
    nh.param("fsm/static_path", static_path, false);
    ROS_WARN("[Test_demo]: global_path step_time is %f", step_time);
    if(static_path) ROS_WARN("[Test_demo]: use static global path");
    plan_num    = 1.0;
    average_time= 0.0;
    total_time  = 0.0;
    _fsm_sign   = 0;
    planner_manager_.initPlanManage(nh);

    odom_sub      = nh.subscribe("/odometry", 50, odomCallback);
    waypoint_sub_ = nh.subscribe("/waypoints", 1, waypointCallback);
    mpc_traj_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 20);

    ros::Timer mainte_fsm = nh.createTimer(ros::Duration(0.02), maintenance_fsm);
    ros::Timer globbal_path_timer = nh.createTimer(ros::Duration(0.1), globalPathPub_Callback);

    ros::Duration(1.0).sleep();
    ROS_WARN("[Test_demo]: ready");
    ros::spin();
    return 0;
}

