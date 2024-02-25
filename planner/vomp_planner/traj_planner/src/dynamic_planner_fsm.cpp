#include "plan_manage/dynamic_planner_fsm.h"

void DynamicReplanFSM::init(ros::NodeHandle& nh){
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  plan_success = false;

  /*  参数设置  */
  nh.param("drone_id", drone_id_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("use_presetGoal", use_presetGoal_, false);
  nh.param("goal_x",   user_target[0], 0.0);
  nh.param("goal_y",   user_target[1], 0.0);
  nh.param("goal_yaw", user_target[2], 0.0);

  // 规划管理：主要的模块 -----------------------------
  planner_manager_.reset(new PlanManager);
  planner_manager_->initPlanModules(nh);

  /* 状态机更新callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.02), &DynamicReplanFSM::execFSMCallback, this);
  
  // 定时器：发布参考轨迹给mpc
  cmd_timer_ = nh.createTimer(ros::Duration(0.05), &DynamicReplanFSM::pubRefTrajCallback, this);

  waypoint_sub_ = nh.subscribe("waypoints", 1, &DynamicReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &DynamicReplanFSM::odometryCallback, this);

  mpc_traj_pub = nh.advertise<std_msgs::Float32MultiArray>("mpc/traj_point", 50);

  std::cout << "[Vehicle " << drone_id_ << " FSM]: init done" << std::endl;
}

void DynamicReplanFSM::init(ros::NodeHandle& nh1, ros::NodeHandle& nh2, ros::NodeHandle& nh3) {
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;
  plan_success = false;

  /*  参数设置  */
  nh1.param("drone_id", drone_id_, -1);
  nh1.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh1.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh1.param("use_presetGoal", use_presetGoal_, false);
  nh1.param("goal_x",   user_target[0], 0.0);
  nh1.param("goal_y",   user_target[1], 0.0);
  nh1.param("goal_yaw", user_target[2], 0.0);

  // 规划管理：主要的模块 -----------------------------
  planner_manager_.reset(new PlanManager);
  planner_manager_->initPlanModules(nh3);

  /* callback */
  exec_timer_   = nh1.createTimer(ros::Duration(0.05), &DynamicReplanFSM::execFSMCallback, this);
  
  // 定时器：发布参考轨迹给mpc
  cmd_timer_ = nh1.createTimer(ros::Duration(0.05), &DynamicReplanFSM::pubRefTrajCallback, this);

  waypoint_sub_ = nh2.subscribe("waypoints", 1, &DynamicReplanFSM::waypointCallback, this);
  odom_sub_ = nh2.subscribe("/odom_world", 1, &DynamicReplanFSM::odometryCallback, this);

  mpc_traj_pub = nh1.advertise<std_msgs::Float32MultiArray>("mpc/traj_point", 50);

  std::cout << "[Vehicle " << drone_id_ << " FSM]: init done" << std::endl;
}

void DynamicReplanFSM::pubRefTrajCallback(const ros::TimerEvent& e){

  if(!plan_success){ // 没有规划成功。返回
    return;
  }

  ros::Time start_time_ = planner_manager_->get_startTime();  // 获得规划的起始时间
  double t_cur = (ros::Time::now() - start_time_).toSec();    // 相对于规划起始时刻的时间
  double traj_duration = planner_manager_->get_duration();    // 轨迹持续时间
  t_cur = min(t_cur, traj_duration);                          // 防止溢出

  std_msgs::Float32MultiArray waypoint_array;                 // 参考轨迹消息类型
  Eigen::Vector3d traj_pt;                                    // 参考轨迹点

  double delta_t = 0.05;

  double last_yaw = yaw_pos_;

  for(int i = 0; i < 20; i++) {
    double t_add = t_cur + i * delta_t;

    if(t_add < traj_duration - delta_t) {
      Eigen::Vector2d pos_now  = planner_manager_->get_optimal_traj().evaluateDeBoorT(t_add);
      Eigen::Vector2d pos_next = planner_manager_->get_optimal_traj().evaluateDeBoorT(t_add + delta_t);
      Eigen::Vector2d pos_diff = pos_next - pos_now;
      // 计算当前点的yaw角
      double yaw_now = pos_diff.norm() > 0.01 ? atan2(pos_diff.y(), pos_diff.x()) : last_yaw;
      last_yaw = yaw_now;   // 保存当前的yaw

      traj_pt.head(2) = pos_now;
      traj_pt[2] = yaw_now;

    } else if(t_add >= traj_duration - delta_t){
      Eigen::Vector2d pos_last  = planner_manager_->get_optimal_traj().evaluateDeBoorT(traj_duration - delta_t);
      Eigen::Vector2d pos_now = planner_manager_->get_optimal_traj().evaluateDeBoorT(traj_duration);
      Eigen::Vector2d pos_diff = pos_now - pos_last;
      // 计算当前点的yaw角
      double yaw_now = pos_diff.norm() > 0.01 ? atan2(pos_diff.y(), pos_diff.x()) : last_yaw;
      last_yaw = yaw_now;   // 保存当前的yaw

      traj_pt.head(2) = pos_now;
      traj_pt[2] = yaw_now;

    }else{
      ROS_WARN("[FSM]: invalid time to pub." );
    }
    // 拿到 X, Y, Theta
    waypoint_array.data.push_back(traj_pt[0]);      // X
    waypoint_array.data.push_back(traj_pt[1]);      // Y
    waypoint_array.data.push_back(traj_pt[2]);      // Theta
  }

  mpc_traj_pub.publish(waypoint_array);             // 发送参考轨迹
}

void DynamicReplanFSM::waypointCallback(const geometry_msgs::PoseStamped& msg){
  std::cout << "Triggered!" << std::endl;
  trigger_ = true;

  if (use_presetGoal_) {
    end_pt_ << user_target[0], user_target[1];
    ROS_INFO_STREAM("TARGET=" << end_pt_.transpose());
    ROS_INFO("[node] receive the planning target");
    have_target_ = true;
  }
  else {
    end_pt_ << msg.pose.position.x, msg.pose.position.y;

    ROS_INFO_STREAM("TARGET=" << end_pt_.transpose());
    ROS_INFO("[node] receive the planning target");
    have_target_ = true;
  }

  if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void DynamicReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);

  yaw_pos_ = atan2(rot_x(1), rot_x(0));

  have_odom_ = true;
}

void DynamicReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call){
  std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  std::cout << "\033[34m[Vehicle " << drone_id_ << " " << pos_call + "]: from " + state_str[int(exec_state_)] + " to " + state_str[int(new_state)] << "\033[0m" << std::endl;
  exec_state_         = new_state;
}

void DynamicReplanFSM::printFSMExecState(){
  std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  std::cout << "\033[34m[Vehicle " << drone_id_ << " FSM]: state: " + state_str[int(exec_state_)] << "\033[0m" << std::endl;
}

void DynamicReplanFSM::execFSMCallback(const ros::TimerEvent& e){
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) std::cout << "no odom............." << std::endl;
    if (!trigger_) std::cout << "wait for goal............." << std::endl;
    fsm_num = 0;
  }

  switch (exec_state_){
    case INIT: 
    {
      if (!have_odom_){return;}
      if (!trigger_){return;}
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: 
    {
      if (!have_target_){return;}
      else 
      {
        // 当前与终点距离小于设定阈值，停止规划
        Eigen::Vector2d target_pt, odom_now;
        target_pt << end_pt_[0] , end_pt_[1];
        odom_now << odom_pos_[0] , odom_pos_[1];
        if((target_pt - odom_now).norm() < 0.1) {
          have_target_ = false;
          break;
        }else{
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case GEN_NEW_TRAJ: 
    {
      start_pt_ = odom_pos_.head(2);
      // start_vel_ = odom_vel_.head(2);
      start_acc_.setZero();
      end_vel_.setZero();
      start_yaw_ = yaw_pos_;
      // 根据yaw角给定一个初始速度
      double init_vel = 1.0;
      start_vel_ << init_vel * cos(start_yaw_), init_vel * sin(start_yaw_);

      bool success = callHybridReplan();  // 规划
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: 
    {
      // 离线轨迹执行模式
      ros::Time start_time_ = planner_manager_->get_startTime();  // 获得规划的起始时间

      double t_cur = (ros::Time::now() - start_time_).toSec();    // 相对于规划起始时刻的时间

      t_cur = min(t_cur, planner_manager_->get_duration());       // 轨迹持续时间

      auto traj_bspline = planner_manager_->get_optimal_traj();   // 拿到B样条轨迹

      Eigen::Vector2d pos = traj_bspline.evaluateDeBoorT(t_cur);  // 根据当前时间，索引到当前轨迹点

      if (t_cur > planner_manager_->get_duration() - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      // else if ((odom_pos_.head(2) - pos).norm() > 0.5){
      //   // 当前位置与轨迹的位置相差过大，重新规划新轨迹
      //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //   return;
      // }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // 终点与当前位置差小于no_replan_thresh_，不用重规划
        return;
      } 
      else if ((planner_manager_->get_startPoint() - pos).norm() < replan_thresh_) {
        // 起点与当前位置差小于replan_thresh_，不用重规划
        return;
      } 
      else {
        // 其他情况，重规划
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }
      
    case REPLAN_TRAJ: 
    {
      // 重规划模式：以当前时间轨迹状态为起点规划轨迹
      ros::Time start_time_ = planner_manager_->get_startTime();  // 获得规划的起始时间
      double t_cur = (ros::Time::now() + ros::Duration(0.05) - start_time_).toSec();    // 相对于规划起始时刻的时间
      t_cur = min(t_cur, planner_manager_->get_duration());       // 轨迹持续时间，并防止溢出

      auto traj_bspline = planner_manager_->get_optimal_traj();   // 拿到B样条轨迹
      auto vel_bspline = traj_bspline.getDerivative();            // 轨迹的导数，速度轨迹

      Eigen::Vector2d pos_now = traj_bspline.evaluateDeBoorT(t_cur);  // 根据当前时间，索引到当前[轨迹]点
      Eigen::Vector2d vel_now = vel_bspline.evaluateDeBoorT(t_cur);   // 根据当前时间，索引到轨迹[速度]点

      start_pt_ = pos_now;
      start_vel_ = vel_now;
      start_acc_.setZero();
      end_vel_.setZero();

      double next_time = t_cur + 0.10;                                       // 计算当前时间后一点，用来算yaw
      Eigen::Vector2d pos_next = traj_bspline.evaluateDeBoorT(next_time);   // 根据当前时间，索引到当前轨迹点

      Eigen::Vector2d pos_diff = pos_next - pos_now;
      start_yaw_ = pos_diff.norm() > 0.05 ? atan2(pos_diff.y(), pos_diff.x()) : yaw_pos_;                     // 计算当前点的yaw角

      bool success = callHybridReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

bool DynamicReplanFSM::callHybridReplan(){

  static int nums_of_planning = 1;
  static double cost_time_sum = 0.0;

  clock_t plan_time_start = clock();                  //开始时间

  plan_success = planner_manager_->hybridReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_);

  double plan_time_spend = double(clock() - plan_time_start) / CLOCKS_PER_SEC;

  // std::cout << "\033[32m[Vehicle " <<  drone_id_ << "]: Plan Local-Trajectory Spend Time := " << plan_time_spend << "\n\033[0m" << std::endl;

  cost_time_sum += plan_time_spend;
  double average_time = cost_time_sum / nums_of_planning;
  std::cout << "plan_time_spend:= " << plan_time_spend << std::endl;
  std::cout << "\033[32m[VOMP]: Local-Trajectory Generation Spend Time := " << average_time << "\n\033[0m" << std::endl;
  nums_of_planning++;

  return plan_success;
}