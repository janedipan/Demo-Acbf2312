#include "mpc_core/mpc_tracker.h"

void mpc_tracker::initMPC(ros::NodeHandle& nh) {
  nh_ = nh;
  nh_.param("mpc/mpc_type", mpc_type_, -1);
  nh_.param("mpc/dt", dT_, 0.10);
  nh_.param("mpc/predict_steps", N_, 10);
  nh_.param("mpc/max_delta", max_delta, 0.6);
  nh_.param("mpc/max_speed", max_speed, 1.0);
  nh_.param("mpc/min_speed", min_speed, -1.0);
  nh_.param("mpc/max_speed_y", max_speed_y, 1.0);
  nh_.param("mpc/L", L_, 0.608);
  nh_.param<std::vector<double>>("mpc/matrix_q", Q, std::vector<double>());
  nh_.param<std::vector<double>>("mpc/matrix_r", R, std::vector<double>());

  has_odom = false;
  receive_traj_ = false;

  desired_state = Eigen::MatrixXd::Zero(3, N_ + 1);

  if(mpc_type_ == Ackman_MPC){                // 多态初始化: 阿克曼mpc
    mpc_solver_.reset(new MPC_Ackman());
    mpc_solver_->init(N_, dT_, max_speed, min_speed, max_speed_y, max_delta, L_, Q, R);
    ROS_WARN("Ackman_MPC init done");
  } 
  else if(mpc_type_ == Differential_MPC){     // 多态初始化: 差速mpc
    mpc_solver_.reset(new MPC_Differential());
    mpc_solver_->init(N_, dT_, max_speed, min_speed, max_speed_y, max_delta, L_, Q, R);
    ROS_WARN("Differential_MPC init done");
  }
  else if(mpc_type_ == Quadruped_MPC){        // 多态初始化: 四足mpc
    mpc_solver_.reset(new MPC_Quadruped());
    mpc_solver_->init(N_, dT_, max_speed, min_speed, max_speed_y, max_delta, L_, Q, R);
    ROS_WARN("Quadruped_MPC init done");
  }
  else {
    ROS_ERROR("ERROR: MPC type is wrong, please check the config/param.yaml!!!!!");
  }

  // 初始化话题接收
  odom_sub_ = nh_.subscribe("/odom", 1, &mpc_tracker::rcvOdomCallBack, this);
  traj_sub_ = nh_.subscribe("/traj", 1, &mpc_tracker::rcvTrajCallBack, this);
  // 初始化话题发布者
  cmd_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 200);
  predict_pub = nh_.advertise<nav_msgs::Path>("/predict_path", 10);
  // 定时器回调：MPC主循环
  cmd_timer_ = nh_.createTimer(ros::Duration(dT_), &mpc_tracker::cmdCallback, this);
}

void mpc_tracker::rcvTrajCallBack(std_msgs::Float32MultiArray msg) {
  receive_traj_ = true;
  ref_traj_msg = msg;
}

void mpc_tracker::rcvOdomCallBack(nav_msgs::OdometryPtr msg) {
  Eigen::Quaterniond q( msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z);
  Eigen::Matrix3d R(q);

  current_state <<  msg->pose.pose.position.x, 
                    msg->pose.pose.position.y, 
                    atan2(R.col(0)[1],R.col(0)[0]);
  has_odom = true;
}

void mpc_tracker::cmdCallback(const ros::TimerEvent &e) {
  if (!has_odom && !receive_traj_) {
    std::cout << "MPC WARNING: no odom & no reference traj\n";
    
    return;
  }
  else if (!has_odom) {
    std::cout << "MPC WARNING: no odom\n";
    return;
  }
  else if (!receive_traj_) {
    std::cout << "MPC WARNING: no reference traj\n";
    return;
  }

  // TODO: 还需要加上对终点的处理

  // 

  // 


  for(int i = 0; i < N_; i++)
  {
    desired_state(0, i) = ref_traj_msg.data[3 * i + 0];
    desired_state(1, i) = ref_traj_msg.data[3 * i + 1];
    desired_state(2, i) = ref_traj_msg.data[3 * i + 2];
  }

  smooth_yaw(desired_state);

  clock_t time_start = clock();

  //// MPC求解!!!!!!!!!!!!!!!!!!!!!!!! ////
  bool success = mpc_solver_->solve(current_state, desired_state);

  if(success) {
    std::vector<double> control_cmd = mpc_solver_->getFirstU();     // 拿到最优控制

    if(mpc_type_ == Differential_MPC || mpc_type_ == Ackman_MPC) {  // 对于差速和阿克曼等非完整约束机器人
      cmd_vel.linear.x  = control_cmd[0];
      cmd_vel.angular.z = control_cmd[1];
    } 
    else if (mpc_type_ == Quadruped_MPC) {                          // 对于全向小车和四足等完整约束机器人
      cmd_vel.linear.x  = control_cmd[0];
      cmd_vel.linear.y  = control_cmd[1];
      cmd_vel.angular.z = control_cmd[2];
    }

    cmd_twist_pub_.publish(cmd_vel);    // 发布控制指令

    // 输出预测轨迹
    std::vector<double> predict_states = mpc_solver_->getPredictX();
    drawPredictTraj(predict_states);
  }
  else {     // MPC求解失败处理
    ROS_ERROR("ERROR: MPC 优化求解失败!");
    cmd_vel.linear.x  = 0.0;
    cmd_vel.linear.y  = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_twist_pub_.publish(cmd_vel);    // 发布静止指令
  }

  clock_t time_end = clock();
  // std::cout << "MPC calc time := " << double(time_end-time_start)/CLOCKS_PER_SEC << std::endl;
}

void mpc_tracker::smooth_yaw(Eigen::MatrixXd& ref_traj) {
  double dyaw = ref_traj(2, 0) - current_state[2];

  while (dyaw >= M_PI / 2)
  {
    ref_traj(2, 0) -= M_PI * 2;
    dyaw = ref_traj(2, 0) - current_state[2];
  }
  while (dyaw <= -M_PI / 2)
  {
    ref_traj(2, 0) += M_PI * 2;
    dyaw = ref_traj(2, 0) - current_state[2];
  }

  for (int i = 0; i < N_ - 1; i++)
  {
    dyaw = ref_traj(2, i+1) - ref_traj(2, i);
    while (dyaw >= M_PI / 2)
    {
      ref_traj(2, i+1) -= M_PI * 2;
      dyaw = ref_traj(2, i+1) - ref_traj(2, i);
    }
    while (dyaw <= -M_PI / 2)
    {
      ref_traj(2, i+1) += M_PI * 2;
      dyaw = ref_traj(2, i+1) - ref_traj(2, i);
    }
  }
}


void mpc_tracker::drawPredictTraj(std::vector<double>& predict_states) {
  nav_msgs::Path predict_path;
  geometry_msgs::PoseStamped pose_msg;
  predict_path.header.frame_id = "world";
  predict_path.header.stamp = ros::Time::now();

  pose_msg.pose.orientation.w = 1.0;
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 0.0;
  pose_msg.pose.orientation.z = 0.0;

  for (int i = 0; i < predict_states.size(); i += 2) {
    pose_msg.pose.position.x = predict_states[i + 0];
    pose_msg.pose.position.y = predict_states[i + 1];
    predict_path.poses.push_back(pose_msg);
  }
  predict_pub.publish(predict_path);
}