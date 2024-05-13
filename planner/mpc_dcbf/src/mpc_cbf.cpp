#include "mpc_cbf.h"

void MPC_PLANNER::init_MPC_CBF(ros::NodeHandle& nh)
{
    nh_ = nh;
    // 定义控制器类型列表
    std::vector<std::string> controller_ls = {
        "None",
        "DC",
        "SCBF",
        "DCBF",
        "ACBF"
    };
    nh_.param("/local_planner/controller_type", controller, 0);
    Smetric_type = controller_ls[controller];
    nh_.param("/local_planner/mpc_frequency",   replan_period_, 5.0);
    nh_.param("/local_planner/step_time",       Ts_, 0.1);
    nh_.param("/local_planner/pre_step",        N_, 25);

    cur_state_.resize(5);
    cur_state_.setZero();
    goal_state_.resize(3, N_);
    goal_state_.setZero();
    last_input_.clear();
    last_state_.clear();

    has_odom_ = false; 
    has_traj_ = false;
    nums_of_planning = 1;
    cost_time_sum = 0;
    // ----------初始化mpc-solver参数
    double v_max = 1.3;
    double v_min = 1.0;
    double o_max = 0.8;
    // for mpc-Q 递增
    std::vector<double> Q = {1.0, 1.0, 0.05}; 
    std::vector<double> R = {0.1, 0.05};    
    // for mpc-Q 固定
    // std::vector<double> Q = {1.5, 1.5, 0.15}; 
    // std::vector<double> R = {0.1, 0.05};    
    double gamma    = 0.3;
    double safe_dist= 0.3+0.6; 

    solver.init_solver(Smetric_type, Ts_, N_, v_max, v_min, o_max, Q, R, gamma, safe_dist);

    timer_replan_   = nh_.createTimer(ros::Duration(1/replan_period_), &MPC_PLANNER::replanCallback, this);
    timer_pub       = nh_.createTimer(ros::Duration(0.02), &MPC_PLANNER::cmdCallback, this);
    sub_curr_state_ = nh_.subscribe("/Odometry", 1, &MPC_PLANNER::rcvOdomCallBack, this);
    sub_obs_        = nh_.subscribe("/obs_Manager_node/obs_predict_pub", 100, &MPC_PLANNER::rcvObsCallBack, this);
    sub_goal_       = nh_.subscribe("/global_path", 100, &MPC_PLANNER::rcvTrajCallBack, this);

    pub_local_path_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    pub_local_plan_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}

void MPC_PLANNER::cmdCallback(const ros::TimerEvent &e)
{
    pub_local_plan_.publish(cmd_vel);
}

void MPC_PLANNER::replanCallback(const ros::TimerEvent &e)
{
    std::lock_guard<std::mutex> curr_pose_lock_guard(curr_pose_mutex);
    std::lock_guard<std::mutex> global_path_lock_guard(global_path_mutex);
    std::lock_guard<std::mutex> obstacle_lock_guard(obstacle_mutex);

    
    if(!has_odom_ || !has_traj_){
        // std::cout << "MPC WARNING: no odom | no reference traj\n";
    }
    else{
        // std::cout << "start replan!\n";
        choose_goal_state();
        smooth_yaw(goal_state_);
        // ------------------------ MPC求解 ------------------------
        bool success = solver.imp_solve(&cur_state_, &goal_state_, &obs_matrix_);
        ros::Time time_in0 = ros::Time::now();
        if(success){
            // ROS_INFO("Solved successfullly!");
            std::vector<double> control_cmd     = solver.getFirstUp();
            cmd_vel.linear.x    = control_cmd[0];
            cmd_vel.angular.z   = control_cmd[1];

            std::vector<double> predict_traj    = solver.getPredictXp(); 
            // std::cout<<"the predict_traj is:"<< predict_traj<< std::endl;
            std::cout<<"the predict_cmd is:"<< control_cmd<< std::endl;
            pub_Predict_traj(predict_traj);
        }
        else{
            // ROS_WARN("Solved failed!");
            // cmd_vel.linear.x    = 0.0;
            // cmd_vel.angular.z   = 0.0;
            cmd_vel.linear.x    = solver.predict_u[0];
            cmd_vel.angular.z   = solver.predict_u[1];
            pub_Predict_traj(solver.predict_x);
        }
        // pub_local_plan_.publish(cmd_vel);
        ros::Time time_in1 = ros::Time::now();
        std::cout<< "replan_time =: "<< (time_in1-time_in0).toSec()<< std::endl;
        std::cout<< "------------------------------------------------------------------\n";
    }

}

void MPC_PLANNER::choose_goal_state()
{
    int waypoint_num  = global_path_.cols();
    double min_dist = std::numeric_limits<double>::max();
    int num = -1;
    for(int i = 0; i<waypoint_num; i++){
        double dist = (cur_state_.block<2,1>(0,0)-global_path_.block<2,1>(0,i)).norm();
        if(dist<min_dist){
            min_dist = dist;
            num = i;
        }
    }

    double last_yaw = cur_state_[2];
    for(int i=0; i<N_; i++){
        int index = (num+i>waypoint_num-1)? waypoint_num-1: num+i;
        goal_state_.col(i) = global_path_.col(index);
        if(i>0&i<N_-1){
            Eigen::VectorXd vec3 = (goal_state_.col(i)-goal_state_.col(i-1));
            double yaw = atan2(vec3(1), vec3(0));
            goal_state_(2,i-1) = vec3.norm()>0.01? yaw: last_yaw;
            last_yaw = goal_state_(2,i-1);
        }
    }
    goal_state_(2,N_-1) = last_yaw;
    // std::cout<< "the goal_station is: \n"<< goal_state_<< std::endl;
}

void MPC_PLANNER::smooth_yaw(Eigen::MatrixXd& ref_traj){
  double dyaw = ref_traj(2, 0) - cur_state_[2];

  while (dyaw >= M_PI / 2)
  {
    ref_traj(2, 0) -= M_PI * 2;
    dyaw = ref_traj(2, 0) - cur_state_[2];
  }
  while (dyaw <= -M_PI / 2)
  {
    ref_traj(2, 0) += M_PI * 2;
    dyaw = ref_traj(2, 0) - cur_state_[2];
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

void MPC_PLANNER::pub_Predict_traj(std::vector<double>& _predict_traj){
    nav_msgs::Path predict_traj;
    geometry_msgs::PoseStamped pose_msg;
    predict_traj.header.frame_id    = "world";
    predict_traj.header.stamp       = ros::Time::now();
    
    pose_msg.pose.orientation.w     = 1.0;
    pose_msg.pose.orientation.x     = 0.0;
    pose_msg.pose.orientation.y     = 0.0;
    pose_msg.pose.orientation.z     = 0.0;

    for(int i=0; i<_predict_traj.size()/2; i++){
        pose_msg.pose.position.x    = _predict_traj[2*i + 0];
        pose_msg.pose.position.y    = _predict_traj[2*i + 1];
        predict_traj.poses.push_back(pose_msg);
    }
    pub_local_path_.publish(predict_traj);

}

void MPC_PLANNER::rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
    std::lock_guard<std::mutex> curr_pose_lock_guard(curr_pose_mutex);
    Eigen::Quaterniond q( msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);    
    double yaw      = atan2(R.col(0)[1],R.col(0)[0]);
    double linear_v = msg->twist.twist.linear.z; 
    cur_state_      <<  msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        yaw, linear_v*cos(yaw), linear_v*sin(yaw);
    has_odom_ = true;
}

void MPC_PLANNER::rcvTrajCallBack(nav_msgs::PathPtr msg)
{
    std::lock_guard<std::mutex> global_path_lock_guard(global_path_mutex);
    int waypoint_num = msg->poses.size();
    global_path_.resize(3, waypoint_num);
    global_path_.setZero();
    for(int i=0; i<waypoint_num; i++){
        global_path_(0,i) = msg->poses[i].pose.position.x;
        global_path_(1,i) = msg->poses[i].pose.position.y;
    }    
    has_traj_ = true;
}

void MPC_PLANNER::rcvObsCallBack(std_msgs::Float32MultiArray msg)
{
    has_obs_ = true; 
    std::lock_guard<std::mutex> obstacle_lock_guard(obstacle_mutex);
    int obsp_num = msg.data.size()/(7);
    // obs_list_.clear();
    obs_matrix_.resize(7, obsp_num);
    obs_matrix_.setZero();

    for(int i=0; i<obsp_num; i++){
        Eigen::VectorXd obs_p(7);
        obs_p<< msg.data[7*i+0], msg.data[7*i+1], msg.data[7*i+2], msg.data[7*i+3], 
                msg.data[7*i+4], msg.data[7*i+5], msg.data[7*i+6];
        // obs_list_.push_back(obs_p);
        obs_matrix_.block<7, 1>(0, i) = obs_p;
    }
}


// -----------------------------------------------casadi-mpc-solver-----------------------------------------------
void MPC_SOLVE::init_solver(std::string _Sm, double& _Ts, int& _Ns, 
                            double _v_max, double _v_min, double _o_max, 
                            std::vector<double> _Q, std::vector<double> _R, 
                            double _gamma, double _safe_dist)
{
    Smetric_type= _Sm;
    Ts_s        = _Ts;
    N_s         = _Ns;
    v_max       = _v_max;
    v_min       = _v_min;
    omega_max   = _o_max;
    Q_p         = _Q;
    R_p         = _R;
    gamma_      = _gamma;
    safe_dist   = _safe_dist;

    kine_equation_ = setKinematicEquation();
    ROS_WARN("MPC_SOLVE initialized successfully! \nSafety-metric is: %s", Smetric_type.c_str());
}

bool MPC_SOLVE::imp_solve(Eigen::VectorXd* param1, Eigen::MatrixXd* param2, 
                            Eigen::MatrixXd* param3){
    cur_state_s     = param1;
    goal_state_s    = param2;
    obs_matrix_s   = param3;
    
    prob    = casadi::Opti();
    X_k     = prob.variable(5, N_s+1);
    U_k     = prob.variable(2, N_s);
    casadi::MX v        = U_k(0, casadi::Slice());
    casadi::MX omega    = U_k(1, casadi::Slice());
    casadi::MX cost = 0;
    
    casadi::MX X_ref = prob.parameter(3, N_s);
    std::vector<double> X_ref_v(goal_state_s->data(), goal_state_s->data()+goal_state_s->size());
    casadi::DM X_ref_d(X_ref_v);
    X_ref = casadi::MX::reshape(X_ref_d, 3, N_s);

    casadi::MX X_0  = prob.parameter(5);
    std::vector<double> X_0_value(cur_state_s->data(), cur_state_s->data()+cur_state_s->size());
    
    // 赋值障碍物数量
    obs_num = obs_matrix_s->cols()/N_s;

    // -------------------------------------MPC目标函数-固定
    // casadi::DM Q_ = casadi::DM::zeros(3,3);
    // Q_(0, 0) = Q_p[0]; 
    // Q_(1, 1) = Q_p[1]; 
    // Q_(2, 2) = Q_p[2];
    // casadi::DM R_ = casadi::DM::zeros(2,2);
    // R_(0, 0) = R_p[0]; 
    // R_(1, 1) = R_p[1];

    // for(int i=0; i<N_s; i++){
    //     casadi::MX X_err    = X_k(casadi::Slice(0, 3), i) - X_ref(casadi::Slice(), i);
    //     casadi::MX U_       = U_k(casadi::Slice(), i); 
    //     cost = cost + casadi::MX::mtimes({X_err.T(), Q_, X_err});
    //     cost = cost + casadi::MX::mtimes({U_.T(), R_, U_});
    // }

    // -------------------------------------MPC目标函数-递增
    casadi::DM R_ = casadi::DM::zeros(2,2);
    R_(0, 0) = R_p[0]; 
    R_(1, 1) = R_p[1];

    for(int i=0; i<N_s; i++){
        casadi::DM Q_ = casadi::DM::zeros(3,3);
        Q_(0, 0) = Q_p[0] + 0.05*i;
        Q_(1, 1) = Q_p[1] + 0.05*i;
        Q_(2, 2) = Q_p[2] + 0.005*i;
        casadi::MX X_err    = X_k(casadi::Slice(0, 3), i) - X_ref(casadi::Slice(), i);
        casadi::MX U_       = U_k(casadi::Slice(), i); 
        cost = cost + casadi::MX::mtimes({X_err.T(), Q_, X_err});
        cost = cost + casadi::MX::mtimes({U_.T(), R_, U_});
    }    

    prob.minimize(cost);
    
    // 运动学约束
    for(int i=0; i<N_s; i++){
        std::vector<casadi::MX> input(2);
        casadi::DM A = casadi::DM::zeros(5,5);
        for(int i=0; i<3; i++){
            A(i,i) = 1.0;
        }
        input[0] = X_k(casadi::Slice(), i);
        input[1] = U_k(casadi::Slice(), i);
        casadi::MX x_next = casadi::MX::mtimes(A, X_k(casadi::Slice(), i))+kine_equation_(input)[0];
        prob.subject_to(x_next == X_k(casadi::Slice(), i+1));
    }

    // 初始状态约束
    prob.subject_to(X_k(casadi::Slice(), 0) == X_0);

    prob.subject_to(prob.bounded(-v_max, v, v_max));
    prob.subject_to(prob.bounded(-omega_max, omega, omega_max));

    // 安全约束-障碍物规避
    int choose_num = 0;
    for(int i=0; i<obs_num; i++){
        bool exceed_obs = exceed_ob(obs_matrix_s->col(i*N_s));
        if(!exceed_obs & choose_num<2){
            std::cout<< "\033[34m add obstacle-num: \033[0m"<< i<< std::endl;
            choose_num ++;
            // 根据不同的安全约束进行casadi的约束设置
            for(int j=0; j<N_s; j++)
            {
                if (Smetric_type=="ACBF")
                {
                    // std::cout<< "------------------ acbf -------------------\n";
                    continue;
                }
                else if (Smetric_type=="DC")
                {
                    casadi::MX X_   = X_k(casadi::Slice(), j);
                    casadi::MX hk   = h1(X_,    obs_matrix_s->col(i*N_s));
                    casadi::MX cbf  = -hk;
                    prob.subject_to(cbf <= 0);
                }
                else if (Smetric_type=="SCBF")
                {
                    casadi::MX X_   = X_k(casadi::Slice(), j);
                    casadi::MX X_1  = X_k(casadi::Slice(), j+1);
                    casadi::MX hk   = h1(X_,    obs_matrix_s->col(i*N_s));
                    casadi::MX hk1  = h1(X_1,   obs_matrix_s->col(i*N_s));
                    casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
                    prob.subject_to(cbf <= 0);
                }
                else if (Smetric_type=="DCBF")
                {
                    casadi::MX X_   = X_k(casadi::Slice(), j);
                    casadi::MX X_1  = X_k(casadi::Slice(), j+1);
                    casadi::MX hk   = h1(X_,    obs_matrix_s->col(i*N_s+j));
                    casadi::MX hk1  = h1(X_1,   obs_matrix_s->col(i*N_s+j));
                    casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
                    prob.subject_to(cbf <= 0);
                }
                else{
                    continue;
                }
            }
        }
    }

    bool sign = false;
    //set solver
    casadi::Dict solver_opts;
    // solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 2000;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 2e-3;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 2e-3;
    prob.solver("ipopt", solver_opts);
    prob.set_value(X_0, X_0_value);

    try{
        // 初始求解
        solution_ = std::make_unique<casadi::OptiSol>(prob.solve());
        sign = true;
    }
    catch(const casadi::CasadiException& e) {
        std::cerr << "\033[31m Infeasible Solution: \033[0m" << e.what() << std::endl;    
        rotate_solution();
    }

    if(sign){
        // 初始求解成功,需要增加对当前解的判断——是否陷入死锁
        // std::cout<< "Preliminary solution successful"<< std::endl;
        std::vector<double> cur_cmd = getFirstUp();
        if(predict_u.size()!=0){
            if(std::fabs(cur_cmd[0])<0.01){
                // std::cout<< "\033[31m Low speed state \033[0m"<< std::endl;
                if(std::fabs(predict_u[0])<0.01 && (std::fabs(predict_u[2])>0.01 || std::fabs(predict_u[4])>0.01)){
                    std::cout<< "\033[31m Deadlock now \033[0m"<< std::endl;
                    rotate_solution();
                    sign = false;
                }
            }
        }
    }

    if(sign){
        predict_x.clear();
        predict_u.clear();
        predict_x = getPredictXp();
        predict_u = getFirstUp();
    }
    return sign;
}

void MPC_SOLVE::rotate_solution(){
    // 滚动赋值预测控制量
    std::rotate(predict_u.begin(), predict_u.begin()+2, predict_u.end());
    predict_u.back() = 0.0;
    predict_u[predict_u.size()-2] = 0.0;
    // 滚动复制预测状态量
    double last_x = predict_x[predict_x.size()-2];
    double last_y = predict_x[predict_x.size()-1];
    std::rotate(predict_x.begin(), predict_x.begin()+2, predict_x.end());
    predict_x[predict_x.size()-2] = last_x;
    predict_x[predict_x.size()-1] = last_y;   
}

bool MPC_SOLVE::exceed_ob(Eigen::VectorXd _obs_p){
    // 机器人前进视野的障碍物
    double ob_r = _obs_p[2]; 
    Eigen::Vector2d ob_vec      = _obs_p.block<2,1>(0,0);
    Eigen::Vector2d cent_vec    = cur_state_s->block<2,1>(0,0) - ob_vec;
    double d = cent_vec.norm();
    Eigen::Vector2d cross_pt    = ob_vec + 1.2*ob_r/d*cent_vec;
    Eigen::Vector2d vec1        = goal_state_s->block<2,1>(0,N_s-1) - cross_pt; 
    Eigen::Vector2d vec2        = cur_state_s->block<2,1>(0,0) - cross_pt;
    double vec1_2 = vec1.dot(vec2);
    return vec1_2 > 0;
}

std::vector<double> MPC_SOLVE::getFirstUp(){
    std::vector<double> res;
    casadi::native_DM control_list  = solution_->value(U_k);
    casadi::native_DM control_first = control_list(casadi::Slice(0, control_list.size1()), 
                                                    casadi::Slice(0, 1));
    // for(int i = 0; i < control_first.rows(); i++) {
    //   res.push_back(static_cast<double>(control_first(i, 0)));
    // }
    for(int i=0; i<N_s; i++){
        res.push_back(static_cast<double>(control_list(0,i)));
        res.push_back(static_cast<double>(control_list(1,i)));
    }
    return res;    
}

std::vector<double> MPC_SOLVE::getPredictXp(){
    std::vector<double> res;
    casadi::native_DM state_list = solution_->value(X_k);
    for(int i=0; i<N_s+1; i++){
        res.push_back(static_cast<double>(state_list(0, i)));
        res.push_back(static_cast<double>(state_list(1, i)));
    }
    return res;
}

casadi::Function MPC_SOLVE::setKinematicEquation() {
    casadi::MX x        = casadi::MX::sym("x");                // MPC 状态包括(x, y, theta)，x位置，y位置和theta航向角
    casadi::MX y        = casadi::MX::sym("y");
    casadi::MX theta    = casadi::MX::sym("theta");      
    casadi::MX vx       = casadi::MX::sym("vx");                // MPC 状态包括(x, y, theta)，x位置，y位置和theta航向角
    casadi::MX vy       = casadi::MX::sym("vy");  
    casadi::MX state_vars = casadi::MX::vertcat({x, y, theta, vx, vy});

    casadi::MX v = casadi::MX::sym("v");                // MPC 控制输入是(v, w)，速度和前轮转角
    casadi::MX w = casadi::MX::sym("w");
    casadi::MX control_vars = casadi::MX::vertcat({v, w});
    // 二轮差速小车模型
    casadi::MX rhs = casadi::MX::vertcat({v * casadi::MX::cos(theta) * Ts_s,
                                            v * casadi::MX::sin(theta) * Ts_s,
                                            w * Ts_s,
                                            v * casadi::MX::cos(theta),
                                            v * casadi::MX::sin(theta)});
    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}

casadi::MX MPC_SOLVE::h1(casadi::MX& _curpos, Eigen::VectorXd _obs){
    // casadi::MX _obs_k  = prob.parameter(7);
    // std::vector<double> obs_k_value(_obs.data(), _obs.data()+_obs.size());
    // prob.set_value(_obs_k, obs_k_value);
    casadi::MX dx = _obs(0) - _curpos(0);
    casadi::MX dy = _obs(1) - _curpos(1);
    casadi::MX h_exp = casadi::MX::sqrt(dx*dx + dy*dy) - _obs(2) - safe_dist;
    return h_exp;
}