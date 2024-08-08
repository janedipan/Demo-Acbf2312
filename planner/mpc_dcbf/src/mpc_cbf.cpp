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
    nh_.param("mpc/controller_type", controller, 0);
    Smetric_type = controller_ls[controller];
    nh_.param("mpc/mpc_frequency",   replan_period_, 5.0);
    nh_.param("mpc/step_time",       Ts_, 0.1);
    nh_.param("mpc/pre_step",        N_, 25);
    nh_.param("mpc/gamma",        gamma_, 0.0);
    nh_.param("mpc/tau_scale",   tau_scale_, 0.0);
    nh_.param("mpc/use_initiguess", use_initguess, false);
    nh_.param("mpc/ahead", use_ahead, false);
    if(use_ahead) ROS_WARN("mpc use ahead_horizen");
    if(use_ahead){
        N_i = N_+2;
    } 
    else {
        N_i = N_;
    }
    cur_state_.resize(5);
    cur_state_.setZero();
    goal_state_.resize(3, N_i);
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
    double o_max = 1.0;
    // for mpc-Q 递增
    std::vector<double> Q = {1.0, 1.0, 0.05}; 
    std::vector<double> R = {0.1, 0.05};    
    // for mpc-Q 固定
    // std::vector<double> Q = {1.5, 1.5, 0.15}; 
    // std::vector<double> R = {0.1, 0.05};    

    // PS:障碍物被参数化表示为半径为0.4m的圆形
    // double gamma    = 0.3;
    double safe_dist= 0.3+0.4;  // 安全距离+机器人半径

    // 数据记录
    time_list1.clear();
    

    solver.init_solver(Smetric_type, Ts_, N_, v_max, v_min, o_max, Q, R, gamma_, tau_scale_, safe_dist, use_initguess);

    timer_replan_   = nh_.createTimer(ros::Duration(1/replan_period_), &MPC_PLANNER::replanCallback, this);
    timer_pub       = nh_.createTimer(ros::Duration(0.01), &MPC_PLANNER::cmdCallback, this);
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
        ros::Time time_in0 = ros::Time::now();
        bool success = solver.imp_solve(&cur_state_, &goal_state_, &obs_matrix_);
        // if(success){
        //     // ROS_INFO("Solved successfullly!");
        // }
        // else{
        //     ROS_WARN("Solved failed!");
        //     // cmd_vel.linear.x    = 0.0;
        //     // cmd_vel.angular.z   = 0.0;           
        // }
        cmd_vel.linear.x    = solver.predict_u[0];
        cmd_vel.angular.z   = solver.predict_u[1];
        // std::cout<< "v:= "<<solver.predict_u[0]<< "; w:= "<<solver.predict_u[1]<< std::endl; 
        pub_Predict_traj(solver.predict_x);

        ros::Time time_in1 = ros::Time::now();
        double cost_time = (time_in1-time_in0).toSec()*1000;
        time_list1.push_back(cost_time);
        std::cout<< "\033[38;2;255;128;0m MPC replan_time =: \033[0m"<< (time_in1-time_in0).toSec()*1000<< "ms"<< std::endl;
        // std::cout<< "------------------------------------------------------------------\n";
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
    for(int i=0; i<N_i; i++){
        int index = (num+i>waypoint_num-1)? waypoint_num-1: num+i;
        goal_state_.col(i) = global_path_.col(index);
        if(i>0&i<N_i-1){
            Eigen::VectorXd vec3 = (goal_state_.col(i)-goal_state_.col(i-1));
            double yaw = atan2(vec3(1), vec3(0));
            goal_state_(2,i-1) = vec3.norm()>0.01? yaw: last_yaw;
            last_yaw = goal_state_(2,i-1);
        }
    }
    goal_state_(2,N_i-1) = last_yaw;
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

    for(int i=0; i<_predict_traj.size()/5; i++){
        pose_msg.pose.position.x    = _predict_traj[5*i + 0];
        pose_msg.pose.position.y    = _predict_traj[5*i + 1];
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
    double linear_v = msg->twist.twist.linear.x; 
    cur_state_      <<  msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        yaw, linear_v*cos(yaw), linear_v*sin(yaw);
    // std::cout<< "robot linear velocity is: "<< linear_v<< std::endl;
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
                            double _gamma, double _tau_scale,  double _safe_dist, bool sign)
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
    tau_scale_  = _tau_scale;
    safe_dist   = _safe_dist;
    use_initguess_s = sign;
    exit_obs1.clear();

    kine_equation_ = setKinematicEquation();
    ROS_WARN("MPC_Horizen_noe initialized successfully! \nSafety-metric is: %s", Smetric_type.c_str());
    ROS_WARN("MPC predictive range is: %f, step time is: %f, gamma is: %f, tau_scale is: %f",N_s*Ts_s, Ts_s, gamma_, tau_scale_);
    if(sign){
        ROS_WARN("mpc use init_guess");
    }
}

void MPC_SOLVE::set_safety_st(std::string& smetric, casadi::Opti& opt, int index){
    if(smetric == "ACBF"){
        double tau_value = set_tau_value(*cur_state_s, obs_matrix_s->col(index*N_s+0));
        Eigen::Vector2d vr = obs_matrix_s->col(index*N_s+0).block<2,1>(5,7) - cur_state_s->block<2,1>(3,5);
        for(int i=0; i<N_s-1; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1); 

            Eigen::VectorXd _obs_vec(obs_matrix_s->col(index*N_s+i)); 
            Eigen::VectorXd _obs_vec1(obs_matrix_s->col(index*N_s+i+1));  

            // std::vector<double> obs_v_v(_obs_vec.data()+5, _obs_vec.data()+_obs_vec.size());        
            // casadi::MX obs_v = opt.parameter(2, 1);
            // opt.set_value(obs_v, obs_v_v);
            // std::vector<double> obs_v1_v(_obs_vec1.data()+5, _obs_vec1.data()+_obs_vec1.size());        
            // casadi::MX obs_v1 = opt.parameter(2, 1);
            // opt.set_value(obs_v1, obs_v1_v);

            casadi::MX tau0 = set_tau(X_, obs_matrix_s->col(index*N_s+i));
            casadi::MX tau1 = set_tau(X_1, obs_matrix_s->col(index*N_s+i+1)); 
            // double scale = tau_scale_;
            // int nums = 5;  // 0.3*N_s
            // if(i>nums) scale = 0.0;
            // casadi::MX tau0_u = scale*tau0;
            // casadi::MX tau1_u = scale*tau1;
            
            // casadi::MX hk   = h2(X_,    _obs_vec, tau0_u);
            // casadi::MX hk1  = h2(X_1,   _obs_vec1, tau1_u);

            double scale = tau_scale_;
            if(i>6) scale = 0.0;
            double tau0_u = tau_value*scale;
            tau_list(index, i) = tau0_u;
            casadi::MX hk = h3(X_, _obs_vec, vr, tau0_u);
            casadi::MX hk1 = h3(X_1, _obs_vec1, vr, tau0_u);

            // casadi::MX cbf  = -hk1 + (1-gamma_)*hk-lambda_(index, i);
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf <= 0);
        }
    } // ACBF
    else if(smetric == "DCBF"){
        for(int i=0; i<N_s-1; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1);  
            casadi::MX hk   = h1(X_, obs_matrix_s->col(index*N_s+i));          
            casadi::MX hk1  = h1(X_1, obs_matrix_s->col(index*N_s+i+1));   
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf<=0);    
        }
    } // DCBF
    else if(smetric == "SCBF"){
        for(int i=0; i<N_s; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1); 
            casadi::MX hk   = h1(X_, obs_matrix_s->col(index*N_s+i));          
            casadi::MX hk1  = h1(X_1, obs_matrix_s->col(index*N_s+i));   
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf<=0);    
        }
    } // CBF
    else if(smetric == "DC"){
        for(int i=0; i<N_s; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX hk   = h1(X_, obs_matrix_s->col(index*N_s+i)); 
            casadi::MX cbf  = -hk;
            opt.subject_to(cbf<=0);
        }
    } // DC
    else{
        return;
    } // None    
}

bool MPC_SOLVE::imp_solve(Eigen::VectorXd* param1, Eigen::MatrixXd* param2, 
                            Eigen::MatrixXd* param3){
    cur_state_s     = param1;
    goal_state_s    = param2;
    // 赋值障碍物数量
    obs_matrix_s    = param3;
    obs_num = obs_matrix_s->cols()/N_s;
    
    prob    = casadi::Opti();
    X_k     = prob.variable(5, N_s+1);
    U_k     = prob.variable(2, N_s);
    lambda_ = prob.variable(obs_num, N_s);
    tau_list = casadi::MX::zeros(obs_num, N_s);
    casadi::MX v        = U_k(0, casadi::Slice());
    casadi::MX omega    = U_k(1, casadi::Slice());
    casadi::MX cost = 0;
    
    // 数值传递
    casadi::MX X_ref = prob.parameter(3, N_s);
    std::vector<double> X_ref_v(goal_state_s->data(), goal_state_s->data()+3*N_s);
    casadi::DM X_ref_d(X_ref_v);
    X_ref = casadi::MX::reshape(X_ref_d, 3, N_s);

    casadi::MX X_0  = prob.parameter(5);
    std::vector<double> X_0_value(cur_state_s->data(), cur_state_s->data()+cur_state_s->size());
    prob.set_value(X_0, X_0_value);

    // MPC目标函数-固定
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

    // MPC目标函数-递增
    casadi::DM R_ = casadi::DM::zeros(2,2);
    R_(0, 0) = R_p[0]; 
    R_(1, 1) = R_p[1];
    casadi::DM Q_ = casadi::DM::zeros(3,3);
    Q_(0, 0) = Q_p[0];
    Q_(1, 1) = Q_p[1];
    Q_(2, 2) = Q_p[2];
    for(int i=0; i<N_s; i++){
        casadi::MX X_err    = X_k(casadi::Slice(0, 3), i) - X_ref(casadi::Slice(), i);
        casadi::MX U_       = U_k(casadi::Slice(), i); 
        cost = cost + casadi::MX::mtimes({X_err.T(), Q_, X_err});
        cost = cost + casadi::MX::mtimes({U_.T(), R_, U_});
        Q_(0, 0) += 0.05;
        Q_(1, 1) += 0.05;
        Q_(2, 2) += 0.005;
        // 添加cbf-soft软约束
        if(Smetric_type=="ACBF"){
            for(int j=0; j<obs_num; j++) cost = cost + lambda_(j, i)*lambda_(j, i)*8000;
        }
    }    
    casadi::MX X_err_e = X_k(casadi::Slice(0, 3), N_s) - X_ref(casadi::Slice(), N_s-1);
    cost += casadi::MX::mtimes({X_err_e.T(), 1.1*Q_, X_err_e});
    prob.minimize(cost);
    

    // 初始状态约束
    prob.subject_to(X_k(casadi::Slice(), 0) == X_0);
    // 控制量约束
    prob.subject_to(prob.bounded(-v_max, v, v_max));
    prob.subject_to(prob.bounded(-omega_max, omega, omega_max));
    prob.subject_to(prob.bounded(0.0, lambda_, 0.2));


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

    // 障碍物约束
    int choose_num = 0;
    exit_obs1.push_back(false);
    for(int i=0; i<obs_num; i++){
        bool exceed_obs = exceed_ob(obs_matrix_s->col(i*N_s));
        if(!exceed_obs && choose_num<2){
            exit_obs1.back() = true;
            double dis = (cur_state_s->block<2,1>(0,0)-obs_matrix_s->col(i*N_s).block<2,1>(0,0)).norm()-0.4-0.4;
            std::cout<< "\033[34m add obstacle-num: \033[0m"<< i<< " center_dis :="<< dis<<std::endl;
            set_safety_st(Smetric_type, prob, i);
            choose_num++;
        }
    }


    // 设置初始猜测值
    if(use_initguess_s){
        if(predict_u.size()!=0){
            std::vector<std::vector<double>> last_res = rotate_solution1(predict_x, predict_u);
            casadi::native_DM x_guess = casadi::DM::reshape(casadi::DM(last_res[0]), 5, N_s+1);
            casadi::native_DM u_guess = casadi::DM::reshape(casadi::DM(last_res[1]), 2, N_s);
            // std::cout<< "x_guess =: "<< x_guess<< std::endl;
            // std::cout<< "u_guess =: "<< u_guess<< std::endl;
            prob.set_initial(X_k, x_guess);
            prob.set_initial(U_k, u_guess);
        }
    }
    
    //求解器设置
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 2500;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 3e-3;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 3e-3;
    prob.solver("ipopt", solver_opts);
    
    try{
        // 初始求解
        solution_ = std::make_unique<casadi::OptiSol>(prob.solve());
        if(Smetric_type == "ACBF"){
            // std::cout<< "tau_value =\n"<<solution_->value(tau_list)<< "\nlambda_value =\n"<< solution_->value(lambda_)(casadi::Slice(), casadi::Slice(0, 20))<< std::endl;;
            // std::cout<< "tau_value =\n"<<solution_->value(tau_list)<< std::endl;
        }

        // 初始求解成功,需要增加对当前解的判断——是否陷入死锁
        if(predict_u.size()!=0){
            std::vector<double> cur_cmd = getFirstUp();
            if(std::fabs(predict_u[0])<0.1 && (std::fabs(predict_u[2])>0.1 && std::fabs(cur_cmd[0])<0.1)){
                std::cout<< "\033[31m Deadlock now \033[0m"<< std::endl;
                rotate_solution();
                return false;
            }
        }
        // 若无死锁 则执行正常流程
        predict_x.clear();
        predict_u.clear();
        predict_x = getPredictXp();
        predict_u = getFirstUp();
        // std::cout<< "opt_state is: "<<solution_->value(X_k)<< std::endl;
        // std::cout<< "opt_control is: "<<solution_->value(U_k)<< std::endl;
        return true;
    }
    catch(const casadi::CasadiException& e) {
        std::cerr << "\033[31m Infeasible Solution: \033[0m" << e.what() << std::endl;    
        rotate_solution();
        return false;
    }

    // if(sign){
    //     // 初始求解成功,需要增加对当前解的判断——是否陷入死锁
    //     std::vector<double> cur_cmd = getFirstUp();
    //     if(predict_u.size()!=0){
    //         if(std::fabs(cur_cmd[0])<0.1){
    //             // std::cout<< "\033[31m Low speed state \033[0m"<< std::endl;
    //             if(std::fabs(predict_u[0])<0.1 && (std::fabs(predict_u[2])>0.1)){
    //                 std::cout<< "\033[31m Deadlock now \033[0m"<< std::endl;
    //                 rotate_solution();
    //                 sign = false;
    //             }
    //         }
    //     }
    // }
}

casadi::MX MPC_SOLVE::h1(casadi::MX& _curpos, Eigen::VectorXd _obs){
    casadi::MX dx = _obs(0) - _curpos(0);
    casadi::MX dy = _obs(1) - _curpos(1);
    casadi::MX h_exp = casadi::MX::sqrt(dx*dx + dy*dy) - _obs(2) - safe_dist;
    return h_exp;
}

casadi::MX MPC_SOLVE::h2(casadi::MX& _curpos, Eigen::VectorXd _obs, casadi::MX& _tau){
    casadi::MX dx = _obs[0] - _curpos(0);
    casadi::MX dy = _obs[1] - _curpos(1);
    casadi::MX xv = _obs[5] - _curpos(3);   
    casadi::MX yv = _obs[6] - _curpos(4);   
    casadi::MX h_exp = casadi::MX::sqrt((dx+xv*_tau)*(dx+xv*_tau)+(dy+yv*_tau)*(dy+yv*_tau))-
                            _obs(2) - safe_dist;
    return h_exp;    
}

casadi::MX MPC_SOLVE::h3(casadi::MX& _curpos, Eigen::VectorXd _obs, Eigen::Vector2d _vr, double _tau){
    casadi::MX dx = _obs[0] - _curpos(0);
    casadi::MX dy = _obs[1] - _curpos(1);
    double xv = _vr[0];
    double yv = _vr[1];
    casadi::MX h_exp = casadi::MX::sqrt((dx+xv*_tau)*(dx+xv*_tau)+(dy+yv*_tau)*(dy+yv*_tau))-
                            _obs[2] - safe_dist;
    return h_exp;
}

casadi::MX MPC_SOLVE::set_tau(casadi::MX& _curpos, Eigen::VectorXd _obs){
    double obs_r = _obs[2];
    std::vector<double> obs_p_v(_obs.data(), _obs.data()+2);
    casadi::DM d_obs_p(obs_p_v);
    casadi::MX obs_p = casadi::MX::reshape(d_obs_p, 2, 1);

    std::vector<double> obs_v_v(_obs.data()+5, _obs.data()+7);
    casadi::DM d_obs_v(obs_v_v);
    casadi::MX obs_v = casadi::MX::reshape(d_obs_v, 2, 1);

    casadi::MX ro_p     = _curpos(casadi::Slice(0, 2),0);
    casadi::MX ro_v     = _curpos(casadi::Slice(3,5),0);

    casadi::MX pr   = obs_p - ro_p;
    casadi::MX vr   = obs_v - ro_v;
    casadi::MX tau_max = (casadi::MX::norm_2(pr)-obs_r-0.4*1.0)*
                            casadi::MX::norm_2(pr)/casadi::MX::dot(pr,vr);
    casadi::MX tau_0 = casadi::MX::if_else(casadi::MX::dot(pr,vr)<0.0, -tau_max, 0.0);
    casadi::MX tau_u = casadi::MX::if_else(tau_0<2.0, tau_0, 2.0);
    // casadi::MX tau_d = casadi::MX::if_else(tau_u>0.4, tau_u, 0.0);
    casadi::MX tau_d = casadi::MX::if_else(casadi::MX::norm_2(pr)-obs_r-safe_dist>0.1*1.3, tau_u, 0);
    return tau_d;
}

double MPC_SOLVE::set_tau_value(Eigen::VectorXd _rob, Eigen::VectorXd _obs){
    double obs_r = _obs[2];
    Eigen::Vector2d obs_p = _obs.block<2,1>(0,0);
    Eigen::Vector2d rob_p = _rob.block<2,1>(0,0);
    Eigen::Vector2d obs_v = _obs.block<2,1>(5,0);
    Eigen::Vector2d rob_v = _rob.block<2,1>(3,0);
    Eigen::Vector2d pr = obs_p - rob_p;
    Eigen::Vector2d vr = obs_v - rob_v;

    double tau_max = (pr.norm()-obs_r-0.4*1.0)*pr.norm()/pr.dot(vr);
    double tau_0 = pr.dot(vr)<0.0? -tau_max : 0.0;
    double tau_u = pr.norm()-obs_r-0.4<1.5? 0.0 : tau_0;
    return tau_u;
}

void MPC_SOLVE::rotate_solution(){
    // 滚动赋值预测控制量
    std::rotate(predict_u.begin(), predict_u.begin()+2, predict_u.end());
    predict_u.back() = 0.0;
    predict_u[predict_u.size()-2] = 0.0;
    // 滚动复制预测状态量
    double last_x = predict_x[predict_x.size()-5];
    double last_y = predict_x[predict_x.size()-4];
    double last_theta = predict_x[predict_x.size()-3];
    double last_xv = predict_x[predict_x.size()-2];
    double last_yv = predict_x[predict_x.size()-1];
    std::rotate(predict_x.begin(), predict_x.begin()+5, predict_x.end());
    predict_x[predict_x.size()-5] = last_x;
    predict_x[predict_x.size()-4] = last_y;   
    predict_x[predict_x.size()-3] = last_theta;   
    predict_x[predict_x.size()-2] = last_xv;   
    predict_x[predict_x.size()-1] = last_yv;   
}

std::vector<std::vector<double>> MPC_SOLVE::rotate_solution1(std::vector<double> x_arr, std::vector<double> u_arr){
    // 滚动赋值预测控制量
    std::vector<std::vector<double>> res(2);
    std::rotate(u_arr.begin(), u_arr.begin()+2, u_arr.end());
    u_arr.back() = 0.0;
    u_arr[u_arr.size()-2] = 0.0;
    // 滚动复制预测状态量
    double last_x = x_arr[x_arr.size()-5];
    double last_y = x_arr[x_arr.size()-4];
    double last_theta = x_arr[x_arr.size()-3];
    double last_xv = x_arr[x_arr.size()-2];
    double last_yv = x_arr[x_arr.size()-1];
    std::rotate(x_arr.begin(), x_arr.begin()+5, x_arr.end());
    x_arr[x_arr.size()-5] = last_x;
    x_arr[x_arr.size()-4] = last_y;  
    x_arr[x_arr.size()-3] = last_theta;  
    x_arr[x_arr.size()-2] = last_xv;  
    x_arr[x_arr.size()-1] = last_yv;  
    res[0] = x_arr;
    res[1] = u_arr; 
    return res; 
}


bool MPC_SOLVE::exceed_ob(Eigen::VectorXd _obs_p){
    // 机器人前进视野的障碍物
    double ob_r = _obs_p[2]; 
    Eigen::Vector2d ob_vec      = _obs_p.block<2,1>(0,0);
    Eigen::Vector2d cent_vec    = cur_state_s->block<2,1>(0,0) - ob_vec;
    double d = cent_vec.norm();
    Eigen::Vector2d cross_pt    = ob_vec + 1.0*ob_r/d*cent_vec;
    int goal_cols = goal_state_s->cols();
    Eigen::Vector2d vec1        = goal_state_s->block<2,1>(0,goal_cols-1) - cross_pt; 
    Eigen::Vector2d vec2        = cur_state_s->block<2,1>(0,0) - cross_pt;
    double vec1_2 = vec1.dot(vec2);
    return (vec1_2 > 0) || (vec2.norm()<0.4*0.85);
    // return vec2.norm() > 5 || vec2.norm()<0.3*1.05;
    // return (vec1_2 > 0);
}

std::vector<double> MPC_SOLVE::getFirstUp(){
    std::vector<double> res;
    casadi::native_DM control_list  = solution_->value(U_k);
    // casadi::native_DM control_first = control_list(casadi::Slice(0, control_list.size1()), 
    //                                                 casadi::Slice(0, 1));
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
        res.push_back(static_cast<double>(state_list(2, i)));
        res.push_back(static_cast<double>(state_list(3, i)));
        res.push_back(static_cast<double>(state_list(4, i)));
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

