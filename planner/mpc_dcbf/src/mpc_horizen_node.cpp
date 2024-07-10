#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

std::vector<std::string> controller_ls = {
    "None",
    "DC",
    "SCBF",
    "DCBF",
    "ACBF"
};

int controller;
std::string Smetric_type;
double replan_period_;
double Ts_;
int N_;
bool use_initguess, show_obs;
int N_i;
double v_max, omega_max, safe_dist;
double gamma_, tau_scale_;

ros::Subscriber sub_curr_state_ ;
ros::Publisher pub_local_path_, obsTraj_pub_;
ros::Timer timer_replan_, timer_vis_;

// 过程变量
bool has_odom_, has_obs_, success;

Eigen::VectorXd cur_state_, goal_state_, obs_init;
Eigen::MatrixXd obs_matrix_;
casadi::MX X_k, U_k, lambda_;
casadi::MX tau_list;

// 结果存储
casadi::native_DM state_list;


void setObstacle(bool isdynamic){
    obs_matrix_.resize(7, N_);
    obs_matrix_.setZero();
    if(!isdynamic){
        Eigen::VectorXd obs_init_1 = obs_init;
        obs_init_1(5) = 0.0;
        obs_init_1(6) = 0.0;
        for(int i=0; i<obs_matrix_.cols(); i++){
            obs_matrix_.col(i) = obs_init_1;
        }
        ROS_WARN("setting static obstacle");
        has_obs_ = true;
    }
    else{
        // 有关矩阵乘法了
        Eigen::VectorXd obs_init_1 = obs_init;
        for(int i=0; i<obs_matrix_.cols(); i++){
            obs_init_1[0] += Ts_*obs_init[5];
            obs_init_1[1] += Ts_*obs_init[6];
            obs_matrix_.col(i) = obs_init_1;
        }
        ROS_WARN("setting dynamic obstacle");
        has_obs_ = true;
    }
}

void show_obsTraj(){
    if(!has_obs_) return;
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
    obs_ball.pose.orientation.w = 1.0;
    for(int i=0; i<N_*0.4; i +=5){
        geometry_msgs::Point p;
        p.x = obs_matrix_(0, i);
        p.y = obs_matrix_(1, i);
        p.z = 0.25;
        obs_ball.points.push_back(p);
        obs_ball.id++; 
        obs_ball.scale.x = obs_matrix_(2, i)*2;
        obs_ball.scale.y = obs_matrix_(2, i)*2;
        obs_ball.scale.z = obs_matrix_(2, i)*2;
        obs_balls_msg.markers.push_back(obs_ball);
    }
    obsTraj_pub_.publish(obs_balls_msg);
}

void show_mpcPath(){
    if(!success) return;
    nav_msgs::Path predict_traj;
    geometry_msgs::PoseStamped pose_msg; 
    predict_traj.header.frame_id    = "world";
    predict_traj.header.stamp       = ros::Time::now(); 
    pose_msg.pose.orientation.w     = 1.0;
    pose_msg.pose.orientation.x     = 0.0;
    pose_msg.pose.orientation.y     = 0.0;
    pose_msg.pose.orientation.z     = 0.0;
    for(int i=0; i<N_; i++){
        pose_msg.pose.position.x = static_cast<double>(state_list(0, i));
        pose_msg.pose.position.y = static_cast<double>(state_list(1, i));
        predict_traj.poses.push_back(pose_msg);
    }
    pub_local_path_.publish(predict_traj);
}

void rcvOdomCallBack(const nav_msgs::Odometry& msg){
    Eigen::Quaterniond q( msg.pose.pose.orientation.w,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    double yaw      = atan2(R.col(0)[1],R.col(0)[0]);
    double linear_v = msg.twist.twist.linear.x; 
    cur_state_<< msg.pose.pose.position.x, 
                msg.pose.pose.position.y, 
                msg.pose.pose.position.z, yaw, 
                linear_v*cos(yaw), linear_v*sin(yaw);
    has_odom_ = true;

    if(show_obs) show_obsTraj();
    return;
}

// mpc内部函数

// 差速运动模型
casadi::Function setKinematicEquation() {
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
    casadi::MX rhs = casadi::MX::vertcat({v * casadi::MX::cos(theta) * Ts_,
                                            v * casadi::MX::sin(theta) * Ts_,
                                            w * Ts_,
                                            v * casadi::MX::cos(theta),
                                            v * casadi::MX::sin(theta)});
    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}

casadi::MX set_tau(casadi::MX& _curpos, Eigen::VectorXd _obs, casadi::MX& _vr){
    casadi::MX tau_;
    double obs_r = _obs(2);
    std::vector<double> obs_p_v(_obs.data(), _obs.data()+2);
    casadi::DM d_obs_p(obs_p_v);
    casadi::MX obs_p = casadi::MX::reshape(d_obs_p, 2, 1);
    casadi::MX ro_p     = _curpos(casadi::Slice(0, 2),0);

    casadi::MX pr   = obs_p - ro_p;
    casadi::MX vr   = _vr;
    casadi::MX tau_max = (casadi::MX::norm_2(pr)-obs_r)*
                            casadi::MX::norm_2(pr)/casadi::MX::dot(pr,vr);
    // 判断tau值符号
    casadi::MX tau_0 = casadi::MX::if_else(casadi::MX::dot(pr,vr)<0.0, -tau_max, 0.0);
    // tau值取上界
    casadi::MX tau_u = casadi::MX::if_else(tau_0<2.0, tau_0, 2.0);
    // tau值
    casadi::MX tau_d = casadi::MX::if_else(casadi::MX::norm_2(pr)-obs_r>0.1*1.3, tau_u, 0);
    return tau_d;
}

casadi::MX h1(casadi::MX& _curpos, Eigen::VectorXd _obs){
    casadi::MX dx = _obs(0) - _curpos(0);
    casadi::MX dy = _obs(1) - _curpos(1);
    casadi::MX h_exp = casadi::MX::sqrt(dx*dx + dy*dy) - _obs(2) - safe_dist;
    return h_exp;
}

casadi::MX h2(casadi::MX& _curpos, Eigen::VectorXd _obs, casadi::MX& _vr, casadi::MX& _tau){
    casadi::MX dx = _obs(0) - _curpos(0);
    casadi::MX dy = _obs(1) - _curpos(1);
    casadi::MX xv = _vr(0);    
    casadi::MX yv = _vr(1);
    casadi::MX h_exp = casadi::MX::sqrt((dx+xv*_tau)*(dx+xv*_tau)+(dy+yv*_tau)*(dy+yv*_tau))-
                            _obs(2) - safe_dist;
    return h_exp;    
}

void set_safety_st(int& smetric, casadi::Opti& opt){
    if(smetric == 4){
        for(int i=0; i<N_-1; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1); 
            casadi::MX ro_V = X_(casadi::Slice(3,5), 0);
            casadi::MX ro_V1= X_1(casadi::Slice(3,5), 0); 

            Eigen::VectorXd _obs_vec(obs_matrix_.col(i));    
            std::vector<double> obs_v_v(_obs_vec.data()+5, _obs_vec.data()+_obs_vec.size());        
            casadi::MX obs_v = opt.parameter(2, 1);
            opt.set_value(obs_v, obs_v_v);

            Eigen::VectorXd _obs_vec1(obs_matrix_.col(i+1)); 
            std::vector<double> obs_v1_v(_obs_vec1.data()+5, _obs_vec1.data()+_obs_vec1.size());        
            casadi::MX obs_v1 = opt.parameter(2, 1);
            opt.set_value(obs_v1, obs_v1_v);

            casadi::MX vr   = obs_v - ro_V;
            casadi::MX vr1  = obs_v1 - ro_V1;
            casadi::MX tau0 = set_tau(X_, obs_matrix_.col(i), vr);
            casadi::MX tau1 = set_tau(X_1, obs_matrix_.col(i+1), vr1); 
            tau_list(0, i) = tau0;
            double scale = tau_scale_;
            int nums = N_*0.8;  // 0.3*100
            if(i>=nums) scale = 0.0;
            casadi::MX tau0_u = scale*tau0;
            casadi::MX tau1_u = scale*tau1;
            casadi::MX hk   = h2(X_,    _obs_vec, vr, tau0_u);
            casadi::MX hk1  = h2(X_1,   _obs_vec1, vr1, tau1_u);
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk-lambda_(0,i);
            // casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf <= 0);
        }
    } // ACBF
    else if(smetric == 3){
        for(int i=0; i<N_-1; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1);  
            casadi::MX hk   = h1(X_, obs_matrix_.col(i));          
            casadi::MX hk1  = h1(X_1, obs_matrix_.col(i+1));   
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf<=0);    
        }
    } // DCBF
    else if(smetric == 2){
        for(int i=0; i<N_; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX X_1  = X_k(casadi::Slice(), i+1); 
            casadi::MX hk   = h1(X_, obs_matrix_.col(0));          
            casadi::MX hk1  = h1(X_1, obs_matrix_.col(0));   
            casadi::MX cbf  = -hk1 + (1-gamma_)*hk;
            opt.subject_to(cbf<=0);    
        }
    } // CBF
    else if(smetric == 1){
        for(int i=2; i<N_; i++){
            casadi::MX X_   = X_k(casadi::Slice(), i);
            casadi::MX hk   = h1(X_, obs_matrix_.col(i-2)); 
            casadi::MX cbf  = -hk;
            opt.subject_to(cbf<=0);
        }
    } // DC
    else{
        return;
    } // None
    
}

// 参数分别为安全约束类型，当前位置，目标位置，障碍物信息
bool imp_solve(int& smetric, Eigen::VectorXd& param1, Eigen::VectorXd& param2, Eigen::MatrixXd& param3){
    casadi::Opti opt;
    X_k = opt.variable(5, N_+1);
    U_k = opt.variable(2, N_);
    lambda_ = opt.variable(1, N_);

    casadi::MX cost = 0;

    casadi::MX v        = U_k(0, casadi::Slice());
    casadi::MX omega    = U_k(1, casadi::Slice());
    tau_list = casadi::MX::zeros(1, N_);

    // // 数值传递
    casadi::MX X_0  = opt.parameter(5);
    std::vector<double> X_0_value(param1.data(), param1.data()+param1.size());
    opt.set_value(X_0, X_0_value);

    casadi::MX X_ref = opt.parameter(3);
    std::vector<double> X_ref_v(param2.data(), param2.data()+param2.size());
    opt.set_value(X_ref, X_ref_v);

    // 目标函数-权重矩阵Q,R固定
    casadi::DM Q_ = casadi::DM::zeros(3,3);
    Q_(0,0) = 1.5;
    Q_(1,1) = 1.5;
    Q_(2,2) = 0.15;
    casadi::DM R_ = casadi::DM::zeros(2,2);
    R_(0,0) = 0.1;
    R_(1,1) = 0.05;
    for(int i=0; i<N_; i++){
        casadi::MX X_err    = X_k(casadi::Slice(0, 3), i) - X_ref;
        casadi::MX U_       = U_k(casadi::Slice(), i); 
        cost = cost + casadi::MX::mtimes({X_err.T(), Q_, X_err});
        cost = cost + casadi::MX::mtimes({U_.T(), R_, U_});
        // 添加cbf-soft软约束
        if(smetric==4) cost = cost + lambda_(0, i)*lambda_(0, i)*8000;
    }
    casadi::MX X_err_e = X_k(casadi::Slice(0,3), N_) - X_ref;
    cost += casadi::MX::mtimes({X_err_e.T(), 1.1*Q_, X_err_e});
    opt.minimize(cost);

    // // 初始状态约束
    opt.subject_to(X_k(casadi::Slice(), 0)==X_0);

    // // 控制量约束
    opt.subject_to(opt.bounded(-v_max, v, v_max));
    opt.subject_to(opt.bounded(-omega_max, omega, omega_max));
    opt.subject_to(opt.bounded(0.0, lambda_, 0.3));

    // // 运动学约束
    casadi::Function kine_equation_ = setKinematicEquation();
    for(int i=0; i<N_; i++){
        std::vector<casadi::MX> input(2);
        casadi::DM A = casadi::DM::zeros(5,5);
        for(int i=0; i<3; i++) A(i,i) = 1.0;
        input[0] = X_k(casadi::Slice(), i);
        input[1] = U_k(casadi::Slice(), i);  
        casadi::MX x_next = casadi::MX::mtimes(A, X_k(casadi::Slice(), i))+kine_equation_(input)[0];
        opt.subject_to(x_next == X_k(casadi::Slice(), i+1));      
    }

    // // 障碍物约束
    set_safety_st(smetric, opt);

    // // 采用初始参考?

    // //求解器设置
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 5000;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 3e-3;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 3e-3;
    opt.solver("ipopt", solver_opts);

    try{
        auto solution_ = opt.solve();
        // 数值提取
        state_list = solution_.value(X_k);
        if(smetric==4) std::cout<< "tau_value =\n"<<solution_.value(tau_list)<< "\nlambda_value =\n"<< solution_.value(lambda_)(0, casadi::Slice(0, 30))<< std::endl;;
        return true;
    }
    catch(const casadi::CasadiException& e) {
        std::cerr << "\033[31m Infeasible Solution: \033[0m" << e.what() << std::endl;
        return false;
    }
}

void timeCallback1(const ros::TimerEvent &e){
    
    if(!has_odom_ || !has_obs_){
        ROS_WARN("no odometry || no obstacle");
        return;
    }
    ros::Time time_in0 = ros::Time::now();
    
    success = imp_solve(controller ,cur_state_, goal_state_, obs_matrix_);
    
    ros::Time time_in1 = ros::Time::now();
    std::cout<< "mpc_metric_type:"<<Smetric_type <<"\033[38;2;255;128;0m replan_time =: \033[0m"<< (time_in1-time_in0).toSec()*1000<< "ms"<< std::endl;
    std::cout<< "------------------------------------------------------------------\n";
    return;
}

void timeCallback2(const ros::TimerEvent &e){
    // if(show_obs) show_obsTraj();
    show_mpcPath();
    return;
}



int main(int argc, char* argv[]){
    ros::init(argc, argv, "MPC_horizen_node");
    ros::NodeHandle nh("~");
    nh.param("mpc/controller_type", controller, 0);
    Smetric_type = controller_ls[controller];
    nh.param("mpc/mpc_frequency",   replan_period_, 5.0);
    nh.param("mpc/step_time",       Ts_, 0.1);
    nh.param("mpc/pre_step",        N_, 25);
    nh.param("mpc/gamma",        gamma_, 0.0);
    nh.param("mpc/tau_scale",   tau_scale_, 0.0);
    nh.param("mpc/use_initiguess", use_initguess, false);
    nh.param("mpc/show_obs", show_obs, false);
    N_i = N_+2;
    has_odom_ = false;
    has_obs_ = false;
    v_max = 1.3;
    omega_max = 1.0;
    safe_dist = 0.1;

    cur_state_.resize(5);
    cur_state_.setZero();
    goal_state_.resize(3);
    goal_state_<< 8.0, 0.0, 0.0;
    obs_init.resize(7);
    obs_init<< 8.0, -0.50, 1.0, 1.0, 0.0, -1.5, 0.0;
    // 设置障碍物
    setObstacle(true);

    sub_curr_state_ = nh.subscribe("/Odometry", 10, rcvOdomCallBack);
    pub_local_path_ = nh.advertise<nav_msgs::Path>("/local_path", 10);
    obsTraj_pub_    = nh.advertise<visualization_msgs::MarkerArray>("obs_traj_vis", 10);
    timer_replan_   = nh.createTimer(ros::Duration(1/replan_period_), timeCallback1);
    timer_vis_      = nh.createTimer(ros::Duration(0.1), timeCallback2);

    ROS_WARN("MPC_Horizen_noe initialized successfully! \nSafety-metric is: %s", Smetric_type.c_str());
    ROS_WARN("MPC predictive range is: %f, step time is: %f, gamma is: %f, tau_scale is: %f",N_*Ts_, Ts_, gamma_, tau_scale_);
    if(use_initguess) ROS_WARN("mpc use init_guess");

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}