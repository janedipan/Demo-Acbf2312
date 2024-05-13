#ifndef MPC_CBF_H
#define MPC_CBF_H

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

// 定义一个用于存储状态和输入的结构体
struct Solution {
    std::vector<std::vector<double>> states; // 多个状态，每个状态是一个向量
    std::vector<std::vector<double>> inputs; // 多个输入，每个输入是一个向量
};


// MPC_SOLVE需要初始化的数据：
class MPC_SOLVE {
public:
    MPC_SOLVE(){};
    ~MPC_SOLVE(){};
    void init_solver(std::string _Sm, double& _Ts, int& _Ns, 
                        double _v_max, double _v_min, double _o_max, 
                        std::vector<double> _Q, std::vector<double> _R, 
                        double _gamma, double _safe_dist);
    bool imp_solve(Eigen::VectorXd* param1, Eigen::MatrixXd* param2, Eigen::MatrixXd* param3);
    std::vector<double> predict_x, predict_u;
    std::vector<double> getFirstUp();
    std::vector<double> getPredictXp();

private:
    std::string Mpc_type;       // 运动学模型
    std::string Smetric_type;   // 安全指标
    double Ts_s;                // 离散时间
    int N_s;                    // 预测步数
    double v_max;               // 最大线速度
    double v_min;               // 最小线速度
    double omega_max;           // 最大角速度
    double gamma_;              // for CBF
    double safe_dist;           // 安全距离=机器人半径+膨胀半径
    std::vector<double> Q_p;    // Q矩阵对角线元素
    std::vector<double> R_p;    // R矩阵对角线元素
    // casadi::DM Q_;              // Q矩阵
    // casadi::DM R_;              // R矩阵

    casadi::Opti prob;
    casadi::MX X_k;
    casadi::MX U_k;

    casadi::Function setKinematicEquation();
    casadi::Function kine_equation_;
    std::unique_ptr<casadi::OptiSol> solution_;     // casadi求解器接口
    

    Eigen::VectorXd* cur_state_s;
    Eigen::MatrixXd* goal_state_s;
    Eigen::MatrixXd* obs_matrix_s;
    int obs_num;                                    // 障碍物的数量
    bool exceed_ob(Eigen::VectorXd _obs_p);         // 障碍物筛选函数
    void rotate_solution();                         // 求解失败时滚动操作
    casadi::MX h1(casadi::MX& _curpos, Eigen::VectorXd _obs);
};


class MPC_PLANNER {
public:
    MPC_PLANNER(){};
    ~MPC_PLANNER(){};
    void init_MPC_CBF(ros::NodeHandle& nh);
private:
    std::string Smetric_type;
    int controller;
    double replan_period_;
    double Ts_;
    int N_;

    Eigen::VectorXd cur_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    std::vector<Eigen::Vector2d> last_input_;
    std::vector<Eigen::VectorXd> last_state_;
    std::vector<Eigen::VectorXd> obs_list_;
    Eigen::MatrixXd obs_matrix_;

    std::mutex curr_pose_mutex;                 // 目前机器人位置互斥量
    std::mutex global_path_mutex;               // 全局路径信息互斥量
    std::mutex obstacle_mutex;                  // 障碍物信息互斥量
    
    bool has_odom_, has_traj_, has_obs_;
    int nums_of_planning;
    double cost_time_sum;

    MPC_SOLVE solver;
    // ros接口
    ros::NodeHandle nh_;
    ros::Timer timer_replan_, timer_pub;
    ros::Subscriber sub_curr_state_, sub_obs_, sub_goal_;
    ros::Publisher pub_local_path_vis_, pub_local_path_, pub_local_plan_;
    // ros::Publisher pub_start_;
    geometry_msgs::Twist cmd_vel;               // 输出cmd_vel消息给小车
    std_msgs::Float32MultiArray ref_traj_msg;   // 接收的参考轨迹消息
    
    // ros回调函数
    void replanCallback(const ros::TimerEvent &e);
    void cmdCallback(const ros::TimerEvent &e);
    void rcvOdomCallBack(nav_msgs::OdometryPtr msg);  
    void rcvTrajCallBack(nav_msgs::PathPtr msg);
    void rcvObsCallBack(std_msgs::Float32MultiArray msg);

    void choose_goal_state();
    void smooth_yaw(Eigen::MatrixXd& ref_traj);
    void pub_Predict_traj(std::vector<double>& _predict_traj);

};


#endif