#ifndef MPC_CORE_H
#define MPC_CORE_H

#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <iostream>


class MPC_base
{
public:
  int N_;         // 预测步长
  double dt_;     // 离散时间
  double L_;      // 轴距( 仅当阿克曼小车时使用 )
  double ux_max_, ux_min_;      // 控制输入: x轴线速度的最大最小值
  double uy_max_, uy_min_;      // 控制输入: y轴线速度
  double w_max_, w_min_;        // 控制输入2的最大最小值
  
  casadi::DM Q_, R_;  // 权重矩阵
  casadi::MX X, U;    // 系统状态和控制输入
  
  casadi::Function kinematic_equation_;         // mpc的状态转移方程: 二自由度自行车模型
  std::unique_ptr<casadi::OptiSol> solution_;   // casadi求解器接口
    
public:
  // 空的构造函数
  MPC_base(){};

  // 虚析构函数
  virtual ~MPC_base(){};

  // 待实现的纯虚函数方法
  virtual casadi::Function setKinematicEquation() = 0;      // 设置系统状态方程
  virtual bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) = 0;   // 求解MPC最优控制

  // 已实现的父类方法
  virtual void init(int _N,
                    double _dt, 
                    double _u_max, 
                    double _u_min,
                    double _u_y_max, 
                    double _delta_max,
                    double _L,
                    std::vector<double> _Q,
                    std::vector<double> _R)
  {
    N_ = _N;
    L_ = _L;
    dt_ = _dt;
    ux_max_ = _u_max;
    ux_min_ = _u_min;
    uy_max_ = _u_y_max;
    uy_min_ = -_u_y_max;
    w_max_ = _delta_max;
    w_min_ = -_delta_max;

    if(_Q.size() != 3){
      ROS_WARN("WARNING: matrix_q size should be equal to 3, please check the config/param.yaml !");
    }
    if(_R.size() != 2){
      ROS_WARN("WARNING: matrix_r size should be equal to 2, please check the config/param.yaml !");
    }

    Q_ = casadi::DM::zeros(3,3);
    R_ = casadi::DM::zeros(2,2);
    
    Q_(0, 0) = _Q[0]; Q_(1, 1) = _Q[1]; Q_(2, 2) = _Q[2];
    R_(0, 0) = _R[0]; R_(1, 1) = _R[1];

    kinematic_equation_ = setKinematicEquation();
  }

  // 返回最优控制输入
  virtual std::vector<double> getFirstU(){
    std::vector<double> res;

    casadi::native_DM control_inputs = solution_->value(U);
    casadi::native_DM first_input = control_inputs( casadi::Slice(0, control_inputs.size1()), 
                                                    casadi::Slice(0, 1));

    for(int i = 0; i < first_input.rows(); i++) {
      res.push_back(static_cast<double>(first_input(i, 0)));
    }
    return res;
  }

  // 返回最优轨迹
  std::vector<double> getPredictX(){
    std::vector<double> res;
    auto predict_x = solution_->value(X);

    for (int i = 0; i <= N_; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
    }
    return res;
  }

};

#endif