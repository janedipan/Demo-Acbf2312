#include "mpc_core/mpc_base.h"

class MPC_Quadruped : public MPC_base
{
public:
  // 构造函数
  MPC_Quadruped(){};
  // 析构函数
  ~MPC_Quadruped(){};

  void init(int _N,
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
    w_max_  = _delta_max;
    w_min_  = -_delta_max;

    if(_Q.size() != 3){
      ROS_WARN("WARNING: matrix_q size should be equal to 3, please check the config/param.yaml !");
    }
    if(_R.size() != 3){
      ROS_WARN("WARNING: matrix_r size should be equal to 3, please check the config/param.yaml !");
    }

    Q_ = casadi::DM::zeros(3,3);
    R_ = casadi::DM::zeros(3,3);
    
    Q_(0, 0) = _Q[0]; Q_(1, 1) = _Q[1]; Q_(2, 2) = _Q[2];
    R_(0, 0) = _R[0]; R_(1, 1) = _R[1]; R_(2, 2) = _R[2];

    kinematic_equation_ = setKinematicEquation();
  }

  // 设置系统状态方程
  casadi::Function setKinematicEquation() {
    casadi::MX x = casadi::MX::sym("x");                // MPC 状态包括(x, y, theta)，x位置，y位置和theta航向角
    casadi::MX y = casadi::MX::sym("y");
    casadi::MX theta = casadi::MX::sym("theta");        
    casadi::MX state_vars = casadi::MX::vertcat({x, y, theta});

    casadi::MX vx = casadi::MX::sym("vx");                // MPC 控制输入是(vx, vy, w)，x速度,y速度和前轮转角
    casadi::MX vy = casadi::MX::sym("vy");                // MPC 控制输入是(vx, vy, w)，x速度,y速度和前轮转角
    casadi::MX w =  casadi::MX::sym("w");
    casadi::MX control_vars = casadi::MX::vertcat({vx, vy, w});

    // 全向运动学模型
    casadi::MX rhs = casadi::MX::vertcat({  vx * casadi::MX::cos(theta) - vy * casadi::MX::sin(theta),
                                            vx * casadi::MX::sin(theta) + vy * casadi::MX::cos(theta),
                                            w });
    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
  }

  // 求解MPC最优控制
  bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) {
    casadi::Opti opti = casadi::Opti();
    casadi::Slice all;
    casadi::MX cost = 0;

    X = opti.variable(3, N_ + 1);
    U = opti.variable(3, N_);

    casadi::MX x =      X(0, all);
    casadi::MX y =      X(1, all);
    casadi::MX theta =  X(2, all);
    casadi::MX vx =     U(0, all);
    casadi::MX vy =     U(1, all);
    casadi::MX w  =     U(2, all);

    casadi::MX X_ref = opti.parameter(3, N_ + 1);
    casadi::MX X_cur = opti.parameter(3);
    casadi::DM x_tmp = {current_states[0], current_states[1], current_states[2]};

    opti.set_value(X_cur, x_tmp);  //set current state

    // 按列索引
    std::vector<double> X_ref_v(desired_states.data(), desired_states.data() + desired_states.size());

    casadi::DM X_ref_d(X_ref_v);
    X_ref = casadi::MX::reshape(X_ref_d, 3, N_ + 1);

    // MPC代价函数
    for (int i = 0; i < N_; ++i) {
        casadi::MX X_err = X(all, i) - X_ref(all, i); 
        casadi::MX U_0 = U(all, i);
        cost += casadi::MX::mtimes({X_err.T(), Q_, X_err});
        cost += casadi::MX::mtimes({U_0.T(), R_, U_0});
    }

    // 取消：终点误差代价
    // cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), Q_,
    //                     X(all, N_) - X_ref(all, N_)});

    opti.minimize(cost);

    // 运动学约束
    for (int i = 0; i < N_; ++i) {
        std::vector<casadi::MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        casadi::MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    // 初始值约束
    opti.subject_to(X(all, 0) == X_cur);

    // 最大x方向速度，最大y方向速度和最大角速度约束
    opti.subject_to(ux_min_ <= vx <= ux_max_);
    opti.subject_to(uy_min_ <= vy <= uy_max_);
    opti.subject_to(w_min_  <= w  <= w_max_);

    //set solver
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-4;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

    opti.solver("ipopt", solver_opts);

    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
  }

};

