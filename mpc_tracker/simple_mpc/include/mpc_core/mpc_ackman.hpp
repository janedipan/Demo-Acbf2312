#include "mpc_core/mpc_base.h"

class MPC_Ackman : public MPC_base
{
public:
  // 构造函数
  MPC_Ackman(){};
  // 析构函数
  ~MPC_Ackman(){};

  // 设置系统状态方程
  casadi::Function setKinematicEquation() {
    casadi::MX x = casadi::MX::sym("x");                // MPC 状态包括(x, y, theta)，x位置，y位置和theta航向角
    casadi::MX y = casadi::MX::sym("y");
    casadi::MX theta = casadi::MX::sym("theta");        
    casadi::MX state_vars = casadi::MX::vertcat({x, y, theta});

    casadi::MX v = casadi::MX::sym("v");                // MPC的控制输入是(v, w)，速度和前轮转角
    casadi::MX w = casadi::MX::sym("w");
    casadi::MX control_vars = casadi::MX::vertcat({v, w});
    // 二自由度自行车模型
    casadi::MX rhs = casadi::MX::vertcat({v * casadi::MX::cos(theta), v * casadi::MX::sin(theta), v * casadi::MX::tan(w) / L_});
    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
  }

  // 求解MPC最优控制
  bool solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) {
    casadi::Opti opti = casadi::Opti();
    casadi::Slice all;
    casadi::MX cost = 0;

    X = opti.variable(3, N_ + 1);
    U = opti.variable(2, N_);

    casadi::MX x = X(0, all);
    casadi::MX y = X(1, all);
    casadi::MX theta = X(2, all);
    casadi::MX v = U(0, all);
    casadi::MX w = U(1, all);

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

    // 最大速度和前轮转角约束
    opti.subject_to(ux_min_ <= v <= ux_max_);
    opti.subject_to(w_min_  <= w <= w_max_);

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



