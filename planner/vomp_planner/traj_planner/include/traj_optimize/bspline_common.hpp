#ifndef __BSPLINE_COMMON_H_
#define __BSPLINE_COMMON_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

// An implementation of non-uniform B-spline with different dimensions
// It also represents uniform B-spline which is a special case of non-uniform
class NonUniformBspline {
private:
  // control points for B-spline with different dimensions.
  // Each row represents one single control point
  // The dimension is determined by column number
  // e.g. B-spline with N points in 3D space -> Nx3 matrix
  Eigen::MatrixXd control_points_;

  int             p_, n_, m_;  // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_;          // knots vector
  double          interval_;   // knot span \delta t

  double limit_vel_, limit_acc_, limit_ratio_;  // physical limits and time adjustment ratio

  Eigen::MatrixXd getDerivativeControlPoints(){
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
    for (int i = 0; i < ctp.rows(); ++i) {
      ctp.row(i) =
          p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

public:
  NonUniformBspline() {}
  ~NonUniformBspline() {};
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval){
    setUniformBspline(points, order, interval);
  }

  //B样条基函数获取
  Eigen::MatrixXd getBsplineMatrix(const double& ts, const int& degree, const int& points_num){
    int K = points_num;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + degree - 1);
    if (degree == 3) {
      // Matrix mapping control points to waypoints and boundary derivatives
      Eigen::Vector3d pt_to_pos = 1 / 6.0 * Eigen::Vector3d(1, 4, 1);
      Eigen::Vector3d pt_to_vel = 1 / (2 * ts) * Eigen::Vector3d(-1, 0, 1);
      Eigen::Vector3d pt_to_acc = 1 / (ts * ts) * Eigen::Vector3d(1, -2, 1);

      for (int i = 0; i < K; ++i)
        A.block<1, 3>(i, i) = pt_to_pos.transpose();
      A.block<1, 3>(K, 0) = pt_to_vel.transpose();
      A.block<1, 3>(K + 1, K - 1) = pt_to_vel.transpose();
      A.block<1, 3>(K + 2, 0) = pt_to_acc.transpose();
      A.block<1, 3>(K + 3, K - 1) = pt_to_acc.transpose();
    } else if (degree == 4) {
      // Repeat the same thing, but for 4 degree B-spline
      Eigen::Vector4d pt_to_pos = 1 / 24.0 * Eigen::Vector4d(1, 11, 11, 1);
      Eigen::Vector4d pt_to_vel = 1 / (6 * ts) * Eigen::Vector4d(-1, -3, 3, 1);
      Eigen::Vector4d pt_to_acc = 1 / (2 * ts * ts) * Eigen::Vector4d(1, -1, -1, 1);

      for (int i = 0; i < K; ++i)
        A.block<1, 4>(i, i) = pt_to_pos.transpose();
      A.block<1, 4>(K, 0) = pt_to_vel.transpose();
      A.block<1, 4>(K + 1, K - 1) = pt_to_vel.transpose();
      A.block<1, 4>(K + 2, 0) = pt_to_acc.transpose();
      A.block<1, 4>(K + 3, K - 1) = pt_to_acc.transpose();
    } else if (degree == 5) {
      Eigen::Matrix<double, 5, 1> pt_to_pos, pt_to_vel, pt_to_acc;
      pt_to_pos << 1, 26, 66, 26, 1;
      pt_to_pos /= 120.0;
      pt_to_vel << -1, -10, 0, 10, 1;
      pt_to_vel /= (24 * ts);
      pt_to_acc << 1, 2, -6, 2, 1;
      pt_to_acc /= (6 * ts * ts);

      for (int i = 0; i < K; ++i)
        A.block<1, 5>(i, i) = pt_to_pos.transpose();
      A.block<1, 5>(K, 0) = pt_to_vel.transpose();
      A.block<1, 5>(K + 1, K - 1) = pt_to_vel.transpose();
      A.block<1, 5>(K + 2, 0) = pt_to_acc.transpose();
      A.block<1, 5>(K + 3, K - 1) = pt_to_acc.transpose();
    }
    return A;
  }

  // initialize as an uniform B-spline
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval){
    control_points_ = points;
    p_              = order;
    interval_       = interval;

    n_ = points.rows() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i) {

      if (i <= p_) {
        u_(i) = double(-p_ + i) * interval_;
      } else if (i > p_ && i <= m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
      } else if (i > m_ - p_) {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  // get / set basic bspline info

  void                                   setKnot(const Eigen::VectorXd& knot) {this->u_ = knot;}
  Eigen::VectorXd                        getKnot() {return this->u_;}
  Eigen::MatrixXd                        getControlPoint() {return control_points_;}
  double                                 getInterval() {return interval_;}
  void                                   getTimeSpan(double& um, double& um_p) {um = u_(p_); um_p = u_(m_ - p_);}
  
  std::pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts(){
    Eigen::VectorXd head = evaluateDeBoor(u_(p_));
    Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
    return std::make_pair(head, tail);
  }

  // compute position / derivative

  Eigen::VectorXd   evaluateDeBoor(const double& u){ // use u \in [up, u_mp]

    double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true) {
      if (u_(k + 1) >= ub) break;
      ++k;
    }

    /* deBoor's alg */
    std::vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i) {
      d.push_back(control_points_.row(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r) {
      for (int i = p_; i >= r; --i) {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  Eigen::VectorXd   evaluateDeBoorT(const double& t){ // use t \in [0, duration]
    return evaluateDeBoor(t + u_(p_));
  }

  NonUniformBspline getDerivative() {
    Eigen::MatrixXd   ctp = getDerivativeControlPoints();
    NonUniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  // 3D B-spline interpolation of points in point_set, with boundary vel&acc
  // constraints
  // input : (K+2) points with boundary vel/acc; ts
  // output: (K+6) control_pts
  static void parameterizeToBspline(const double& ts, const std::vector<Eigen::Vector2d>& point_set,
                                    const std::vector<Eigen::Vector2d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts) {
    if (ts <= 0) {
      std::cout << "[B-spline]:time step error." << std::endl;
      return;
    }

    if (point_set.size() < 2) {
      std::cout << "[B-spline]:point set have only " << point_set.size() << " points." << std::endl;
      return;
    }

    if (start_end_derivative.size() != 4) {
      std::cout << "[B-spline]:derivatives error." << std::endl;
    }

    int K = point_set.size();

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
    // cout << "A:\n" << A << endl;

    // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
    // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
    // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
    // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4);
    for (int i = 0; i < K; ++i) {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
    }

    for (int i = 0; i < 4; ++i) {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);

    // convert to control pts
    ctrl_pts.resize(K + 2, 2);
    ctrl_pts.col(0) = px;
    ctrl_pts.col(1) = py;

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  /* check feasibility, adjust time */

  void   setPhysicalLimits(const double& vel, const double& acc) {
    limit_vel_   = vel;
    limit_acc_   = acc;
    limit_ratio_ = 1.1;
  }

  bool   checkFeasibility(bool show = false) {
    bool fea = true;
    // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

    Eigen::MatrixXd P         = control_points_;
    int             dimension = control_points_.cols();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    for (int i = 0; i < P.rows() - 1; ++i) {
      Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
          fabs(vel(2)) > limit_vel_ + 1e-4) {

        if (show) std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_vel = std::max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    for (int i = 0; i < P.rows() - 2; ++i) {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
          (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
          (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
          fabs(acc(2)) > limit_acc_ + 1e-4) {

        if (show) std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;
        fea = false;

        for (int j = 0; j < dimension; ++j) {
          max_acc = std::max(max_acc, fabs(acc(j)));
        }
      }
    }

    double ratio = std::max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
    return fea;
  }

  double checkRatio() {
    Eigen::MatrixXd P         = control_points_;
    int             dimension = control_points_.cols();

    // find max vel
    double max_vel = -1.0;
    for (int i = 0; i < P.rows() - 1; ++i) {
      Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
      for (int j = 0; j < dimension; ++j) {
        max_vel = std::max(max_vel, fabs(vel(j)));
      }
    }
    // find max acc
    double max_acc = -1.0;
    for (int i = 0; i < P.rows() - 2; ++i) {
      Eigen::VectorXd acc = p_ * (p_ - 1) *
          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
          (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
          (u_(i + p_ + 1) - u_(i + 2));
      for (int j = 0; j < dimension; ++j) {
        max_acc = std::max(max_acc, fabs(acc(j)));
      }
    }
    double ratio = std::max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
    ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

    return ratio;
  }

  void   lengthenTime(const double& ratio){
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc   = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
  }

  bool   reallocateTime(bool show = false){
    // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
    // cout << "origin knots:\n" << u_.transpose() << endl;
    bool fea = true;

    Eigen::MatrixXd P         = control_points_;
    int             dimension = control_points_.cols();

    double max_vel, max_acc;

    /* check vel feasibility and insert points */
    for (int i = 0; i < P.rows() - 1; ++i) {
      Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
          fabs(vel(2)) > limit_vel_ + 1e-4) {

        fea = false;
        if (show) std::cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;

        max_vel = -1.0;
        for (int j = 0; j < dimension; ++j) {
          max_vel = std::max(max_vel, fabs(vel(j)));
        }

        double ratio = max_vel / limit_vel_ + 1e-4;
        if (ratio > limit_ratio_) ratio = limit_ratio_;

        double time_ori = u_(i + p_ + 1) - u_(i + 1);
        double time_new = ratio * time_ori;
        double delta_t  = time_new - time_ori;
        double t_inc    = delta_t / double(p_);

        for (int j = i + 2; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 1) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "vel j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }

    /* acc feasibility */
    for (int i = 0; i < P.rows() - 2; ++i) {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
          ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
          (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
          (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
          fabs(acc(2)) > limit_acc_ + 1e-4) {

        fea = false;
        if (show) std::cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;

        max_acc = -1.0;
        for (int j = 0; j < dimension; ++j) {
          max_acc = std::max(max_acc, fabs(acc(j)));
        }

        double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
        if (ratio > limit_ratio_) ratio = limit_ratio_;
        // cout << "ratio: " << ratio << endl;

        double time_ori = u_(i + p_ + 1) - u_(i + 2);
        double time_new = ratio * time_ori;
        double delta_t  = time_new - time_ori;
        double t_inc    = delta_t / double(p_ - 1);

        if (i == 1 || i == 2) {
          // cout << "acc i: " << i << endl;
          for (int j = 2; j <= 5; ++j) {
            u_(j) += double(j - 1) * t_inc;
          }

          for (int j = 6; j < u_.rows(); ++j) {
            u_(j) += 4.0 * t_inc;
          }

        } else {

          for (int j = i + 3; j <= i + p_ + 1; ++j) {
            u_(j) += double(j - i - 2) * t_inc;
            if (j <= 5 && j >= 1) {
              // cout << "acc j: " << j << endl;
            }
          }

          for (int j = i + p_ + 2; j < u_.rows(); ++j) {
            u_(j) += delta_t;
          }
        }
      }
    }
    return fea;
  }

  /* for performance evaluation */

  double getTimeSum(){
    double tm, tmp;
    getTimeSpan(tm, tmp);
    return tmp - tm;
  }

  double getLength(const double& res = 0.01){
    double          length = 0.0;
    double          dur    = getTimeSum();
    Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res) {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double getJerk(){
    NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();
    Eigen::VectorXd times     = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
    int             dimension = ctrl_pts.cols();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
      for (int j = 0; j < dimension; ++j) {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
      }
    }
    return jerk;
  }

  void   getMeanAndMaxVel(double& mean_v, double& max_v){
    NonUniformBspline vel = getDerivative();
    double            tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int    num = 0;
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double          vn  = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel) {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v   = mean_vel;
    max_v    = max_vel;
  }

  void   getMeanAndMaxAcc(double& mean_a, double& max_a){
    NonUniformBspline acc = getDerivative().getDerivative();
    double            tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int    num = 0;
    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double          an  = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc) {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a   = mean_acc;
    max_a    = max_acc;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif