#ifndef _BSPLINE_OPT_H_
#define _BSPLINE_OPT_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

#include "plan_env/edt_environment.h"
#include "obs_manager/obs_manager.hpp"

#include "traj_optimize/bspline_common.hpp"


class Bspline_Opt {

public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;
  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;
  static const int CURVATURE;
  static const int VELOCITYOBSTACLE;

  Bspline_Opt() {}   // 空的构造和析构
  ~Bspline_Opt() {}

  /* main API */
  void            init( ros::NodeHandle& nh, 
                        const fast_planner::EDTEnvironment::Ptr& env, 
                        const std::shared_ptr<Obs_Manager>& obs_manager);
  void            setEnvironment(const fast_planner::EDTEnvironment::Ptr& env);
  void            setObsManager(const std::shared_ptr<Obs_Manager>& obs_manager);
  void            setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const vector<Eigen::Vector2d>& guide_pt);
  void setWaypoints(const vector<Eigen::Vector2d>& waypts,
                    const vector<int>&             waypt_idx);  // N-2 constraints at most

  void setStartTime(const ros::Time startTime) {this->startTime_ = startTime;}     // 设置优化开始时间，给动态避障优化用

  void optimize();

  Eigen::MatrixXd         getControlPoints();
  vector<Eigen::Vector2d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  fast_planner::EDTEnvironment::Ptr edt_environment_;
  std::shared_ptr<Obs_Manager> obs_Manager_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector2d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline, xy维度, 2
                                       //
  vector<Eigen::Vector2d> guide_pts_;  // geometric guiding path points, N-6
  vector<Eigen::Vector2d> waypoints_;  // waypts constraints
  vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function

  /* optimization parameters */
  int    order_;                  // bspline degree

  double lambda_Smoothness_;                  // jerk smoothness weight
  double lambda_SdfDistance_;                 // distance weight
  double lambda_Feasibility_;                 // feasibility weight
  double lambda_EndPoint_;                    // end point weight
  double lambda_GuidePoint_;                  // guide cost weight
  double lambda_Visibility_;                  // visibility cost weight
  double lambda_WayPoint_;                    // waypoints cost weight
  double lambda_DynObsDis_;                   // 动态障碍物权重
  double lambda_Curvature_;                   // 曲率代价权重

  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  vector<Eigen::Vector2d> g_q_;
  vector<Eigen::Vector2d> g_smoothness_;
  vector<Eigen::Vector2d> g_distance_;
  vector<Eigen::Vector2d> g_feasibility_;
  vector<Eigen::Vector2d> g_endpoint_;
  vector<Eigen::Vector2d> g_guide_;
  vector<Eigen::Vector2d> g_waypoints_;
  vector<Eigen::Vector2d> g_culvature_;
  vector<Eigen::Vector2d> g_vo_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  ros::Time startTime_;                // 优化开始时间，动态避障用

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void          combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const vector<Eigen::Vector2d>& q, double& cost,
                          vector<Eigen::Vector2d>& gradient);   // 光滑性代价
  void calcDistanceCost(const vector<Eigen::Vector2d>& q, double& cost,
                        vector<Eigen::Vector2d>& gradient);     // ESDF障碍物距离惩罚
  void calcFeasibilityCost(const vector<Eigen::Vector2d>& q, double& cost,
                           vector<Eigen::Vector2d>& gradient);  // 轨迹的最大速度和最大加速度惩罚
  void calcEndpointCost(const vector<Eigen::Vector2d>& q, double& cost,
                        vector<Eigen::Vector2d>& gradient);
  void calcGuideCost(const vector<Eigen::Vector2d>& q, double& cost, vector<Eigen::Vector2d>& gradient);
  void calcVisibilityCost(const vector<Eigen::Vector2d>& q, double& cost,
                          vector<Eigen::Vector2d>& gradient);
  void calcWaypointsCost(const vector<Eigen::Vector2d>& q, double& cost,
                         vector<Eigen::Vector2d>& gradient);
  void calcViewCost(const vector<Eigen::Vector2d>& q, double& cost, vector<Eigen::Vector2d>& gradient);
  bool isQuadratic();

  void calcCulvatureCost(const vector<Eigen::Vector2d>& q, double& cost,
                        vector<Eigen::Vector2d>& gradient);   // B样条轨迹的曲率惩罚

  void calcDynDistanceCost(const vector<Eigen::Vector2d>& q, double& cost, 
                          vector<Eigen::Vector2d>& gradient_q);   // 动态障碍物距离惩罚

  void calcDynDistanceCost2(const vector<Eigen::Vector2d>& q, double& cost, 
                          vector<Eigen::Vector2d>& gradient_q);   // 动态障碍物距离惩罚

  void calcCtrPointDistanceCost(const vector<Eigen::Vector2d>& q, double& cost, 
                                vector<Eigen::Vector2d>& gradient_q); // 浙大论文的方法

  /* for benckmark evaluation only */
public:
  vector<double> vec_cost_;
  vector<double> vec_time_;
  ros::Time      time_start_;

  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  ros::Time getStartTime() {return this->startTime_;}     // 获取开始时间

  typedef unique_ptr<Bspline_Opt> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif