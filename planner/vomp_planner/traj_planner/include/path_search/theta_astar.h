#ifndef _THETA_ASTAR_H
#define _THETA_ASTAR_H

#include "path_search/path_common.h"
#include "obs_manager/obs_manager.hpp"

class ThetaAstar {
private:
  /* ---------- 主要的数据结构 ---------- */
  vector<PathNodePtr> path_node_pool_;
  NodeHashTable expanded_nodes_;    // 用于记录扩展节点
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_; // Open-set
  std::vector<PathNodePtr> path_nodes_;
  int use_node_num_, iter_num_;

  /* ---------- record data ---------- */
  Eigen::Vector2d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 5, 5> phi_;  // state transit matrix
  double omega_max_;

  /* ---------- ESDF地图 ---------- */
  fast_planner::EDTEnvironment::Ptr edt_environment_;

  /* ---------- 动态障碍物 ---------- */
  std::shared_ptr<Obs_Manager> obs_Manager_;

  bool is_consider_omega_ = true;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* 搜索参数 */
  double max_tau_, init_max_tau_;
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;

  /* map参数 */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector2d origin_, map_size_2d_;
  double time_origin_;

  bool is_used_VO_, is_used_DIS_;

  /* helper */
  Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj( Eigen::VectorXd state1, Eigen::VectorXd state2, 
                        ros::Time start_time, double time_to_goal);                    
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);
  bool is_occupancy(const Eigen::Vector3d& pt);

  /* 状态转移：预积分 */
  void stateTransit(Eigen::Matrix<double, 5, 1>& state0,
                    Eigen::Matrix<double, 5, 1>& state1, 
                    Eigen::Vector2d um,
                    double tau);

public:
  ThetaAstar(){};
  ~ThetaAstar();

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void init(ros::NodeHandle& nh,
            const fast_planner::EDTEnvironment::Ptr& env,
            const shared_ptr<Obs_Manager>& obs_Manager);
  
  void reset();
  int search(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel,
             Eigen::Vector2d start_acc, double start_yaw,
             Eigen::Vector2d end_pt, Eigen::Vector2d end_vel,
             bool init, bool dynamic = true, double time_start = -1.0);

  void setEnvironment(const fast_planner::EDTEnvironment::Ptr& env);

  void setObsManager(const shared_ptr<Obs_Manager>& obs_Manager);

  void set_IsConsiderVO(bool is_consider_VO) {is_used_VO_ = is_consider_VO;}

  std::vector<Eigen::Vector2d> getKinoTraj(double delta_t);

  void getSamples(double& ts, vector<Eigen::Vector2d>& point_set,
                  vector<Eigen::Vector2d>& start_end_derivatives);

  void getSamples_trunc(double& ts, vector<Eigen::Vector2d>& point_set,
                        vector<Eigen::Vector2d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<ThetaAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif