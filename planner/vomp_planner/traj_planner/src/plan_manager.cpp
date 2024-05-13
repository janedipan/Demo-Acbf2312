#include "plan_manage/plan_manager.h"

void PlanManager::initPlanModules(ros::NodeHandle& nh){

  nh.param("manager/init_traj_sample_time", sample_time_, 0.5);

  KinopathPub = nh.advertise<nav_msgs::Path>("/vis_dense_kino_traj", 1);
  TrajpathPub = nh.advertise<visualization_msgs::Marker>("/vis_bspline", 1);
  ctrlPtPub   = nh.advertise<visualization_msgs::Marker>("/vis_ctrlPt", 1);

  // ESDF初始化
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new fast_planner::EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  // 动态障碍物管理初始化
  obs_Manager_.reset(new Obs_Manager);
  obs_Manager_->init(nh);

  // 路径搜索初始化
  theta_path_finder_.reset(new ThetaAstar);
  theta_path_finder_->init(nh, edt_environment_, obs_Manager_);

  // B样条优化初始化, 用nlopt优化的，后期可改为浙大的lbfgs
  bspline_optimizers_.reset(new Bspline_Opt);
  bspline_optimizers_->init(nh, edt_environment_, obs_Manager_);
}

// for jane's mpc_cbf
void PlanManager::initPlanManage(ros::NodeHandle& nh){
  nh.param("manager/init_traj_sample_time", sample_time_, 0.5);

  KinopathPub = nh.advertise<nav_msgs::Path>("/vis_dense_kino_traj", 1);
  // ESDF初始化
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new fast_planner::EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  // 动态障碍物管理初始化
  obs_Manager_.reset(new Obs_Manager);
  obs_Manager_->init(nh);

  // 路径搜索初始化
  theta_path_finder_.reset(new ThetaAstar);
  theta_path_finder_->init(nh, edt_environment_, obs_Manager_);

  // B样条优化初始化, 用nlopt优化的，后期可改为浙大的lbfgs
  // bspline_optimizers_.reset(new Bspline_Opt);
  // bspline_optimizers_->init(nh, edt_environment_, obs_Manager_);
}

// 轨迹生成
bool PlanManager::hybridReplan(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                               Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw){

  clock_t search_time_start, search_time_end;    //定义clock_t变量用于计时
  search_time_start = clock();           //开始时间

  theta_path_finder_->reset();
  theta_path_finder_->set_IsConsiderVO(true);
  status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, true, true, -1.0); // 带theta的kino astar

  // TODO1: 搜索失败不一定要退出！保留之前规划的路径点，继续优化并执行
  // 可以把前端路径直接参数化为B样条保存下来，后面直接根据B样条去拿点给B样条重新参数化和优化
  if (status_ == NO_PATH) {   // 搜索失败，返回false退出
    theta_path_finder_->reset();
    theta_path_finder_->set_IsConsiderVO(false);    // 再尝试一下使用距离指标搜索路径
    status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, true, true, -1.0);

    if (status_ == NO_PATH) {   // 搜索失败，返回false退出
      std::cout << "[PlanManager replan]: Can't find path." << std::endl;
      return false;
    }
  }

  std::cout << "[PlanManager replan]: Kino search success." << std::endl;

  search_time_end = clock();   // 结束时间
  double search_time_spend = double(search_time_end - search_time_start) / CLOCKS_PER_SEC;

  drawKinoPath(theta_path_finder_->getKinoTraj(0.05));      // 输出前端路径可视化

  clock_t opt_time_start, opt_time_end;     // 定义clock_t变量用于计时
  opt_time_start = clock();                 // 开始时间

  callTrajOptimize();                       // B样条优化

  opt_time_end = clock();                   // 结束时间
  double opt_time_spend = double(opt_time_end - opt_time_start) / CLOCKS_PER_SEC;

  drawBspline(bspline_traj_);       // 输出B样条可视化

  std::cout << "---------------------------------------------------------------------------\n" << std::endl;
  std::cout << "[Vehicle " <<  drone_id_ << "]: Theta-A* cost time := " << search_time_spend << ", Bspline-Opt cost time := " << opt_time_spend << std::endl;

  return true;
}
// 轨迹生成
bool PlanManager::hybridReplanAdsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                               Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw){

  clock_t search_time_start, search_time_end;    //定义clock_t变量用于计时
  search_time_start = clock();           //开始时间

  theta_path_finder_->reset();
  // theta_path_finder_->set_IsConsiderVO(false);
  status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0); // 带theta的kino astar

  // TODO1: 搜索失败不一定要退出！保留之前规划的路径点，继续优化并执行
  // 可以把前端路径直接参数化为B样条保存下来，后面直接根据B样条去拿点给B样条重新参数化和优化
  if (status_ == NO_PATH) {   // 搜索失败，返回false退出
    theta_path_finder_->reset();
    // theta_path_finder_->set_IsConsiderVO(false);    // 再尝试一下使用距离指标搜索路径
    status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0);

    if (status_ == NO_PATH) {   // 搜索失败，返回false退出
      std::cout << "[PlanManager replan]: Can't find path." << std::endl;
      return false;
    }
  }

  std::cout << "[PlanManager replan]: Kino search success." << std::endl;

  search_time_end = clock();   // 结束时间
  double search_time_spend = double(search_time_end - search_time_start) / CLOCKS_PER_SEC;

  drawKinoPath(theta_path_finder_->getKinoTraj(0.2));      // 输出前端路径可视化-Path数据类型

  // clock_t opt_time_start, opt_time_end;     // 定义clock_t变量用于计时
  // opt_time_start = clock();                 // 开始时间

  // callTrajOptimize();                       // B样条优化

  // opt_time_end = clock();                   // 结束时间
  // double opt_time_spend = double(opt_time_end - opt_time_start) / CLOCKS_PER_SEC;

  // drawBspline(bspline_traj_);       // 输出B样条可视化

  // std::cout << "------------------------------------------------------------------------------------\n" << std::endl;
  // std::cout << "[Vehicle " <<  drone_id_ << "]: Theta-A* cost time := " << search_time_spend << ", Bspline-Opt cost time := " << opt_time_spend << std::endl;
  // std::cout << "[Vehicle " <<  drone_id_ << "]: Theta-A* cost time := " << search_time_spend << std::endl;

  return true;
}
// 前端路径搜索状态机
bool PlanManager::hybridReplanFsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                               Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw){

  clock_t search_time_start, search_time_end;    //定义clock_t变量用于计时
  search_time_start = clock();           //开始时间

  theta_path_finder_->reset();
  // theta_path_finder_->set_IsConsiderVO(false);
  status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0); // 带theta的kino astar

  // TODO1: 搜索失败不一定要退出！保留之前规划的路径点，继续优化并执行
  // 可以把前端路径直接参数化为B样条保存下来，后面直接根据B样条去拿点给B样条重新参数化和优化
  if (status_ == NO_PATH) {   // 搜索失败，返回false退出
    theta_path_finder_->reset();
    // theta_path_finder_->set_IsConsiderVO(false);    // 再尝试一下使用距离指标搜索路径
    status_ = theta_path_finder_->search(start_pt, start_vel, start_acc, start_yaw, end_pt, end_vel, false, true, -1.0);

    if (status_ == NO_PATH) {   // 搜索失败，返回false退出
      std::cout << "[PlanManager replan]: Can't find path." << std::endl;
      return false;
    }
  }

  std::cout << "[PlanManager replan]: Kino search success." << std::endl;

  search_time_end = clock();   // 结束时间
  double search_time_spend = double(search_time_end - search_time_start) / CLOCKS_PER_SEC;

  // drawKinoPath(theta_path_finder_->getKinoTraj(0.2));      // 输出前端路径可视化-Path数据类型

  return true;
}

void PlanManager::callTrajOptimize() {

  // parameterize the path to bspline
  double ts = sample_time_ ;

  vector<Eigen::Vector2d> point_set, start_end_derivatives;

  // TODO2: 保存之前规划的路径，根据当前时间索引后面5s的路径点

  
  theta_path_finder_->getSamples_trunc(ts, point_set, start_end_derivatives); // 只取5s内的有用前端路径用于优化
  // theta_path_finder_->getSamples(ts, point_set, start_end_derivatives);  // 取完整前端路径用于优化

  // TODO3: 将索引到的路径点送入优化器

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  cost_function_ = Bspline_Opt::NORMAL_PHASE;

  // if (status_ != REACH_END) {
  //   cost_function_ |= Bspline_Opt::ENDPOINT;
  // }
  // cost_function_ |= Bspline_Opt::ENDPOINT;
  // cost_function_ |= Bspline_Opt::WAYPOINTS;   // 让优化后的轨迹，贴近前端路径的起始点
  // cost_function_ |= Bspline_Opt::CURVATURE;   // 添加曲率约束，效果不好，可能导致优化失败

  std::vector<Eigen::Vector2d> start_waypts;
  std::vector<int> start_waypts_idx;

  // 取得初始路径点，让前段的轨迹贴近初始前端路径
  if(point_set.size() >= 3) {    // 初始路点的个数，可调整
    for(long unsigned int i = 0; i < 3; i++) {
      Eigen::Vector2d waypt;
      waypt = point_set[i];
      start_waypts.push_back(waypt);
      start_waypts_idx.push_back(i);
    }
  } else {
    for(long unsigned int i = 0; i < point_set.size(); i++) {
      Eigen::Vector2d waypt;
      waypt = point_set[i];
      start_waypts.push_back(waypt);
      start_waypts_idx.push_back(i);
    }
  }
  bspline_optimizers_->setWaypoints(start_waypts, start_waypts_idx);    // 设置初始路径点惩罚

  ctrl_pts = bspline_optimizers_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function_, 1, 1);  // B样条优化

  bspline_traj_ = NonUniformBspline(ctrl_pts, 3, ts);   // 获得优化后的轨迹

  start_time_ = bspline_optimizers_->getStartTime();    // 记录轨迹的开始时间
}

Eigen::Vector2d PlanManager::evaluateFrontPose(ros::Time& t, std::vector<ros::Time>& tlist)
{
  int closeIndex;
  std::vector<Eigen::Vector2d> traj_list;
  Eigen::Vector2d evaluPos;
  double minDiff = std::numeric_limits<double>::max();
  
  for(int i=0; i<tlist.size(); i++){
    double diff = std::fabs(t.toSec() - tlist[i].toSec());
    if(diff<minDiff){
      minDiff     = diff;
      closeIndex  = static_cast<int>(i);
    }
  }
  traj_list = theta_path_finder_->getKinoTraj(0.2);
  evaluPos = traj_list[closeIndex];
  // std::cout<< closeIndex<< std::endl;
  // std::cout<< "............ here .............\n";
  // std::cout<< evaluPos<< std::endl;
  return evaluPos;
}