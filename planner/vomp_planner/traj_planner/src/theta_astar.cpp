// 考虑差速车辆模型的 kino-Astar
#include "path_search/theta_astar.h"

ThetaAstar::~ThetaAstar()   // 析构函数
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

// kino_Astar初始化
void ThetaAstar::init(ros::NodeHandle& nh,
                      const fast_planner::EDTEnvironment::Ptr& env,
                      const shared_ptr<Obs_Manager>& obs_Manager)
{
  setParam(nh);
  setEnvironment(env);
  setObsManager(obs_Manager);
  init();
}

int ThetaAstar::search(Eigen::Vector2d start_pt, Eigen::Vector2d start_v, Eigen::Vector2d start_a, double start_yaw,
                      Eigen::Vector2d end_pt, Eigen::Vector2d end_v, bool init, bool dynamic, double time_start)
{
  start_vel_ = start_v;
  start_acc_ = start_a;

  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state_car.head(2) = start_pt;
  cur_node->state_car.tail(2) = start_v;
  // Heading: 起始状态的heading
  cur_node->state_car[2] = start_yaw;

  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  cur_node->node_time = ros::Time::now(); //保存：从当前节点到起点的时间

  Eigen::VectorXd end_state(5);
  Eigen::Vector2i end_index;
  double time_to_goal;

  end_state.head(2) = end_pt;
  end_state.tail(2) = end_v;
  // Heading: 终点状态的heading
  end_state[2] = 0.0;

  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state_car, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    cur_node->node_time = ros::Time::now(); //保存：从当前节点到起点的时间
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();

    // Terminate?
    bool reach_horizon = (cur_node->state_car.head(2) - start_pt).norm() >= horizon_;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state_car, end_state, time_to_goal);
        computeShotTraj(cur_node->state_car, end_state, cur_node->node_time, time_to_goal);
        if (init_search)
          ROS_ERROR("Shot in first search loop!");
      }
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 4.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0; // 搜索扩展分辨率

    Eigen::Matrix<double, 5, 1> cur_state = cur_node->state_car;
    Eigen::Matrix<double, 5, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector2d um;
    double pro_t;
    vector<Eigen::Vector2d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      double max_acc_y_ = max_acc_ * 0.5;
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_y_; ay <= max_acc_y_ + 1e-3; ay += max_acc_y_ * res)
          {
            um << ax, ay;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    for (long unsigned int i = 0; i < inputs.size(); ++i) {
      for (long unsigned int j = 0; j < durations.size(); ++j) {
        
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);

        // 筛选掉角速度过大的节点，适配差速车
        if (is_consider_omega_) {
          double yaw_diff = abs(pro_state[2] - cur_state[2]);
          if(yaw_diff > 0.5 * M_PI) {
            yaw_diff -= 0.5 * M_PI;
          }
          if((yaw_diff / tau) > omega_max_) {
            continue;
          }
        }
        
        pro_t = cur_node->time + tau;

        Eigen::Vector2d pro_pos = pro_state.head(2);

        // Check if in close set
        Eigen::Vector2i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector2d pro_v = pro_state.tail(2);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_)
        {
          if (init_search){
            std::cout << "vel" << std::endl;
          }
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector2i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 5, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);

          if(is_occupancy(pos)) { // esdf地图碰撞检查
            is_occ = true;
            break;
          }
          
          ros::Time node_time = cur_node->node_time + ros::Duration(dt);
          bool is_vo_unsafe = false;
          if (is_used_VO_) {  // 用vo来检查动态障碍物安全性
            Eigen::Vector4d node_state;
            node_state.head(2) = xt.head(2);
            node_state.tail(2) = xt.tail(2);
            is_vo_unsafe = obs_Manager_->is_VO_unsafe(node_state, 0.8, node_time);
          } else if (is_used_DIS_){            // 用距离来检查动态障碍物安全性
            is_vo_unsafe = obs_Manager_->is_collide(xt.head(2), 0.8, node_time);
          } else {
            is_vo_unsafe = false;
          }

          if (is_vo_unsafe)
          {
            is_occ = true;
            break;
          }
        }

        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (long unsigned int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state_car = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              expand_node->node_time = cur_node->node_time + ros::Duration(tau);
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state_car = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            pro_node->node_time = cur_node->node_time + ros::Duration(tau);

            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              std::cout << "run out of memory." << std::endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state_car = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic) {
                pro_node->time = cur_node->time + tau;
                pro_node->node_time = cur_node->node_time + ros::Duration(tau);
              }
            }
          }
          else
          {
            std::cout << "error type in searching: " << pro_node->node_state << std::endl;
          }
        }
      }
    }
  }

  std::cout << "open set empty, no path!" << std::endl;
  std::cout << "use node num: " << use_node_num_ << std::endl;
  std::cout << "iter num: " << iter_num_ << std::endl;
  return NO_PATH;
}

bool ThetaAstar::is_occupancy(const Eigen::Vector3d& pos_XYTheta)
{
  Eigen::Vector2d pos(pos_XYTheta[0], pos_XYTheta[1]);
  Eigen::Matrix2d rota_M;
  double yaw = pos_XYTheta[2];
  rota_M  <<  cos(yaw),-sin(yaw),
              sin(yaw),cos(yaw);

  Eigen::Vector2d car_size(0.6, 0.8);   // 用矩形盒子做碰撞检查

  // 遍历： 逐个检查车辆的内点
  for(double w = -car_size[0]/2; w <= car_size[0]/2 + 1e-3; w += 0.1){
    for(double l = -car_size[1]/2; l <= car_size[1]/2 + 1e-3; l += 0.1){
      for(double h = 0.0; h <= 0.4 + 1e-3; h += 0.1){                    // 车体z方向碰撞检查
        Eigen::Vector2d cur_p = pos + rota_M * Eigen::Vector2d(l, w);
        Eigen::Vector3d pointXYZ(cur_p[0], cur_p[1], h);
        if(edt_environment_->sdf_map_->getInflateOccupancy(pointXYZ) == 1){
          return true;
        }
      }
    }
  }
  return false;
}

void ThetaAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double ThetaAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Eigen::Vector2d dp = x2.head(2) - x1.head(2);
  const Eigen::Vector2d v0 = x1.tail(2);
  const Eigen::Vector2d v1 = x2.tail(2);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(2) - x2.head(2)).lpNorm<Eigen::Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool ThetaAstar::computeShotTraj( Eigen::VectorXd state1, Eigen::VectorXd state2, 
                                  ros::Time start_time, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Eigen::Vector2d p0 = state1.head(2);
  const Eigen::Vector2d dp = state2.head(2) - p0;
  const Eigen::Vector2d v0 = state1.segment(2, 2);
  const Eigen::Vector2d v1 = state2.segment(2, 2);
  const Eigen::Vector2d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(2, 4);
  end_vel_ = v1;

  Eigen::Vector2d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Eigen::Vector2d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Eigen::Vector2d c = v0;
  Eigen::Vector2d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Eigen::Vector2d coord, coord_next, coord_next_2, vel, acc;
  Eigen::VectorXd poly1d, t, t_next, t_next_2, polyv, polya;
  Eigen::Vector2i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0,
        0, 0, 2, 0,
        0, 0, 0, 3,
        0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d - 2 * t_delta; time += t_delta)
  {
    // 当前点的时间
    t = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    // 下一点的时间
    t_next = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t_next(j) = pow(time + t_delta, j);

    // 下下个点的时间
    t_next_2 = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t_next_2(j) = pow(time + 2 * t_delta, j);

    for (int dim = 0; dim < 2; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      coord_next(dim) = poly1d.dot(t_next);
      coord_next_2(dim) = poly1d.dot(t_next_2);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        return false;
      }
    }

    // 计算当前采样点的yaw
    Eigen::Vector2d pos_now_diff, pos_next_diff;
    pos_now_diff = coord_next - coord;
    pos_next_diff = coord_next_2 - coord_next;

    double yaw_now = atan2(pos_now_diff.y(), pos_now_diff.x());
    double yaw_next = atan2(pos_next_diff.y(), pos_next_diff.x());

    // 筛选掉角速度过大的节点，适配差速车
    if (is_consider_omega_) {
      double yaw_diff = abs(yaw_next - yaw_now);
      if (yaw_diff > 0.5 * M_PI) {
        yaw_diff -= 0.5 * M_PI;
      }
      if((yaw_diff / t_delta) > omega_max_) {
        return false;;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_2d_(0) 
        || coord(1) < origin_(1) || coord(1) >= map_size_2d_(1))
    {
      return false;
    }

    /***********************Shot碰撞检查*********************/
    Eigen::Vector3d pos_XYTheta(coord.x(), coord.y(), yaw_now);
    if(is_occupancy(pos_XYTheta)){
      return false;
    }

    ros::Time time_cur = start_time + ros::Duration(time);
    bool is_vo_unsafe = false;
    if (is_used_VO_) {  // 用vo来检查动态障碍物安全性
      Eigen::Vector4d node_state;
      node_state.head(2) = coord;
      node_state.tail(2) = vel;
      is_vo_unsafe = obs_Manager_->is_VO_unsafe(node_state, 1.0, time_cur);
    } else if (is_used_DIS_){             // 用距离来检查动态障碍物安全性
      is_vo_unsafe = obs_Manager_->is_collide(coord.head(2), 1.0, time_cur);
    } else {
      is_vo_unsafe = false;
    }

    if (is_vo_unsafe) {
      return false;
    }

  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

std::vector<double> ThetaAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> ThetaAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void ThetaAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  Eigen::Vector3d origin_3d_, map_size_3d_;
  edt_environment_->sdf_map_->getRegion(origin_3d_, map_size_3d_);

  origin_ = origin_3d_.head(2);
  map_size_2d_ = map_size_3d_.head(2);

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    // std::cout << "---------------- :" << i << std::endl;
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(5, 5);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void ThetaAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/omega_max", omega_max_, 0.526);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, 5);
  nh.param("search/optimistic", optimistic_, true);
  nh.param("search/is_consider_omega", is_consider_omega_, true);
  nh.param("search/is_used_VO", is_used_VO_, false);
  nh.param("search/is_used_DIS", is_used_DIS_, false);

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
}

void ThetaAstar::setEnvironment(const fast_planner::EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void ThetaAstar::setObsManager(const shared_ptr<Obs_Manager>& obs_Manager)
{
  this->obs_Manager_ = obs_Manager;
}

void ThetaAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

std::vector<Eigen::Vector2d> ThetaAstar::getKinoTraj(double delta_t)
{
  vector<Eigen::Vector2d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Eigen::Matrix<double, 5, 1> x0, xt;

  while (node->parent != NULL)
  {
    Eigen::Vector2d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state_car;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(2));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Eigen::Vector2d coord;
    Eigen::VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 2; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void ThetaAstar::getSamples(double& ts, vector<Eigen::Vector2d>& point_set,
                           vector<Eigen::Vector2d>& start_end_derivatives)
{
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;
  PathNodePtr node = path_nodes_.back();
  while (node->parent != NULL)
  {
    T_sum += node->duration;
    node = node->parent;
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc
  Eigen::Vector2d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 2; ++dim)
    {
      Eigen::Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }
  else
  {
    t = path_nodes_.back()->duration;
    end_vel = node->state_car.tail(2);
    end_acc = path_nodes_.back()->input;
  }

  // Get point samples

  // int seg_num = floor(T_sum / ts);
  // seg_num = max(8, seg_num);
  // ts = T_sum / double(seg_num);

  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj
      Eigen::Vector2d coord;
      Eigen::Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 2; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 5, 1> x0 = node->parent->state_car;
      Eigen::Matrix<double, 5, 1> xt;
      Eigen::Vector2d ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(2));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector2d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

void ThetaAstar::getSamples_trunc(double& ts, std::vector<Eigen::Vector2d>& point_set,
                                  std::vector<Eigen::Vector2d>& start_end_derivatives) 
{
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;
  PathNodePtr node = path_nodes_.back();
  while (node->parent != NULL)
  {
    T_sum += node->duration;
    node = node->parent;
  }
  // cout << "duration:" << T_sum << endl;
  
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
  }
  else
  {
    t = path_nodes_.back()->duration;
  }

  // Get point samples

  // int seg_num = floor(T_sum / ts);
  // seg_num = max(8, seg_num);
  // ts = T_sum / double(seg_num);

  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0,
        0, 0, 2, 0,
        0, 0, 0, 3,
        0, 0, 0, 0;

  std::vector<Eigen::VectorXd> all_points;
  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // samples on shot traj
      Eigen::Vector2d coord, vel, acc;
      Eigen::Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 2; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
        vel(dim) = (Tm * poly1d).dot(time);
        acc(dim) = (Tm * Tm * poly1d).dot(time);
      }

      Eigen::VectorXd state_cur(6);
      state_cur << coord.x(), coord.y(), vel.x(), vel.y(), acc.x(), acc.y();
      all_points.push_back(state_cur);

      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 5, 1> x0 = node->parent->state_car;
      Eigen::Matrix<double, 5, 1> xt;
      Eigen::Vector2d ut = node->input;

      stateTransit(x0, xt, ut, t);

      Eigen::VectorXd state_cur(6);
      Eigen::Vector2d coord, vel, acc;
      coord = xt.head(2);
      vel = xt.tail(2);
      acc = ut;
      state_cur << coord.x(), coord.y(), vel.x(), vel.y(), acc.x(), acc.y();
      all_points.push_back(state_cur);

      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(all_points.begin(), all_points.end());

  // 截取前几秒的路径
  for (int i = 0; i * ts <= 5.0 && i < all_points.size(); i++)
  {
    point_set.push_back(all_points[i].head(2)); // 放入位置
  }

  // calculate start acc
  Eigen::Vector2d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // no searched traj, calculate by shot traj
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  Eigen::Vector2d end_vel, end_acc;
  end_vel << all_points[all_points.size() - 1][2], all_points[all_points.size() - 1][3];
  end_acc << all_points[all_points.size() - 1][4], all_points[all_points.size() - 1][5];

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> ThetaAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector2i ThetaAstar::posToIndex(Eigen::Vector2d pt)
{
  Eigen::Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

int ThetaAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void ThetaAstar::stateTransit(Eigen::Matrix<double, 5, 1>& state0, Eigen::Matrix<double, 5, 1>& state1,
                                    Eigen::Vector2d um, double tau)
{
  Eigen::Matrix<double, 4, 1> integral;
  integral.head(2) = 0.5 * pow(tau, 2) * um;
  integral.tail(2) = tau * um;
  // 状态转移
  state1.head(2) = state0.head(2) + state0.tail(2) * tau + integral.head(2);
  state1.tail(2) = state0.tail(2) + integral.tail(2);

  if(state1.tail(2).norm() < 1e-1) {  // state1的速度非常小时，yaw与state0相同
    state1[2] = state0[2];
  } else {
    double yaw = atan2(state1[1] - state0[1] ,state1[0] - state0[0]);
    state1[2] = yaw;
  }
}