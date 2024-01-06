#include "dynamic_perception/obstacle_prediction.h"
#include <ros/package.h>

Obstalce_prediction::Obstalce_prediction(ros::NodeHandle nh):nh_(nh)
{
  nh_.param("min_size_sliding_window", min_size_sliding_window_, 4);  // 这个系数不能改，与casadi求解绑定了
  nh_.param("max_size_sliding_window", max_size_sliding_window_, 40); // 这个系数不能改，与casadi求解绑定了
  nh_.param("meters_to_create_new_track", meters_to_create_new_track_,1.0);
  nh_.param("max_frames_skipped", max_frames_skipped_,0);

  cluster_sub_ = nh_.subscribe("/l_shape_fitting/jsk_bbox_array", 1, &Obstalce_prediction::cluster_cb, this);
  obs_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_prediction/marker_predicted_traj", 1);
  predict_traj_pub_ = nh_.advertise<dynamic_simulator::DynTraj>("obstacle_prediction/trajs_predicted", 1, true);

  draw_timer_ = nh_.createTimer(ros::Duration(0.10), &Obstalce_prediction::pubPredictTraj, this);

  // 加载casadi配置
  for (int i = min_size_sliding_window_; i <= max_size_sliding_window_; i++){
    std::string folder = ros::package::getPath("dynamic_perception") + "/matlab/casadi_generated_files/";
    cf_get_mean_variance_pred_[i] =
          casadi::Function::load(folder + "get_mean_variance_pred_" + std::to_string(i) + ".casadi");
  }
}

void Obstalce_prediction::init_predictor(ros::NodeHandle nh){
  this->nh_ = nh;
  nh_.param("min_size_sliding_window", min_size_sliding_window_, 4);  // 这个系数不能改，与casadi求解绑定
  nh_.param("max_size_sliding_window", max_size_sliding_window_, 40); // 这个系数不能改，与casadi求解绑定
  nh_.param("meters_to_create_new_track", meters_to_create_new_track_,1.0);
  nh_.param("max_frames_skipped", max_frames_skipped_,0);

  cluster_sub_ = nh_.subscribe("/l_shape_fitting/jsk_bbox_array", 1, &Obstalce_prediction::cluster_cb, this);
  obs_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_prediction/marker_predicted_traj", 1);
  predict_traj_pub_ = nh_.advertise<dynamic_simulator::DynTraj>("obstacle_prediction/trajs_predicted", 1, true);

  draw_timer_ = nh_.createTimer(ros::Duration(0.01), &Obstalce_prediction::pubPredictTraj, this);

  // 加载casadi配置
  for (int i = min_size_sliding_window_; i <= max_size_sliding_window_; i++){
    std::string folder = ros::package::getPath("plan_env") + "/matlab/casadi_generated_files/";
    cf_get_mean_variance_pred_[i] =
          casadi::Function::load(folder + "get_mean_variance_pred_" + std::to_string(i) + ".casadi");
  }
}

// 聚类接收回调
void Obstalce_prediction::cluster_cb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& input)
{
  clock_t time_start,time_end;  //定义clock_t变量用于计时
  time_start = clock();          //搜索开始时间

  std::for_each(all_tracks_.begin(), all_tracks_.end(), [](tp::track& x) { x.num_frames_skipped++; });

  int tracks_removed = 0;

  // 删除丢失过久的跟踪
  all_tracks_.erase(std::remove_if(all_tracks_.begin(), all_tracks_.end(),
                                   [this, &tracks_removed](const tp::track& x) mutable {
                                     tracks_removed++;
                                     bool should_be_removed = (x.num_frames_skipped > max_frames_skipped_);
                                     return should_be_removed;
                                   }),
                    all_tracks_.end());

  time_pcloud = input->header.stamp.toSec();  // 当前障碍物消息的时间戳

  // 从消息中拿到聚类信息
  std::vector<tp::cluster> clusters;
  for(int i = 0; i < input->boxes.size(); i++){
    tp::cluster tmp;
    tmp.centroidXYZ = Eigen::Vector3d(input->boxes[i].pose.position.x,
                                      input->boxes[i].pose.position.y,
                                      input->boxes[i].pose.position.z);
    tmp.orientation = Eigen::Vector4d(input->boxes[i].pose.orientation.w,
                                      input->boxes[i].pose.orientation.x,
                                      input->boxes[i].pose.orientation.y,
                                      input->boxes[i].pose.orientation.z);                              
    tmp.boundingbox = Eigen::Vector3d(input->boxes[i].dimensions.x,
                                      input->boxes[i].dimensions.y,
                                      input->boxes[i].dimensions.z);
    tmp.time = input->header.stamp.toSec();
    clusters.push_back(tmp);
  }

  std::vector<unsigned int> indexes_costs_too_big;

  for(unsigned int i = 0; i < clusters.size(); i++){
    double min_cost = std::numeric_limits<double>::max();
    for(auto& track_j : all_tracks_){
      min_cost = std::min(min_cost, getCostRowColum(clusters[i], track_j, time_pcloud));
    }

    if(min_cost > meters_to_create_new_track_){
      indexes_costs_too_big.push_back(i);
    }
  }
  
  for(auto i : indexes_costs_too_big){ addNewTrack(clusters[i]);}  // 对于代价过大的聚类，重新分配跟踪轨道

  if(clusters.size() > 0){
    // 创建匈牙利算法的代价矩阵
    std::vector<std::vector<double>> costMatrix;
    
    for (auto cluster_i : clusters){          // for each of the rows
      std::vector<double> costs_cluster_i;
      for (auto& track_j : all_tracks_){      // for each of the columns
        costs_cluster_i.push_back(getCostRowColum(cluster_i, track_j, time_pcloud));
      }
      costMatrix.push_back(costs_cluster_i);  // Add row to the matrix
    }

    // Run the Hungarian Algorithm;
    HungarianAlgorithm HungAlgo;
    std::vector<int> track_assigned_to_cluster;
    double cost = HungAlgo.Solve(costMatrix, track_assigned_to_cluster);

    for (unsigned int i = 0; i < costMatrix.size(); i++){
      all_tracks_[track_assigned_to_cluster[i]].num_frames_skipped--;

      if (track_assigned_to_cluster[i] == -1){  // 为新出现的障碍物分配一个新轨道
        // std::cout << "cluster " << i << " unassigned, creating new track for it" << std::endl;
        // std::cout << clusters[i].centroidXYZ.transpose() << std::endl;
        addNewTrack(clusters[i]);
      }
      else{
        if (all_tracks_[track_assigned_to_cluster[i]].is_new == true){
          all_tracks_[track_assigned_to_cluster[i]].is_new = false;
          // std::cout << "track " << i << " is new" << std::endl;
        }
        else{
          // std::cout << "Add track " << i << " to History" << std::endl;
          all_tracks_[track_assigned_to_cluster[i]].addToHistory(clusters[i]);
        }
      }
    }
  }
  else{
    std::cout << "No clusters detected" << std::endl;
  }

  // 生成轨迹
  for (auto& track_j : all_tracks_){
    generatePredictedPwpForTrack(track_j);
  }

  time_end = clock();   //搜索结束时间

  double t_prediction = double(time_end-time_start)/CLOCKS_PER_SEC;

  std::cout << "------------------------------------------------------------------\n";
  std::cout << "[Obstacle_prediction]: Track size := " << all_tracks_.size();
  std::cout << ", cost time := " << t_prediction * 1000.0 << "ms" << std::endl;
}

void Obstalce_prediction::pubPredictTraj(const ros::TimerEvent& e){
  int j = 0; int samples = 20;

  for(auto& track_j : all_tracks_){

    if(track_j.shouldPublish() == false){
      // std::cout << "track " << j << " can not be Published" << std::endl;
      continue;
    }

    std::string ns = "predicted_traj_" + std::to_string(j);
    // 发布预测轨迹可视化
    obs_traj_pub_.publish(
        pwp2ColoredMarkerArray(track_j.pwp_mean, time_pcloud, time_pcloud + 1.0, samples, ns, track_j.color));


    // ROS_ERROR("%lf,%lf",(double)(time_pcloud), (double)(ros::Time::now().toSec()));

    dynamic_simulator::DynTraj dynTraj_msg;
    dynTraj_msg.header.frame_id = "world";
    dynTraj_msg.header.stamp = ros::Time::now();
    dynTraj_msg.use_pwp_field = true;
    dynTraj_msg.pwp_mean = pwp2PwpMsg(track_j.pwp_mean);
    dynTraj_msg.pwp_var = pwp2PwpMsg(track_j.pwp_var);
    dynTraj_msg.s_mean = pieceWisePol2String(track_j.pwp_mean);

    std::vector<double> tmp = eigen2std(track_j.getLatestBbox()); // 使用最近的盒子大小

    dynTraj_msg.bbox = std::vector<float>(tmp.begin(), tmp.end());

    double t_now = ros::Time::now().toSec();
    dynTraj_msg.pos = eigen2rosvector(track_j.pwp_mean.eval(t_now));
    dynTraj_msg.id = track_j.id_int;
    dynTraj_msg.is_agent = false;
    // 发布预测轨迹
    predict_traj_pub_.publish(dynTraj_msg);

    j++;
  }
}

void Obstalce_prediction::addNewTrack(const tp::cluster& c)
{
  tp::track tmp(c, min_size_sliding_window_, max_size_sliding_window_);
  generatePredictedPwpForTrack(tmp);
  all_tracks_.push_back(tmp);
}

// 打印所有的跟踪轨道
void Obstalce_prediction::printAllTracks()
{
  std::cout << "All tracks: ";
  for (int i = 0; i < all_tracks_.size(); i++){
    std::cout << "Track " << i << " (" << all_tracks_[i].num_frames_skipped << "), ";
  }
  std::cout << std::endl;
}

// 计算聚类与跟踪轨道的相对距离，作为匈牙利算法的代价
double Obstalce_prediction::getCostRowColum(tp::cluster& a, tp::track& b, double time)
{
  double result = (a.centroidXYZ - b.pwp_mean.eval(time)).norm();
  return result;
}


// 调用Casadi为跟踪轨道拟合二次多项式，预测障碍物轨迹
void Obstalce_prediction::generatePredictedPwpForTrack(tp::track& track_j)
{
  casadi::DM all_pos = casadi::DM::zeros(3, track_j.getSizeSW());  //(casadi::Sparsity::dense(3, track_j.getSizeSW()));
  casadi::DM all_t = casadi::DM::zeros(1, track_j.getSizeSW());    //(casadi::Sparsity::dense(1, track_j.getSizeSW()));

  int current_ssw = track_j.getSizeSW();

  for (int i = 0; i < current_ssw; i++){
    Eigen::Vector3d centroid_i_filtered;
    Eigen::Vector3d centroid_i = track_j.getCentroidHistory(i);

    centroid_i_filtered = centroid_i;

    all_pos(0, i) = centroid_i_filtered.x();
    all_pos(1, i) = centroid_i_filtered.y();
    all_pos(2, i) = centroid_i_filtered.z();

    all_t(0, i) = track_j.getTimeHistory(i);
  }

  std::map<std::string, casadi::DM> map_arguments;
  map_arguments["all_t"] = all_t;
  map_arguments["all_pos"] = all_pos;

  std::map<std::string, casadi::DM> result = cf_get_mean_variance_pred_[current_ssw](map_arguments);

  casadi::DM coeffs_mean = result["coeff_mean"];
  casadi::DM coeffs_var = result["coeff_var"];
  double secs_prediction = double(result["secs_prediction"]);

  Eigen::VectorXd mean_coeff_x(coeffs_mean.columns());
  Eigen::VectorXd mean_coeff_y(coeffs_mean.columns());
  Eigen::VectorXd mean_coeff_z(coeffs_mean.columns());

  for (int i = 0; i < mean_coeff_x.size(); i++){
    mean_coeff_x(i) = double(coeffs_mean(0, i));
    mean_coeff_y(i) = double(coeffs_mean(1, i));
    mean_coeff_z(i) = double(coeffs_mean(2, i));
  }

  mt::PieceWisePol pwp_mean;  // will have only one interval
  pwp_mean.times.push_back(track_j.getLatestTimeSW());
  pwp_mean.times.push_back(track_j.getLatestTimeSW() + secs_prediction);

  pwp_mean.all_coeff_x.push_back(mean_coeff_x);
  pwp_mean.all_coeff_y.push_back(mean_coeff_y);
  pwp_mean.all_coeff_z.push_back(mean_coeff_z);

  track_j.pwp_mean = pwp_mean;

  Eigen::VectorXd var_coeff_x(coeffs_var.columns());
  Eigen::VectorXd var_coeff_y(coeffs_var.columns());
  Eigen::VectorXd var_coeff_z(coeffs_var.columns());

  for (int i = 0; i < var_coeff_x.size(); i++){
    var_coeff_x(i) = double(coeffs_var(0, i));
    var_coeff_y(i) = double(coeffs_var(1, i));
    var_coeff_z(i) = double(coeffs_var(2, i));
  }

  mt::PieceWisePol pwp_var;  // will have only one interval
  pwp_var.times = pwp_mean.times;

  pwp_var.all_coeff_x.push_back(var_coeff_x);
  pwp_var.all_coeff_y.push_back(var_coeff_y);
  pwp_var.all_coeff_z.push_back(var_coeff_z);

  track_j.pwp_var = pwp_var;
}

