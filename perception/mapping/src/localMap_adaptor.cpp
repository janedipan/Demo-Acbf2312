#include "localMap_adaptor.h"

void LocalMap_manager::initMapManager(const ros::NodeHandle& nh, int drone_id){

  this->nh_ = nh; //存储ROS句柄
  this->drone_id_ = drone_id;

  nh_.param("vehicle/car_width", car_width_, 0.75);         // 车辆宽度(m)
  nh_.param("vehicle/car_length", car_length_, 0.98);       // 车辆长度(m)
  nh_.param("vehicle/car_d_cr", car_d_cr_, 0.30);           // 车辆中心到后轴中心的偏移量(m)

  map_ptr_.reset(new MappingProcess);                       // 地图初始化
  map_ptr_->init(nh);

  mapResolution_ = map_ptr_->getResolution();               // get地图的分辨率

  car_vertex_.clear();                                      // 车辆矩形边框四个角点
  Eigen::Vector2d vertex;
  vertex << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
  car_vertex_.push_back(vertex);
  vertex << car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
  car_vertex_.push_back(vertex);
  vertex << -car_length_ / 2.0 + car_d_cr_, -car_width_ / 2.0;
  car_vertex_.push_back(vertex);
  vertex << -car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
  car_vertex_.push_back(vertex);
  vertex << car_length_ / 2.0 + car_d_cr_, car_width_ / 2.0;
  car_vertex_.push_back(vertex);
}

// 检查该二维点的碰撞情况
bool LocalMap_manager::checkIfCollisionUsingPoint(Eigen::Vector2d point){
  if(map_ptr_->getVoxelState2d(point) == 1){
    return true;
  }
  else{
    return false;
  }
}

// 检查小车盒子的碰撞情况
bool LocalMap_manager::CheckIfCollisionUsingPosAndYaw(Eigen::Vector3d state){
  Eigen::Vector2d pos = state.head(2);
  double yaw = state[2];
  Eigen::Matrix2d Rotation_matrix;
  Rotation_matrix << cos(yaw),  -sin(yaw),
                     sin(yaw),  cos(yaw);
  for(int i = 0; i < 4; i++)
  {
    Eigen::Vector2d start_point = pos + Rotation_matrix * car_vertex_[i];
    Eigen::Vector2d end_point = pos + Rotation_matrix * car_vertex_[i+1];

    RayCaster raycaster;
    bool need_ray = raycaster.setInput2d(start_point / mapResolution_, end_point / mapResolution_);
    Eigen::Vector2d half(0.5, 0.5);       // step检查的步长
    Eigen::Vector2d ray_pt;
    while(raycaster.step2d(ray_pt))
    {
      Eigen::Vector2d tmp = (ray_pt + half) * mapResolution_;
      if(checkIfCollisionUsingPoint(tmp))
      {
        return true;
      }
    }
  }
  return false;
}

// 检查 start_pt 到 end_pt 间的离散点的安全性
bool LocalMap_manager::CheckIfCollisionUsingLine(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt){
  RayCaster raycaster;
  bool need_ray = raycaster.setInput2d(start_pt / mapResolution_, end_pt / mapResolution_);

  Eigen::Vector2d half(0.5, 0.5);         // step检查的步长
  Eigen::Vector2d ray_pt;

  while(raycaster.step2d(ray_pt))
  {
    Eigen::Vector2d tmp = (ray_pt + half) * mapResolution_;
    if(checkIfCollisionUsingPoint(tmp))
    {
      return true;
    }
  }
  return false;
}
