#ifndef _LOCAL_MAP_ADAPTER_H
#define _LOCAL_MAP_ADAPTER_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "mapping.h"
#include "raycast.h"


class LocalMap_manager
{
public:
  LocalMap_manager(){};  // 空的构造和析构
  ~LocalMap_manager(){};

  void initMapManager(const ros::NodeHandle& nh, int drone_id);                               // 初始化
  bool checkIfCollisionUsingPoint(Eigen::Vector2d point);                               // 从地图碰撞状态
  bool CheckIfCollisionUsingPosAndYaw(Eigen::Vector3d state);                           // 返回碰撞状态
  bool CheckIfCollisionUsingLine(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);     // 返回碰撞状态

  double get_resolusion(){ return mapResolution_; }  // 返回地图的分辨率

  typedef std::shared_ptr<LocalMap_manager> Ptr;   // shared_ptr智能指针

private:
  int drone_id_;                                  // 车辆标签id
  ros::NodeHandle nh_;                            // ros句柄
  MappingProcess::Ptr map_ptr_;                   // 占据栅格地图指针
  double mapResolution_;                          // 地图分辨率
  std::vector<Eigen::Vector2d> car_vertex_;       // 车辆四个角点
  double car_width_, car_length_, car_d_cr_;      // 车辆外形尺寸
};


#endif