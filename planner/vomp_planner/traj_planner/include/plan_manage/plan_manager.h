#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>

#include "plan_env/edt_environment.h"
#include "obs_manager/obs_manager.hpp"

#include "path_search/theta_astar.h"
#include "traj_optimize/bspline_opt.h"

class PlanManager
{
public:
  PlanManager(){};
  ~PlanManager(){};

  typedef std::unique_ptr<PlanManager> Ptr;

  /* main planning interface */
  bool hybridReplan(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                    Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw);
  bool hybridReplanAdsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                    Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw);
  bool hybridReplanFsm(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                    Eigen::Vector2d end_pt, Eigen::Vector2d end_vel, double start_yaw);

  void callTrajOptimize();

  void initPlanModules(ros::NodeHandle& nh);
  void initPlanManage(ros::NodeHandle& nh);

  ros::Time get_startTime(){ return start_time_; }
  double get_duration(){ return bspline_traj_.getTimeSum(); }

  Eigen::Vector2d get_startPoint(){ return bspline_traj_.evaluateDeBoorT(0.0);}

  NonUniformBspline get_optimal_traj(){return bspline_traj_;}

  Eigen::Vector2d evaluateFrontPose(ros::Time& t, std::vector<ros::Time>& tlist);

private:

  int drone_id_;                                        // ID号，多机规划使用

  ros::Publisher KinopathPub, TrajpathPub, ctrlPtPub;   // 发布轨迹可视化

  std::unique_ptr<ThetaAstar> theta_path_finder_;       // 路径搜索

  std::unique_ptr<Bspline_Opt> bspline_optimizers_;     // 轨迹优化

  SDFMap::Ptr sdf_map_;
  fast_planner::EDTEnvironment::Ptr edt_environment_;   // ESDF地图

  std::shared_ptr<Obs_Manager> obs_Manager_;            // 动态障碍物预测

  // 轨迹参数
  double sample_time_;
  int status_;                                          // 路径搜索状态
  int cost_function_;                                   // B样条代价函数

  NonUniformBspline bspline_traj_;                      // 优化得到的B样条轨迹

  ros::Time start_time_;  // 轨迹起始时间

public:
  // 前端路径可视化
  void drawKinoPath(const std::vector<Eigen::Vector2d>& trajlist){
    nav_msgs::Path vis_pathmsg;
    vis_pathmsg.header.frame_id = "world";
    vis_pathmsg.header.stamp = ros::Time::now();
    for(auto pos : trajlist){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = pos[0];//x
      pose.pose.position.y = pos[1];//y
      pose.pose.position.z = 0.8;
      pose.header.stamp =  ros::Time::now();//time
      vis_pathmsg.poses.push_back(pose);
    }
    KinopathPub.publish(vis_pathmsg);
  }

  std::vector<Eigen::Vector2d> getGlobalPath(double step_time)
  {
    std::vector<Eigen::Vector2d> trajlist;
    trajlist = theta_path_finder_->getKinoTraj(step_time);
    return trajlist;
  }

  std::vector<ros::Time> get_time_list(double step_time)
  {
    std::vector<ros::Time> timelist;
    timelist = theta_path_finder_->getKinoTraj_t(step_time);
    return timelist; 
  }

  void drawBspline(NonUniformBspline& bspline){
    if (bspline.getControlPoint().size() == 0) return;

    std::vector<Eigen::Vector2d> traj_pts;
    double                  tm, tmp;
    bspline.getTimeSpan(tm, tmp);

    for (double t = tm; t <= tmp; t += 0.01) {
      Eigen::Vector2d pt = bspline.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }

    // B样条路径可视化
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::SPHERE_LIST;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.id                 = 0;
    mk.action             = visualization_msgs::Marker::ADD;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
    mk.color.a = 1.0;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(traj_pts.size()); ++i) {
      pt.x = traj_pts[i](0);
      pt.y = traj_pts[i](1);
      pt.z = 0.8;
      mk.points.push_back(pt);
    }
    TrajpathPub.publish(mk);

    // B样条控制点可视化
    Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector2d> ctp;

    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
      Eigen::Vector2d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }

    visualization_msgs::Marker mk2;
    mk2.header.frame_id = "world";
    mk2.header.stamp    = ros::Time::now();
    mk2.type            = visualization_msgs::Marker::SPHERE_LIST;

    mk2.pose.orientation.x = 0.0;
    mk2.pose.orientation.y = 0.0;
    mk2.pose.orientation.z = 0.0;
    mk2.pose.orientation.w = 1.0;

    mk2.id                 = 1;
    mk2.action             = visualization_msgs::Marker::ADD;
    mk2.color.r = 0.0;
    mk2.color.g = 0.0;
    mk2.color.b = 1.0;
    mk2.color.a = 1.0;
    mk2.scale.x = 0.15;
    mk2.scale.y = 0.15;
    mk2.scale.z = 0.15;

    for (int i = 0; i < int(ctp.size()); ++i) {
      pt.x = ctp[i](0);
      pt.y = ctp[i](1);
      pt.z = 0.8;
      mk2.points.push_back(pt);
    }
    ctrlPtPub.publish(mk2);
  }

};


#endif
