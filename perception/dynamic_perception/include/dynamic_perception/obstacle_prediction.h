#include <ros/ros.h>
#include <Eigen/Core>
#include <string>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <casadi/casadi.hpp>
#include "dynamic_perception/cluster_track.hpp"
#include "dynamic_perception/panther_types.hpp"
#include "dynamic_perception/Hungarian.h"         // 匈牙利多目标分配算法
#include "dynamic_simulator/DynTraj.h"

class Obstalce_prediction{

public:

  Obstalce_prediction(ros::NodeHandle nh);     // 构造函数，初始化
  Obstalce_prediction(){};    // 空构造
  ~Obstalce_prediction(){};   // 空析构

  void init_predictor(ros::NodeHandle nh);  // 初始化函数
  void cluster_cb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& input);
  void generatePredictedPwpForTrack(tp::track& track_j);
  void printAllTracks();

  typedef std::shared_ptr<Obstalce_prediction> Ptr;

private:
  ros::NodeHandle nh_;

  ros::Publisher obs_traj_pub_;
  ros::Publisher predict_traj_pub_;
  
  ros::Subscriber cluster_sub_;

  ros::Timer draw_timer_;

  double time_pcloud;

  // 滑动窗口参数
  int min_size_sliding_window_;
  int max_size_sliding_window_;
  double meters_to_create_new_track_;
  int max_frames_skipped_;

  std::map<int, casadi::Function> cf_get_mean_variance_pred_;
  std::vector<tp::track> all_tracks_;
  int num_seg_prediction_;  // Comes from Matlab

  double getCostRowColum(tp::cluster& a, tp::track& b, double time);
  void addNewTrack(const tp::cluster& c);

  void pubPredictTraj(const ros::TimerEvent& e);

  geometry_msgs::Point eigen2point(Eigen::Vector3d vector)
  {
    geometry_msgs::Point tmp;
    tmp.x = vector[0];
    tmp.y = vector[1];
    tmp.z = vector[2];
    return tmp;
  }

  visualization_msgs::MarkerArray pwp2ColoredMarkerArray(mt::PieceWisePol& pwp, double t_init, double t_final,
                                                        int samples, std::string ns, Eigen::Vector3d& color)
  {
    visualization_msgs::MarkerArray marker_array;

    if (t_final < t_init){
      abort();
      return marker_array;
    }
    double deltaT = (t_final - t_init) / (1.0 * samples);
    geometry_msgs::Point p_last = eigen2point(pwp.eval(t_init));
    int j = 7 * 9000;  // TODO

    for (double t = t_init; t <= t_final; t = t + deltaT){
      visualization_msgs::Marker m;
      m.type = visualization_msgs::Marker::ARROW;
      m.lifetime = ros::Duration(0.50);
      m.header.frame_id = "world";
      m.header.stamp = ros::Time::now();
      m.ns = ns;
      m.action = visualization_msgs::Marker::ADD;
      m.id = j;
      m.color.r = color.x();  // color(RED_NORMAL);
      m.color.g = color.y();
      m.color.b = color.z();
      m.color.a = 1.0;
      m.scale.x = 0.2;
      m.scale.y = 0.0000001;  // rviz complains if not
      m.scale.z = 0.0000001;  // rviz complains if not
      m.pose.orientation.w = 1.0;
      geometry_msgs::Point p = eigen2point(pwp.eval(t));
      m.points.push_back(p_last);
      m.points.push_back(p);
      p_last = p;
      marker_array.markers.push_back(m);
      j = j + 1;
    }
    return marker_array;
  }

  std::vector<std::string> pieceWisePol2String(const mt::PieceWisePol& pwp)
  {
    std::string s_x = "0.0";
    std::string s_y = "0.0";
    std::string s_z = "0.0";

    int deg = pwp.getDeg();

    for (int i = 0; i < (pwp.times.size() - 1); i++)  { // i is the index of the interval
      std::string div_by_delta = "/ (" + std::to_string(pwp.times[i + 1] - pwp.times[i]) + ")";

      std::string t = "(min(t," + std::to_string(pwp.times.back()) + "))";

      std::string u = "(" + t + "-" + std::to_string(pwp.times[i]) + ")" + div_by_delta;
      u = "(" + u + ")";

      std::string cond;  
      if (i == (pwp.times.size() - 2)){ // if the last interval
        cond = "(t>=" + std::to_string(pwp.times[i]) + ")";
      }
      else{
        cond = "(t>=" + std::to_string(pwp.times[i]) + " and " + "t<" + std::to_string(pwp.times[i + 1]) + ")";
      }

      std::string s_x_i = "";
      std::string s_y_i = "";
      std::string s_z_i = "";
      for (int j = 0; j <= deg; j++){
        std::string power_u = "(" + u + "^" + std::to_string(deg - j) + ")";
        s_x_i = s_x_i + "+" + std::to_string((double)pwp.all_coeff_x[i](j)) + "*" + power_u;
        s_y_i = s_y_i + "+" + std::to_string((double)pwp.all_coeff_y[i](j)) + "*" + power_u;
        s_z_i = s_z_i + "+" + std::to_string((double)pwp.all_coeff_z[i](j)) + "*" + power_u;
      }

      s_x_i = cond + "*(" + s_x_i + ")";
      s_y_i = cond + "*(" + s_y_i + ")";
      s_z_i = cond + "*(" + s_z_i + ")";

      s_x = s_x + " + " + s_x_i;
      s_y = s_y + " + " + s_y_i;
      s_z = s_z + " + " + s_z_i;
    }
    std::vector<std::string> s;
    s.push_back(s_x);
    s.push_back(s_y);
    s.push_back(s_z);

    return s;
  }

  std::vector<double> eigen2std(const Eigen::Vector3d& v)
  {
    return std::vector<double>{ v.x(), v.y(), v.z() };
  }

  geometry_msgs::Vector3 eigen2rosvector(Eigen::Vector3d vector)
  {
    geometry_msgs::Vector3 tmp;
    tmp.x = vector(0, 0);
    tmp.y = vector(1, 0);
    tmp.z = vector(2, 0);
    return tmp;
  }

  dynamic_simulator::PieceWisePolTraj pwp2PwpMsg(const mt::PieceWisePol& pwp)
  {
    dynamic_simulator::PieceWisePolTraj pwp_msg;

    for (int i = 0; i < pwp.times.size(); i++){
      pwp_msg.times.push_back(pwp.times[i]);
    }

    for (auto coeff_x_i : pwp.all_coeff_x){
      dynamic_simulator::CoeffPoly coeff_poly3;

      for (int i = 0; i < coeff_x_i.size(); i++){
        coeff_poly3.data.push_back(coeff_x_i(i));
      }
      pwp_msg.all_coeff_x.push_back(coeff_poly3);
    }

    for (auto coeff_y_i : pwp.all_coeff_y){
      dynamic_simulator::CoeffPoly coeff_poly3;
      for (int i = 0; i < coeff_y_i.size(); i++){
        coeff_poly3.data.push_back(coeff_y_i(i));
      }
      pwp_msg.all_coeff_y.push_back(coeff_poly3);
    }

    for (auto coeff_z_i : pwp.all_coeff_z){
      dynamic_simulator::CoeffPoly coeff_poly3;
      for (int i = 0; i < coeff_z_i.size(); i++){
        coeff_poly3.data.push_back(coeff_z_i(i));
      }
      pwp_msg.all_coeff_z.push_back(coeff_poly3);
    }

    return pwp_msg;
  }

};
