/* 绘制走廊并显示 */

#include <ros/ros.h>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>


ros::Publisher odom_traj_pub_;

Eigen::Vector4d traj_color_;

double line_width_;
int show_odom_size_;

std::deque<nav_msgs::Odometry> odom_vector;

void draw_corridor(const nav_msgs::Odometry& odom_msg) {
  if (odom_vector.size() > show_odom_size_) {
    odom_vector.pop_front();
  }
  odom_vector.push_back(odom_msg);
  // ROS_WARN("pub_odom_traj is:%d",odom_vector.size());
  visualization_msgs::MarkerArray odom_traj_msg;
  visualization_msgs::Marker odom_point;
  odom_point.header= odom_msg.header;
  odom_point.action = visualization_msgs::Marker::ADD;
  odom_point.type = visualization_msgs::Marker::LINE_STRIP;
  odom_point.color.r = traj_color_[0];
  odom_point.color.g = traj_color_[1];
  odom_point.color.b = traj_color_[2];
  odom_point.color.a = traj_color_[3];
  odom_point.scale.x = line_width_;
  odom_point.pose.orientation.w = 1.0;

  for (int i = 1; i < odom_vector.size(); i++) {
    odom_point.id = i;
    geometry_msgs::Point p;
    p = odom_vector[i].pose.pose.position;
    // ROS_WARN("traj_id is:%f,%f",p.x, p.y);
    odom_point.points.push_back(p);
  }
  odom_traj_msg.markers.push_back(odom_point);
  odom_traj_pub_.publish(odom_traj_msg);
}

void rcvOdomCallBack(const nav_msgs::Odometry& odom_msg)
{
  draw_corridor(odom_msg);
}

int main (int argc, char** argv) {

	ros::init (argc, argv, "odom_traj_vis_node");
	ros::NodeHandle nh( "~" );

  // odom轨迹颜色设置: @traj_color: r, g, b, a
	nh.param("odom_traj_vis/color_r", traj_color_[0], 1.0);
	nh.param("odom_traj_vis/color_g", traj_color_[1], 0.0);
	nh.param("odom_traj_vis/color_b", traj_color_[2], 0.0);
	nh.param("odom_traj_vis/color_a", traj_color_[3], 1.0);
	nh.param("odom_traj_vis/show_odom_size", show_odom_size_, 2000);
	nh.param("odom_traj_vis/line_width", line_width_, 0.10);

  odom_vector.clear();

	odom_traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/odom_traj", 1, false);

	ros::Subscriber odom_raw_sub  = nh.subscribe("/odometry", 1, rcvOdomCallBack);

  ros::spin();
	return 0;
}
