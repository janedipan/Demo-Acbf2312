#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Odometry odom_;
ros::Publisher global_path_pub;

bool is_odom_rcv_, is_target_rcv_;

Eigen::Vector2d start_pos, target_pos;

void odomCallback(const nav_msgs::Odometry& msg) 
{
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  odom_ = msg;

  start_pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y;
  
  is_odom_rcv_ = true;
}

void waypointCallback(const geometry_msgs::PoseStamped& msg) {
  target_pos << msg.pose.position.x, msg.pose.position.y;
  if(is_odom_rcv_) {
    start_pos << odom_.pose.pose.position.x, odom_.pose.pose.position.y;
  }
  is_target_rcv_ = true;
}

void globalPathPub_Callback(const ros::TimerEvent& e) {

  if (!is_target_rcv_ || !is_odom_rcv_) return;

  double dist = (target_pos - start_pos).norm();
  Eigen::Vector2d diff = (target_pos - start_pos) / dist;

  const double step = 0.12;

  nav_msgs::Path global_path;
  global_path.header.stamp = ros::Time::now();
  global_path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "world";
  pose_stamped.pose.orientation.x = 0;
  pose_stamped.pose.orientation.y = 0;
  pose_stamped.pose.orientation.z = 0;
  pose_stamped.pose.orientation.w = 1;

  int idx = 0;
  for (double i = 0.0; i < dist; i += step)
  {
    pose_stamped.header.seq = idx++;

    Eigen::Vector2d waypoint = start_pos + i * diff;
    pose_stamped.pose.position.x = waypoint.x();
    pose_stamped.pose.position.y = waypoint.y();
    pose_stamped.pose.position.z = 0;

    global_path.poses.push_back(pose_stamped);
  }

  global_path_pub.publish(global_path);
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "global_path_by_rviz");
  ros::NodeHandle node;

  is_odom_rcv_ = false; is_target_rcv_ = false; 

  ros::Subscriber odom_sub = node.subscribe("/odometry", 50, odomCallback);

  ros::Subscriber waypoint_sub_ = node.subscribe("/move_base_simple/goal", 1, waypointCallback);

  global_path_pub = node.advertise<nav_msgs::Path>("/global_path", 1);

  ros::Timer global_path_timer = node.createTimer(ros::Duration(0.10), globalPathPub_Callback);  // 定时碰撞检查

  ros::Duration(1.0).sleep();

  ROS_WARN("[Test_demo]: ready");

  ros::spin();

  return 0;
}
