/**********************************************************************/
// 该节点将slam提供的里程计定位转换到车身的后轴中心后，提供修正后的odom给规划和控制
/**********************************************************************/

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

ros::Publisher odom_fixed_pub;
double axle_offset;

void rcvOdomCallBack(const nav_msgs::Odometry odom_msg)
{
  nav_msgs::Odometry new_odom = odom_msg;  // 创建一个新的里程计消息对象

  // 获取当前里程计的姿态角度
  double yaw = tf::getYaw(new_odom.pose.pose.orientation);

  // 进行坐标变换
  new_odom.pose.pose.position.x += axle_offset * cos(yaw);
  new_odom.pose.pose.position.y += axle_offset * sin(yaw);

  // 发布新的里程计消息
  odom_fixed_pub.publish(new_odom);
}

int main (int argc, char** argv)
{        
	ros::init (argc, argv, "odom_fixed_node");
	ros::NodeHandle nh( "~" );
  
	nh.param("odom_fixed/axle_offset" ,axle_offset, 0.0);

	odom_fixed_pub = nh.advertise<nav_msgs::Odometry>("/odometry_car", 100,true);

	ros::Subscriber odom_raw_sub  = nh.subscribe("odometry", 1, rcvOdomCallBack);

  ros::spin();
	return 0;
}