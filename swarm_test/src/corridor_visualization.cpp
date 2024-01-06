/* 绘制走廊并显示 */

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher corridor_pub_;

// 走廊参数: @wall_width, @x_offset, @box_scale
double wall_width, x_offset;
Eigen::Vector3d box_scale;

void draw_corridor() {

  Eigen::Vector3d boardX_scale( box_scale.x() - 2 * wall_width,
                                wall_width,
                                box_scale.z());

  Eigen::Vector3d boardY_scale( wall_width,
                                box_scale.y() + 2 * wall_width,
                                box_scale.z());

  // 走廊的四堵外墙
  visualization_msgs::Marker corridor1, corridor2, corridor3, corridor4;

  corridor1.header.frame_id = "world";
  corridor1.header.stamp = ros::Time::now();
  corridor1.action = visualization_msgs::Marker::ADD;
  corridor1.type = visualization_msgs::Marker::CUBE;
  corridor1.id = 1;
	corridor1.color.r = 0.0;
	corridor1.color.g = 1.0;
	corridor1.color.b = 0.0;
	corridor1.color.a = 0.8;
	corridor1.scale.x = boardX_scale.x();
	corridor1.scale.y = boardX_scale.y();
	corridor1.scale.z = boardX_scale.z();
	corridor1.pose.orientation.w = 1.0;
	corridor1.pose.orientation.x = 0.0;
	corridor1.pose.orientation.y = 0.0;
	corridor1.pose.orientation.z = 0.0;
	corridor1.pose.position.x = box_scale.x() / 2.0 - x_offset;
	corridor1.pose.position.y = - (box_scale.y() / 2.0 + wall_width / 2.0);
	corridor1.pose.position.z = box_scale.z() / 2.0;

  corridor2 = corridor1;
  corridor2.id = 2;
	corridor2.pose.position.y = box_scale.y() / 2.0 + wall_width / 2.0;

  corridor3 = corridor1;
  corridor3.id = 3;
	corridor3.scale.x = boardY_scale.x();
	corridor3.scale.y = boardY_scale.y();
	corridor3.scale.z = boardY_scale.z();
	corridor3.pose.position.x = - (x_offset - wall_width / 2.0);
	corridor3.pose.position.y = 0.0;

  corridor4 = corridor3;
  corridor4.id = 4;
	corridor4.pose.position.x = box_scale.x() - x_offset - wall_width / 2.0;

  // 走廊地板
  visualization_msgs::Marker corridor_ground;
  corridor_ground = corridor1;
  corridor_ground.id = 5;
	corridor_ground.color.r = 1.0;
	corridor_ground.color.g = 1.0;
	corridor_ground.color.b = 1.0;
	corridor_ground.color.a = 1.0;
  corridor_ground.scale.x = box_scale.x();
	corridor_ground.scale.y = box_scale.y();
	corridor_ground.scale.z = wall_width/2.0;
  corridor_ground.pose.position.x = box_scale.x() / 2.0 - x_offset;
	corridor_ground.pose.position.y = 0.0;
	corridor_ground.pose.position.z = -wall_width/4.0;

  // 发布走廊可视化
  visualization_msgs::MarkerArray corridor_msg;
  corridor_msg.markers.push_back(corridor1);
  corridor_msg.markers.push_back(corridor2);
  corridor_msg.markers.push_back(corridor3);
  corridor_msg.markers.push_back(corridor4);
  corridor_msg.markers.push_back(corridor_ground);
  corridor_pub_.publish(corridor_msg);
}

int main (int argc, char** argv) {

	ros::init (argc, argv, "corridor_visualization_node");
	ros::NodeHandle nh( "~" );

  // 走廊参数设置: @wall_width, @x_offset, @box_scale
	nh.param("corridor_visualization/wall_width" , wall_width,      0.2);
	nh.param("corridor_visualization/x_offset" ,   x_offset,        1.0);
	nh.param("corridor_visualization/box_scale_x", box_scale.x(),   22.0);
	nh.param("corridor_visualization/box_scale_y", box_scale.y(),   8.0);
	nh.param("corridor_visualization/box_scale_z", box_scale.z(),   1.0);

	corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/corridor_vis", 1, true);

  draw_corridor();

  ros::spin();
	return 0;
}





