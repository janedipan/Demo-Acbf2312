#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

using namespace std;

ros::Subscriber velocity_cmdsub;
ros::Publisher  odom_pub, visugv_pub;
ros::Timer tf_timer_;

nav_msgs::Odometry last_odom;

double p_init_x, p_init_y, p_init_z;

double time_resolution = 0.01;
double L = 608.54e-3;

bool rcv_cmd = false;

string mesh_resource;
geometry_msgs::Twist cmd_vel_;

int car_id_;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> dis(-0.0001, 0.0001);	// 噪声

void rcvVelCmdCallBack(const geometry_msgs::Twist cmd)
{	
	rcv_cmd 	= true;
	cmd_vel_ = cmd;
}

void normyaw(double& y)
{
  if (y > M_PI){
    y-=2*M_PI;
  }
  else if (y < -M_PI){
    y+=2*M_PI;
  }
}

void pubTF() 
{
    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(last_odom.pose.pose.position.x,
                                    last_odom.pose.pose.position.y,
                                    last_odom.pose.pose.position.z));
    q.setW(last_odom.pose.pose.orientation.w);
    q.setX(last_odom.pose.pose.orientation.x);
    q.setY(last_odom.pose.pose.orientation.y);
    q.setZ(last_odom.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "world", "robot_"+std::to_string(car_id_) ) );
}


void pubOdom()
{
	nav_msgs::Odometry new_odom;

	new_odom.header.stamp       = ros::Time::now();
	new_odom.header.frame_id    = "world";

	if(rcv_cmd){
		// rcv_cmd = false;
		double vel_x = cmd_vel_.linear.x;
		double omega = cmd_vel_.angular.z;

		Eigen::Quaterniond q(	last_odom.pose.pose.orientation.w,
							    				last_odom.pose.pose.orientation.x,
							    				last_odom.pose.pose.orientation.y,
							    				last_odom.pose.pose.orientation.z);

		Eigen::Matrix3d R(q);
		double last_yaw 	= atan2(R.col(0)[1],R.col(0)[0]);
		double last_x   	= last_odom.pose.pose.position.x;
		double last_y  	  = last_odom.pose.pose.position.y;
		double last_z  	  = last_odom.pose.pose.position.z;
		double last_a     = last_odom.twist.twist.angular.x;

		new_odom.pose.pose.position.x  = last_x + vel_x * cos(last_yaw) * time_resolution + dis(gen);
		new_odom.pose.pose.position.y  = last_y + vel_x * sin(last_yaw) * time_resolution + dis(gen);
		new_odom.pose.pose.position.z  = last_z;
		new_odom.twist.twist.linear.x  = vel_x;
		new_odom.twist.twist.linear.y  = 0.0;
		new_odom.twist.twist.linear.z  = 0.0;
		new_odom.twist.twist.angular.z = omega;

		double yaw = last_yaw + omega * time_resolution;
		normyaw(yaw);

		Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
		Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
		Eigen::Vector3d zC(0, 0, 1);
		Eigen::Matrix3d R2;
		R2.col(0) = xC;
		R2.col(1) = yC;
		R2.col(2) = zC;
		Eigen::Quaterniond q2(R2);
		new_odom.pose.pose.orientation.w = q2.w();
		new_odom.pose.pose.orientation.x = q2.x();
		new_odom.pose.pose.orientation.y = q2.y();
		new_odom.pose.pose.orientation.z = q2.z();
	}
	else{
		new_odom = last_odom;
		new_odom.header.stamp = ros::Time::now();
		new_odom.pose.pose.position.x += dis(gen);
		new_odom.pose.pose.position.y += dis(gen);
	}

	last_odom = new_odom;
	odom_pub.publish(new_odom);

	visualization_msgs::Marker WpMarker;
	WpMarker.id               = 0;
	WpMarker.header.stamp     = ros::Time::now();
	WpMarker.header.frame_id  = "world";
	WpMarker.action           = visualization_msgs::Marker::ADD;
	WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
	WpMarker.ns               = "ugv_mesh";
	WpMarker.mesh_use_embedded_materials = true;
	WpMarker.color.r          = 1.0;
	WpMarker.color.g          = 1.0;
	WpMarker.color.b          = 1.0;
	WpMarker.color.a          = 1.0;
	WpMarker.scale.x          = 0.001;
	WpMarker.scale.y          = 0.001;
	WpMarker.scale.z          = 0.001;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(new_odom.pose.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

	geometry_msgs::Quaternion q;
	q=tf::createQuaternionMsgFromRollPitchYaw(roll + M_PI/2.0,pitch,yaw + M_PI);
	WpMarker.pose.orientation.w = q.w;
	WpMarker.pose.orientation.x = q.x;
	WpMarker.pose.orientation.y = q.y;
	WpMarker.pose.orientation.z = q.z;
	WpMarker.pose.position.x = new_odom.pose.pose.position.x;
	WpMarker.pose.position.y = new_odom.pose.pose.position.y;
	WpMarker.pose.position.z = new_odom.pose.pose.position.z;
	WpMarker.mesh_resource = mesh_resource;
	visugv_pub.publish(WpMarker);
}

void tftimer_cb(const ros::TimerEvent& e)
{
	pubOdom();
	pubTF();
}

int main (int argc, char** argv)
{        
	ros::init (argc, argv, "scout_simulator_node");
	ros::NodeHandle nh( "~" );

	nh.param("ugv/mesh" ,mesh_resource, std::string("package://kimatic_simulator/param/scout2.STL"));
	nh.param("car_id" ,car_id_, 1);
	nh.param("time_res" ,time_resolution, 0.01);
	nh.param("p_init_x", p_init_x, 0.0);
	nh.param("p_init_y", p_init_y, 0.0);
	nh.param("p_init_z", p_init_z, 0.0);
	
	velocity_cmdsub  = nh.subscribe("command", 1, rcvVelCmdCallBack );
	odom_pub  = nh.advertise<nav_msgs::Odometry>("odometry", 1);
	visugv_pub = nh.advertise<visualization_msgs::Marker>("odom_mesh", 1, true);

	last_odom.header.stamp    = ros::Time::now();
	last_odom.header.frame_id = "world";
	last_odom.pose.pose.position.x = p_init_x;
	last_odom.pose.pose.position.y = p_init_y;
	last_odom.pose.pose.position.z = p_init_z;
	last_odom.pose.pose.orientation.w = 1;
	last_odom.pose.pose.orientation.x = 0;
	last_odom.pose.pose.orientation.y = 0;
	last_odom.pose.pose.orientation.z = 0;
	last_odom.twist.twist.linear.x = 0.0; last_odom.twist.twist.linear.y = 0.0; last_odom.twist.twist.linear.z = 0.0;
	last_odom.twist.twist.angular.x = 0.0; last_odom.twist.twist.angular.y = 0.0; last_odom.twist.twist.angular.z = 0.0;

	tf_timer_ = nh.createTimer(ros::Duration(time_resolution), &tftimer_cb);

	ros::spin();
	return 0;
}