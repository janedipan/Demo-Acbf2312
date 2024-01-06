// 订阅小车odom消息，发布小车marker显示

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


ros::Subscriber odom_sub;
ros::Publisher  visugv_pub;

std::string mesh_resource; // scout建模文件地址
nav_msgs::Odometry odom_now;
bool is_rcv_odom;

void rcvOdomCallBack(const nav_msgs::Odometry odom_msg)
{	
	is_rcv_odom = true;
	odom_now = odom_msg;
}

void pub_UGV_visualize()
{
	if(!is_rcv_odom) return;
	
	visualization_msgs::Marker WpMarker;
	WpMarker.id               = 0;
	WpMarker.header.stamp     = ros::Time::now();
	WpMarker.header.frame_id  = "world";
	WpMarker.action           = visualization_msgs::Marker::ADD;
	WpMarker.type             = visualization_msgs::Marker::MESH_RESOURCE;
	WpMarker.ns               = "ugv_mesh";
	WpMarker.mesh_use_embedded_materials = true;
	WpMarker.color.r          = 0.0;
	WpMarker.color.g          = 0.0;
	WpMarker.color.b          = 0.0;
	WpMarker.color.a          = 0.0;
	WpMarker.scale.x          = 0.001;
	WpMarker.scale.y          = 0.001;
	WpMarker.scale.z          = 0.001;

	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom_now.pose.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

	geometry_msgs::Quaternion q;
	q=tf::createQuaternionMsgFromRollPitchYaw(roll + M_PI/2.0,pitch,yaw + M_PI/2.0);
	WpMarker.pose.orientation.w = q.w;
	WpMarker.pose.orientation.x = q.x;
	WpMarker.pose.orientation.y = q.y;
	WpMarker.pose.orientation.z = q.z;
	WpMarker.pose.position.x = odom_now.pose.pose.position.x;
	WpMarker.pose.position.y = odom_now.pose.pose.position.y;
	WpMarker.pose.position.z = odom_now.pose.pose.position.z + 0.30;
	WpMarker.mesh_resource = mesh_resource;
	visugv_pub.publish(WpMarker);
}

int main (int argc, char** argv)
{        
	ros::init (argc, argv, "hunter_visualizer_node");
	ros::NodeHandle nh( "~" );
	is_rcv_odom = false;
	odom_now.pose.pose.orientation.w = 1.0;

	nh.param("ugv/mesh" ,mesh_resource, std::string("package://kimatic_simulator/param/hunter_se.STL"));
	
	odom_sub  = nh.subscribe("odometry", 1, rcvOdomCallBack);
	visugv_pub = nh.advertise<visualization_msgs::Marker>("odom_mesh", 100,true);

	ros::Rate rate(100);
	bool status = ros::ok();

	while(status) {
		pub_UGV_visualize();     
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}

	return 0;
}