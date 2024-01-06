
// 裁剪点云，输出裁剪后的点云

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

tf2_ros::Buffer tf_buffer_;

ros::Publisher pub_pcloud_filtered_;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud2_;
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud1;

void pointCallbck(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg) 
{
  pcl::fromROSMsg(*pcl2ptr_msg, *input_cloud1);   // 从ROS的点云消息转换为PCL格式

  std_msgs::Header header_pcloud;
  pcl_conversions::fromPCL(input_cloud1->header, header_pcloud);

  // Transform w_T_b
  Eigen::Affine3d w_T_b;
  geometry_msgs::TransformStamped transform_stamped;

  try
  {
    // 监听点云帧坐标到world坐标的转换，延时参数需根据实际情况调整
    transform_stamped = tf_buffer_.lookupTransform("world", header_pcloud.frame_id, header_pcloud.stamp,
                                                   ros::Duration(0.10));
    w_T_b = tf2::transformToEigen(transform_stamped);
  }
  catch (tf2::TransformException& ex)
  {
    // ROS_WARN("[tracker_predictor] OnGetTransform failed with %s", ex.what());
    return;
  }

  // ROS消息转PCL
  pcl::transformPointCloud(*input_cloud1, *input_cloud2_, w_T_b);

  sensor_msgs::PointCloud2 filtered_pcl2_msg;   // 发布转换后的点云
  pcl::toROSMsg(*input_cloud2_, filtered_pcl2_msg);
  filtered_pcl2_msg.header.frame_id = "world";
  filtered_pcl2_msg.header.stamp = pcl2ptr_msg->header.stamp;
  pub_pcloud_filtered_.publish(filtered_pcl2_msg);
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "points_fixed_node");

  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  input_cloud2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  input_cloud1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

  ros::Subscriber pcl_sub_ = node.subscribe("/velodyne_points", 1, pointCallbck);

  pub_pcloud_filtered_ = node.advertise<pcl::PointCloud<pcl::PointXYZ>>("/fixed_points", 1);

  ROS_WARN("points_fixed_node: ready");

  ros::spin();

  return 0;
}