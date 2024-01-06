#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
#include<opencv/cv.h>
#else
#include <opencv2/imgproc.hpp>
#endif

#include <std_msgs/Float32.h>
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <dynamic_perception/ri_dbscan_Config.h>

#include "dynamic_perception/CloudCluster.h"
#include "dynamic_perception/CloudClusterArray.h"

#include <tf/transform_listener.h>

#include "ground_truth.hpp"

#define __APP_NAME__ "ri_dbscan"

using PointType = PointXYZILID;


std::string output_frame_;
std::string non_ground_cloud_topic_;
std::string segmented_cloud_topic_;
std::string cluster_cloud_topic_;
std::string colored_cloud_topic_;
std::string cluster_cloud_trans_topic_;
std::string output_cluster_array_topic_;
std::string output_roi_topic_;
std_msgs::Header ros_header;

// Pointcloud Filtering Parameters
float DETECT_MIN, DETECT_MAX;

int CLUSTER_MAX_SIZE, CLUSTER_MIN_SIZE, MinClusterSize, MaxClusterSize;
float cvc_coef;

tf::TransformListener *_transform_listener;
tf::StampedTransform *_transform;

boost::shared_ptr<groundtruth::DepthCluster<PointType>> gt_verify;


static ros::Publisher time_rviz_pub_;
static std_msgs::Float32 time_rviz;
// static double exe_time = 0.0;