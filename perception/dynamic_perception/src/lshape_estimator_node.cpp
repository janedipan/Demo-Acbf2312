#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "dynamic_perception/lshape_estimator.hpp"
#include "dynamic_perception/DetectedObjectArray.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <chrono>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dynamic_perception/CloudClusterArray.h"
#include "dynamic_perception/CloudCluster.h"
#include <opencv2/opencv.hpp>

#define __APP_NAME__ "L shape Fitting"

static ros::Publisher time_lshape_pub;
static std::chrono::time_point<std::chrono::system_clock> lshape_start, lshape_end;
static std_msgs::Float32 time_lshape;
static double exe_time = 0.0;
bool first_appear = true;

class LShapeFittingNode
{
private: 
  tf::TransformListener *_transform_listener;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh;
  ros::Publisher pub_autoware_object_;
  ros::Publisher pub_jsk_bboxes_;
  ros::Subscriber sub_from_clustering;

  std::string bbox_source_frame_;
  std::string bbox_target_frame_;
  std::string L_shape_input_;
  std::string L_shape_output_;
  std::string L_shape_visualization_;
  std::string criterion_;

  float filter_res_;
  std::vector<double> bias_vector;
  std::vector<int> runtime_vector;
  bool first_appear;

  tf2_ros::TransformListener tf2_listener;
  tf2_ros::Buffer tf2_buffer;

  jsk_recognition_msgs::BoundingBox jsk_bbox_transform(const dynamic_perception::DetectedObject &autoware_bbox,
                                                       const std_msgs::Header &header, const geometry_msgs::Pose &pose_transformed);

  void MainLoop(const dynamic_perception::CloudClusterArray::ConstPtr &input_msg);
  void calculateDimPos(const pcl::PointCloud<pcl::PointXYZ> &cluster, dynamic_perception::DetectedObject &output, double &theta_star);

  void eval_running_time(int running_time);

  void eval_performance(double &theta_trans, double &theta_optim, const uint32_t &index, const uint32_t &index_seq);

public:
  LShapeFittingNode();
  ~LShapeFittingNode(){};
};


LShapeFittingNode::LShapeFittingNode() : nh_(""), private_nh("~"),tf2_listener(tf2_buffer)
{
  bias_vector.clear();
  runtime_vector.clear();
  first_appear = true;

  /* Initialize tuning parameter */
  private_nh.param<std::string>("/l_shape_fitting/criterion", criterion_, "AREA");
  private_nh.param<std::string>("/l_shape_fitting/target_frame", bbox_target_frame_, "velodyne_1");    
  private_nh.param<float>("/l_shape_fitting/filter_res", filter_res_, 0.0);    

  private_nh.param<std::string>("/l_shape_fitting/input_topic", L_shape_input_, "/kitti3d/cluster_array");
  private_nh.param<std::string>("/l_shape_fitting/output_objects_topic", L_shape_output_, "/l_shape_fitting/autoware_objects");
  private_nh.param<std::string>("/l_shape_fitting/visualization_topic", L_shape_visualization_, "/l_shape_fitting/jsk_bbox_array");

  ROS_INFO("[%s] bounding box's target frame is: %s", __APP_NAME__, bbox_target_frame_);
  ROS_INFO("[%s] filter_res is: %f", __APP_NAME__, filter_res_);
  ROS_INFO("[%s] L shape input_topic is: %s", __APP_NAME__, L_shape_input_);
  ROS_INFO("[%s] L shape output_topic is: %s", __APP_NAME__, L_shape_output_);
  ROS_INFO("[%s] L-shape visualization topic is: %s", __APP_NAME__, L_shape_visualization_);

  sub_from_clustering = nh_.subscribe(L_shape_input_, 1, &LShapeFittingNode::MainLoop, this);
  pub_autoware_object_ = nh_.advertise<dynamic_perception::DetectedObjectArray>(L_shape_output_, 1, true);
  pub_jsk_bboxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(L_shape_visualization_, 1);
  time_lshape_pub = nh_.advertise<std_msgs::Float32>("/time_lshape_fitting", 1);
}


void LShapeFittingNode::MainLoop(const dynamic_perception::CloudClusterArray::ConstPtr& input_msg)
{
  lshape_start = std::chrono::system_clock::now();

  // Create output msg
  auto output_msg = *input_msg;

  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_recognition_msgs::BoundingBoxArray jsk_bbox_array;
  geometry_msgs::TransformStamped transform_stamped;

  auto bbox_header = input_msg->header;
  bbox_source_frame_ = bbox_header.frame_id;
  bbox_header.frame_id = bbox_target_frame_;

  jsk_bbox_array.header = bbox_header;
  output_msg.header = bbox_header;

  try
  {
    transform_stamped = tf2_buffer.lookupTransform(bbox_target_frame_, bbox_source_frame_, ros::Time());
    // ROS_INFO("target_frame is %s",bbox_target_frame_.c_str());
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("Frame Transform Given Up! Outputing obstacles in the original LiDAR frame %s instead...", bbox_source_frame_.c_str());
    bbox_header.frame_id = bbox_source_frame_;
    try
    {
      transform_stamped = tf2_buffer.lookupTransform(bbox_source_frame_, bbox_source_frame_, ros::Time(0));
    }
    catch (tf2::TransformException& ex2)
    {
      ROS_ERROR("%s", ex2.what());
      return;
    }
  }
  
  orientation_calc orient_calc_(criterion_);

  // Estimate shape for each object and pack msg
  for (auto &cluster_ : output_msg.clusters)
  {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cluster_.cloud, *cluster);

    if (filter_res_ > 0)
    {
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cluster);
      vg.setLeafSize(filter_res_, filter_res_, filter_res_);
      vg.filter(*cluster);
    }
    else{
      //ROS_INFO("no filting for the L-shape fitting");
    }

    dynamic_perception::DetectedObject object;
    double theta_optim;

    const auto eval_start_time = std::chrono::system_clock::now();

    bool success_fitting = orient_calc_.LshapeFitting(*cluster, theta_optim);

    // int max = 1570;
    // int min = 0;
    // int rand_theta = min + rand() % (max - min + 1);
    // theta_optim = rand_theta * 0.001;

    // ROS_INFO("rand_theta=%d", rand_theta);

    if(!success_fitting)
      continue;

    const auto eval_end_time = std::chrono::system_clock::now();
    const auto eval_exe_time = std::chrono::duration_cast<std::chrono::microseconds>(eval_end_time - eval_start_time);

    eval_running_time(eval_exe_time.count());
    eval_performance(cluster_.orientation, theta_optim, cluster_.index, cluster_.index_seq);

    calculateDimPos(*cluster, object, theta_optim);

    geometry_msgs::Pose pose,
    pose_transformed;
    pose.position = object.pose.position;
    pose.orientation = object.pose.orientation;
 
    tf2::doTransform(pose, pose_transformed, transform_stamped);

    object.header = bbox_header;
    object.pose = pose_transformed;

    // if(object.dimensions.x > 2.0 || object.dimensions.y > 2.0 || object.dimensions.z > 2.0)
    //   continue;

    jsk_bbox = jsk_bbox_transform(object, bbox_header, pose_transformed);
    jsk_bbox_array.boxes.emplace_back(jsk_bbox);
  }

  // Publish bounding box information
  pub_autoware_object_.publish(output_msg);
  pub_jsk_bboxes_.publish(jsk_bbox_array);

  // Time the whole process
  lshape_end = std::chrono::system_clock::now();
  exe_time = std::chrono::duration_cast<std::chrono::microseconds>(lshape_end - lshape_start).count() / 1000.0;
  time_lshape.data = exe_time;
  time_lshape_pub.publish(time_lshape);
  // std::cout << "BBOX L-shape fitting took " << exe_time.count() << " milliseconds" << std::endl;


  std::string filename = "/home/dnn/paper_pose_est/L_shape_fitting/src/l_shape_fitting/evaluation/time.txt";
  if (boost::filesystem::exists(filename) && first_appear)
  {
    boost::filesystem::remove(filename);
    first_appear = false;
  }

  std::ofstream out_txt(filename, std::ios::app);
  out_txt << exe_time << std::endl;
  out_txt.close();
}

void LShapeFittingNode::calculateDimPos(const pcl::PointCloud<pcl::PointXYZ>& cluster, dynamic_perception::DetectedObject& output, double &theta_star)
{
  constexpr double ep = 0.001;
  // calc centroid point for cylinder height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto& pcl_point : cluster)
  {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / (double)cluster.size();
  centroid.y = centroid.y / (double)cluster.size();
  centroid.z = centroid.z / (double)cluster.size();

  // // calc min and max z for cylinder length
  double min_z = 0;
  double max_z = 0;
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    if (cluster.at(i).z < min_z || i == 0)
      min_z = cluster.at(i).z;
    if (max_z < cluster.at(i).z || i == 0)
      max_z = cluster.at(i).z;
  }

  // calc circumscribed circle on x-y plane
  cv::Mat_<float> cv_points((int)cluster.size(), 2);
  for (size_t i = 0; i < cluster.size(); ++i)
  {
    cv_points(i, 0) = cluster.at(i).x;  // x
    cv_points(i, 1) = cluster.at(i).y;  // y
  }

  cv::Point2f center;
  float radius;
  cv::minEnclosingCircle(cv::Mat(cv_points).reshape(2), center, radius);

  // Paper : Algo.2 Search-Based Rectangle Fitting
  Eigen::Vector2d e_1_star;  // col.11, Algo.2
  Eigen::Vector2d e_2_star;
  e_1_star << std::cos(theta_star), std::sin(theta_star);
  e_2_star << -std::sin(theta_star), std::cos(theta_star);
  std::vector<double> C_1_star;  // col.11, Algo.2
  std::vector<double> C_2_star;  // col.11, Algo.2
  for (const auto& point : cluster)
  {
    C_1_star.push_back(point.x * e_1_star.x() + point.y * e_1_star.y());
    C_2_star.push_back(point.x * e_2_star.x() + point.y * e_2_star.y());
  }

  // col.12, Algo.2
  const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
  const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
  const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
  const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

  const double a_1 = std::cos(theta_star);
  const double b_1 = std::sin(theta_star);
  const double c_1 = min_C_1_star;
  const double a_2 = -1.0 * std::sin(theta_star);
  const double b_2 = std::cos(theta_star);
  const double c_2 = min_C_2_star;
  const double a_3 = std::cos(theta_star);
  const double b_3 = std::sin(theta_star);
  const double c_3 = max_C_1_star;
  const double a_4 = -1.0 * std::sin(theta_star);
  const double b_4 = std::cos(theta_star);
  const double c_4 = max_C_2_star;

  // calc center of bounding box
  double intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
  double intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
  double intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
  double intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

  // calc dimention of bounding box
  Eigen::Vector2d e_x;
  Eigen::Vector2d e_y;
  e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
  e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
  Eigen::Vector2d diagonal_vec;
  diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

  // calc yaw
  tf2::Quaternion quat;
  quat.setEuler(/* roll */ 0, /* pitch */ 0, /* yaw */ std::atan2(e_1_star.y(), e_1_star.x()));

  output.pose.orientation = tf2::toMsg(quat);
  // constexpr double ep = 0.001;
  output.dimensions.x = std::fabs(e_x.dot(diagonal_vec));
  output.dimensions.y = std::fabs(e_y.dot(diagonal_vec));
  output.dimensions.z = std::max((max_z - min_z), ep);
  output.pose_reliable = true;
  output.pose.position.x = (intersection_x_1 + intersection_x_2) / 2.0;
  output.pose.position.y = (intersection_y_1 + intersection_y_2) / 2.0;
  output.pose.position.z = centroid.z;

  // check wrong output
  output.dimensions.x = std::max(output.dimensions.x, ep);
  output.dimensions.y = std::max(output.dimensions.y, ep);
}


void LShapeFittingNode::eval_running_time(int running_time)
{
  double runtime_std;
  double runtime_sqr_sum = 0.0;
  double runtime_aver;

  runtime_vector.push_back(running_time);

  double runtime_total_v = 0.0;

  for (size_t i = 0; i < runtime_vector.size(); i++)
  {
    runtime_total_v += runtime_vector[i];
  }

  runtime_aver = runtime_total_v / runtime_vector.size();

  for (size_t i = 0; i < runtime_vector.size(); i++)
  {
    runtime_sqr_sum += (runtime_vector[i] - runtime_aver) * (runtime_vector[i] - runtime_aver);
  }

  runtime_std = sqrt(runtime_sqr_sum / runtime_vector.size());

  std::cout << "runtime_vector.size() is = " << runtime_vector.size() << std::endl;
  std::cout << "running_time is = " << running_time / 1000.0 << std::endl;
  std::cout << "runtime_aver is = " << runtime_aver / 1000.0 << std::endl;
  std::cout << "runtime_std is = " << runtime_std / 1000.0 << std::endl;
  std::cout << "---------------------------------" << std::endl;
}

void LShapeFittingNode::eval_performance(double &theta_kitti, double &theta_optim, const uint32_t &index, const uint32_t &index_seq)
{
  double bias_org = abs(theta_kitti - theta_optim);

  double bias = std::min(bias_org, M_PI / 2 - bias_org);

  double bias_std;
  double bias_sqr_sum = 0.0;
  double aver_accu;

  bias_vector.push_back(bias);

  double bias_total_v = 0.0;

  for (size_t i = 0; i < bias_vector.size(); i++)
  {
    bias_total_v += bias_vector[i];
  }
  aver_accu = bias_total_v / bias_vector.size();

  for (size_t i = 0; i < bias_vector.size(); i++)
  {
    bias_sqr_sum += (bias_vector[i] - aver_accu) * (bias_vector[i] - aver_accu);
  }

  bias_std = sqrt(bias_sqr_sum / bias_vector.size());

  std::cout << "index is = " << index << std::endl;
  std::cout << "index_seq is = " << index_seq << std::endl;
  std::cout << "theta_kitti is = " << theta_kitti << std::endl;
  std::cout << "theta_optim is = " << theta_optim << std::endl;

  std::cout << "bias_total_v is = " << bias_total_v * 180 / M_PI << std::endl;
  std::cout << "bias_vector[] is = " << bias_vector[bias_vector.size() - 1] << std::endl;
  std::cout << "bias_vector size is = " << bias_vector.size() << std::endl;
  std::cout << "aver_mean is = " << aver_accu * 180 / M_PI << std::endl;
  std::cout << "bias_std is = " << bias_std * 180 / M_PI << std::endl;
  std::cout << "bias is = " << bias * 180 / M_PI << std::endl;
  std::cout << "---------------------------------" << std::endl;

  std::string filename = "/home/dnn/paper_pose_est/L_shape_fitting/src/l_shape_fitting/evaluation/bias.txt";
  if (boost::filesystem::exists(filename) && first_appear)
  {
    boost::filesystem::remove(filename);
    first_appear = false;
  }

  std::ofstream out_txt(filename, std::ios::app);
  if (bias * 180 / M_PI > 16.0)
    out_txt << index << " " << index_seq << " " << bias * 180 / M_PI << std::endl;
  out_txt.close();
}

jsk_recognition_msgs::BoundingBox LShapeFittingNode::jsk_bbox_transform(const dynamic_perception::DetectedObject &autoware_bbox, 
          const std_msgs::Header& header, const geometry_msgs::Pose& pose_transformed)
{
  jsk_recognition_msgs::BoundingBox jsk_bbox;
  jsk_bbox.header = header;
  jsk_bbox.pose = pose_transformed;
  jsk_bbox.dimensions = autoware_bbox.dimensions;
  jsk_bbox.label = autoware_bbox.id;
  jsk_bbox.value = 1.0f;

  return std::move(jsk_bbox);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_shape_estimator");
  LShapeFittingNode node;
  ros::spin();
  return 0;
}