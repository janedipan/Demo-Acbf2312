#ifndef RAY_GROUND_FILTER_H
#define RAY_GROUND_FILTER_H

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

struct PointXYZIRTColor
{
  pcl::PointXYZI point;

  float radius; //cylindric coords on XY Plane
  float theta;  //angle deg on XY plane

  size_t radial_div;     //index of the radial divsion to which this point belongs to
  size_t concentric_div; //index of the concentric division to which this points belongs to
  size_t original_index; //index of this point in the source pointcloud
};
typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

class RayGroundFilter
{

// #define CLIP_HEIGHT 1.0 //截取掉高于雷达自身0.2米的点
// #define MIN_DISTANCE 1.0
// #define RADIAL_DIVIDER_ANGLE 0.18
// #define SENSOR_HEIGHT 0.65

// #define concentric_divider_distance_ 0.01 //0.1 meters default
// #define min_height_threshold_ 0.05
// #define local_max_slope_ 8   //max slope of the ground between points, degree
// #define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
// #define reclass_distance_threshold_ 0.2

private:
  // 参数
  double clip_height_ = 1.0;                      // 过滤掉高于雷达clip_height_的点
  double min_distance_ = 1.0;                       // 过滤min_distance_半径内的点
  double radial_divider_angle_ = 0.18;            // 水平角度微分步长
  double sensor_height_ = 0.65;                   // 雷达到地面的距离，单位：m
  double concentric_divider_distance_ = 0.01;     // 点间距，单位：m
  double min_height_threshold_ = 0.01;            // 地面高度阈值，单位：m
  double local_max_slope_ = 8.0;                  // 点与点之间的最大地面坡度，单位：度
  double general_max_slope_ = 5.0;                // 整个点云中地面的最大坡度，单位：度
  double reclass_distance_threshold_ = 0.2;       // 判断两点相距过远的阈值，用于重分类

  size_t radial_dividers_num_;
  size_t concentric_dividers_num_;

public:
  // 默认构造
  RayGroundFilter(){};
  // 有参构造
  RayGroundFilter(double clip_height, double min_distance, double radial_divider_angle, double sensor_height, 
                  double concentric_divider_distance, double min_height_threshold, double local_max_slope,
                  double general_max_slope, double reclass_distance_threshold);
  // 空析构
  ~RayGroundFilter(){};

  // 过滤掉高于雷达clip_height_的点
  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  // 过滤min_distance_半径内的点
  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in, const pcl::PointCloud<pcl::PointXYZ>::Ptr out);

  // 将点云PointXYZ格式转换为RTZColor格式
  void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                        PointCloudXYZIRTColor &out_organized_points,
                        std::vector<pcl::PointIndices> &out_radial_divided_indices,
                        std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

  // Ray ground filter 算法
  void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                   pcl::PointIndices &out_ground_indices,
                   pcl::PointIndices &out_no_ground_indices);

};


#endif