#include "ray_ground_filter.h"

RayGroundFilter::RayGroundFilter(double clip_height, double min_distance, double radial_divider_angle, double sensor_height, 
                double concentric_divider_distance, double min_height_threshold, double local_max_slope,
                double general_max_slope, double reclass_distance_threshold)
{
  this->clip_height_ = clip_height;
  this->min_distance_ = min_distance;
  this->radial_divider_angle_ = radial_divider_angle;
  this->sensor_height_ = sensor_height;
  this->concentric_divider_distance_ = concentric_divider_distance;
  this->min_height_threshold_ = min_height_threshold;
  this->local_max_slope_ = local_max_slope;
  this->general_max_slope_ = general_max_slope;
  this->reclass_distance_threshold_ = reclass_distance_threshold;
}

void RayGroundFilter::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
  pcl::ExtractIndices<pcl::PointXYZ> cliper;
  cliper.setInputCloud(in);

  pcl::PointIndices indices;

  #pragma omp for
  for (size_t i = 0; i < in->points.size(); i++){
    if (in->points[i].z > clip_height){
      indices.indices.push_back(i);
    }
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  cliper.setNegative(true); //ture to remove the indices
  cliper.filter(*out);
}


void RayGroundFilter::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
  pcl::ExtractIndices<pcl::PointXYZ> cliper;
  cliper.setInputCloud(in);
  pcl::PointIndices indices;

  #pragma omp for
  for (size_t i = 0; i < in->points.size(); i++){
    double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);
    if (distance < min_distance){
      indices.indices.push_back(i);
    }
  }
  cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  cliper.setNegative(true); //ture to remove the indices
  cliper.filter(*out);
}

/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundFilter::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
  radial_dividers_num_ = ceil(360 / radial_divider_angle_);
  out_organized_points.resize(in_cloud->points.size());
  out_radial_divided_indices.clear();
  out_radial_divided_indices.resize(radial_dividers_num_);
  out_radial_ordered_clouds.resize(radial_dividers_num_);

  for (size_t i = 0; i < in_cloud->points.size(); i++){
    PointXYZIRTColor new_point;
    auto radius = (float)sqrt( in_cloud->points[i].x * in_cloud->points[i].x + 
                               in_cloud->points[i].y * in_cloud->points[i].y);
    auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;

    if (theta < 0){
      theta += 360;
    }

    //角度的微分
    auto radial_div = (size_t)floor(theta / this->radial_divider_angle_);
    //半径的微分
    auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

    new_point.point = in_cloud->points[i];
    new_point.radius = radius;
    new_point.theta = theta;
    new_point.radial_div = radial_div;
    new_point.concentric_div = concentric_div;
    new_point.original_index = i;

    out_organized_points[i] = new_point;

    //radial divisions更加角度的微分组织射线
    out_radial_divided_indices[radial_div].indices.push_back(i);
    out_radial_ordered_clouds[radial_div].push_back(new_point);
  }

  //将同一根射线上的点按照半径（距离）排序
  #pragma omp for
  for (size_t i = 0; i < radial_dividers_num_; i++)
  {
    std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
              [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
  }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
  out_ground_indices.indices.clear();
  out_no_ground_indices.indices.clear();
  
  #pragma omp for
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
  {
    float prev_radius = 0.f;
    float prev_height = - this->sensor_height_;
    bool prev_ground = false;
    bool current_ground = false;

    //loop through each point in the radial div
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {

      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
      float current_height = in_radial_ordered_clouds[i][j].point.z;
      float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

      //for points which are very close causing the height threshold to be tiny, set a minimum value
      if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_){
        height_threshold = min_height_threshold_;
      }

      //check current point height against the LOCAL threshold (previous point)
      if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold)){
        //Check again using general geometry (radius from origin) if previous points wasn't ground
        if (!prev_ground){
          if (current_height <= (-this->sensor_height_ + general_height_threshold) && current_height >= (-this->sensor_height_ - general_height_threshold)){
            current_ground = true;
          }
          else{
            current_ground = false;
          }
        }
        else{
          current_ground = true;
        }
      }
      else{
        //check if previous point is too far from previous one, if so classify again
        if (points_distance > reclass_distance_threshold_ && (current_height <= (-this->sensor_height_ + height_threshold)
             && current_height >= (-this->sensor_height_ - height_threshold))){
          current_ground = true;
        }
        else{
          current_ground = false;
        }
      }

      if (current_ground){
        out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = true;
      }
      else{
        out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].point.z;
    }
  }
}