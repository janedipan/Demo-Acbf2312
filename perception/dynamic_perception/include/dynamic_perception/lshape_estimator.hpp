#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "dynamic_perception/DetectedObject.h"
#include "dynamic_perception/DetectedObject.h"
#include <numeric>

class orientation_calc
{
private:
  std::string criterion_;

public:
  orientation_calc(std::string value) : criterion_(value){};
  ~orientation_calc(){};
  bool LshapeFitting(const pcl::PointCloud<pcl::PointXYZ> &cluster, double &theta_optim);
  double calcClosenessCriterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_var(const std::vector<double> &v);
  double calc_variances_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_closeness_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
  double calc_area_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2);
};