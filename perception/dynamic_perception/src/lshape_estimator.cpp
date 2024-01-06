#include "dynamic_perception/lshape_estimator.hpp"
#include <memory>
#include <iostream>

#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

bool orientation_calc::LshapeFitting(const pcl::PointCloud<pcl::PointXYZ> &cluster, double &theta_optim)
{
  // if (cluster.size() < 10){
  //   std::cout << "cluster.size() < 10    LshapeFitting\n";
  //   return false;
  // }

  /* Paper : IV2017, Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners */
  const double angle_reso = 0.5 * M_PI / 180.0;
  const double max_angle = M_PI / 2.0;
  double max_q = std::numeric_limits<double>::min();

  double q;
  std::vector<std::pair<double /*theta*/, double /*q*/>> Q;

  // search
  for (double theta = 0; theta < max_angle; theta += angle_reso)
  {
    Eigen::Vector2d e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2d e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<double> C_1;                   // col.5, Algo.2
    std::vector<double> C_2;                   // col.6, Algo.2
    
    for (const auto& point : cluster)
    {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }

    if (criterion_ == "AREA")
    {
      q = calc_area_criterion(C_1, C_2);
    }
    else if (criterion_ == "CLOSENESS")
    {
      q = calc_closeness_criterion(C_1, C_2);
    }
    else if (criterion_ == "VARIANCE")
    {
      q = calc_variances_criterion(C_1, C_2);
    }
    else
    {
      std::cout << "L Shaped Algorithm Criterion Is Not Supported." << std::endl;
      break;
    }
    Q.push_back(std::make_pair(theta, q));        // col.8, Algo.2
  }

  for (size_t i = 0; i < Q.size(); ++i)
  {
    if (Q.at(i).second > max_q || i == 0)
    {
      max_q = Q.at(i).second;
      theta_optim = Q.at(i).first;
    }
  }
  return true;
}


double orientation_calc::calc_area_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  const double c1_min = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c1_max = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c2_min = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double c2_max = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  double alpha = -(c1_max - c1_min) * (c2_max - c2_min);

  return alpha;
}


double orientation_calc::calc_closeness_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  // Paper : Algo.4 Closeness Criterion
  const double min_c_1 = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double max_c_1 = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double min_c_2 = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double max_c_2 = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  std::vector<double> D_1; // col.4, Algo.4
  for (const auto &c_1_element : C_1)
  {
    const double v = std::min(std::fabs(max_c_1 - c_1_element), std::fabs(c_1_element - min_c_1));
    D_1.push_back(v);
  }

  std::vector<double> D_2; // col.5, Algo.4
  for (const auto &c_2_element : C_2)
  {
    const double v = std::min(std::fabs(max_c_2 - c_2_element), std::fabs(c_2_element - min_c_2));
    D_2.push_back(v);
  }

  double beta = 0;

  for (size_t i = 0; i < D_1.size(); i++)
  {
      double d = std::max(std::min(D_1[i], D_2[i]), 0.05);
      beta += (1.0 / d);
  }
  return beta;
}


double orientation_calc::calc_variances_criterion(const std::vector<double> &C_1, const std::vector<double> &C_2)
{
  const double c1_min = *std::min_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c1_max = *std::max_element(C_1.begin(), C_1.end()); // col.2, Algo.4
  const double c2_min = *std::min_element(C_2.begin(), C_2.end()); // col.3, Algo.4
  const double c2_max = *std::max_element(C_2.begin(), C_2.end()); // col.3, Algo.4

  std::vector<double> d1; // col.4, Algo.4
  for (const auto &c_1_element : C_1)
  {
    const double v = std::min(std::fabs(c1_max - c_1_element), std::fabs(c_1_element - c1_min));
    d1.push_back(v);
  }

  std::vector<double> d2; // col.5, Algo.4
  for (const auto &c_2_element : C_2)
  {
    const double v = std::min(std::fabs(c2_max - c_2_element), std::fabs(c_2_element - c2_min));
    d2.push_back(v);
  }

  std::vector<double> e1;
  std::vector<double> e2;

  assert(d1.size() == d2.size());

  for (size_t i = 0; i < d1.size(); i++)
  {
    if (d1[i] < d2[i])
    {
      e1.push_back(d1[i]);
    }
    else
    {
      e2.push_back(d2[i]);
    }
  }

  double v1 = 0.0;
  if (!e1.empty())
  {
    v1 = calc_var(e1);
  }

  double v2 = 0.0;
  if (!e2.empty())
  {
    v2 = calc_var(e2);
  }

  double gamma = -v1 - v2;

  return gamma;
}


double orientation_calc::calc_var(const std::vector<double>& v)
{
  double sum  = std::accumulate(std::begin(v), std::end(v), 0.0);
  double mean = sum / v.size();

  double acc_var_num = 0.0;

  std::for_each(std::begin(v), std::end(v), [&](const double d) { acc_var_num += (d - mean) * (d - mean); });

  double var = sqrt(acc_var_num / (v.size() - 1));
/*-------------------*/
  // double v_mean = 0.0;
  // double v_sum = 0.0;    
  // double v_sqr_sum = 0.0; 
  // double v_std = 0.0;

  // for (size_t i = 0; i < v.size(); i++)
  // {
  //   v_sum += v[i];
  // }
  // v_mean = v_sum / v.size();

  // for (size_t i = 0; i < v.size(); i++)
  // {
  //   v_sqr_sum += (v[i] - v_mean) * (v[i] - v_mean);
  // }

  // v_std = sqrt(v_sqr_sum / (v.size()-1));

  // ROS_INFO("v_std-var=%f", v_std - var);

  return var;
}