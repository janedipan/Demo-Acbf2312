#include "dynamic_perception/ri_dbscan.h"

class cloud_segmentation
{
 private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  cv::Mat rangeMat; // range matrix for range image
  cv::Mat labelMat; // label matrix for segmentaiton marking

  std::vector<float> vert_angles_;
  std::vector<int> index_v;
  std::vector<int> runtime_vector;

  pcl::PointCloud<PointType>::Ptr laserCloudIn;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;

  PointType nanPoint; // fill in fullCloud at each iteration

  std_msgs::Header cloudHeader;
  pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix

  uint32_t *queueIndX; // array for breadth-first search process of segmentation, for speed
  uint32_t *queueIndY;

  uint32_t *allPushedIndX; // array for tracking points of a segmented object
  uint32_t *allPushedIndY;

  std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

  int labelCount;

  std::vector<pcl::PointIndices> cluster_indices;
  
  std::vector<std::vector<int> > clusterIndices;
  std::vector<std::vector<int> > clusterIndices_ri;
  std::vector<std::vector<int> > gt_clusterIndices;  

  std::vector<double> ose_vector;
  std::vector<double> use_vector;

  dynamic_reconfigure::Server<ri_dbscan::ri_dbscan_Config> server;
  dynamic_reconfigure::Server<ri_dbscan::ri_dbscan_Config>::CallbackType f;

  pcl::PointCloud<pcl::PointXYZI>::Ptr segment_visul;

  ros::Subscriber sub_lidar_points;
  ros::Publisher pub_colored_cluster_cloud_;
  ros::Publisher pub_obsdet_clusters_;
  ros::Publisher pub_segment_visul_;
  ros::Publisher pub_static_pcloud_;

  int getRowIdx(PointType pt);

  int getColIdx(PointType pt);

  void sphericalProjection(const sensor_msgs::PointCloud2::ConstPtr &laserRosCloudMsg);

  void range_image_search(int row, int col, std::vector<int> &k_indices, bool &is_core_point);
  void RI_DBSCAN();
  void calculate_index2rc(int index, int &i, int &j);
  void expandCluster(std::vector<int> &curCluster, const std::vector<int> &neighbors,
                     std::vector<char> &visited, std::vector<char> &isNoise);
  void Mainloop(const sensor_msgs::PointCloud2::ConstPtr &lidar_points);
  void postSegment(dynamic_perception::CloudClusterArray &cluster_array);

  void postSegment2(dynamic_perception::CloudClusterArray& cluster_array);

  void clusterIndices_Trans();
  void eval_running_time(int running_time);
  void eval_USE();
  void eval_OSE();

public:
  cloud_segmentation();
  ~cloud_segmentation() {};

  void allocateMemory(){
    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    segment_visul.reset(new pcl::PointCloud<pcl::PointXYZI>());
    fullCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(VERT_SCAN * HORZ_SCAN);

    index_v.resize(VERT_SCAN * HORZ_SCAN);
    queueIndX = new uint32_t[VERT_SCAN * HORZ_SCAN];
    queueIndY = new uint32_t[VERT_SCAN * HORZ_SCAN];

    allPushedIndX = new uint32_t[VERT_SCAN * HORZ_SCAN];
    allPushedIndY = new uint32_t[VERT_SCAN * HORZ_SCAN];
  }

  void resetParameters(){
    laserCloudIn->clear();
    segment_visul->clear();
    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);

    labelMat = cv::Mat(VERT_SCAN, HORZ_SCAN, CV_32S, cv::Scalar::all(0));
    rangeMat = cv::Mat(VERT_SCAN, HORZ_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    
    labelCount = 1;
    cluster_indices.clear();
    index_v.clear();
    clusterIndices_ri.clear();
    clusterIndices.clear();
    gt_clusterIndices.clear();  
  }
};

// Dynamic parameter server callback function
void dynamicParamCallback(ri_dbscan::ri_dbscan_Config &config, uint32_t level)
{
  // Pointcloud Filtering Parameters
  DETECT_MIN = config.detect_min;
  DETECT_MAX = config.detect_max;

  cvc_coef = config.cvc_coef;
  MinClusterSize = config.MinClusterSize;
  MaxClusterSize = config.MaxClusterSize;
  // ROS_ERROR("------------------------------------------------------");
  // std::cout << "config.detect_min:= " << config.detect_min << std::endl;
  // std::cout << "config.detect_max:= " << config.detect_max << std::endl;
  // std::cout << "config.cvc_coef:= " << config.cvc_coef << std::endl;
  // std::cout << "config.MinClusterSize:= " << config.MinClusterSize << std::endl;
  // std::cout << "config.MaxClusterSize:= " << config.MaxClusterSize << std::endl;
  // ROS_ERROR("------------------------------------------------------");

}

cloud_segmentation::cloud_segmentation():private_nh("~")
{
  allocateMemory();

  /* Initialize tuning parameter */
  private_nh.param<std::string>("non_ground_cloud_topic", non_ground_cloud_topic_, "/semi_kitti/non_ground_pc");
  ROS_INFO("non_ground_cloud_topic: %s", non_ground_cloud_topic_.c_str());

  private_nh.param<std::string>("output_frame", output_frame_, "velodyne_1");
  ROS_INFO("output_frame: %s", output_frame_.c_str());

  private_nh.param<std::string>("colored_cloud_topic", colored_cloud_topic_, "/clustering/colored_cloud");
  ROS_INFO("colored_cloud_topic: %s", colored_cloud_topic_.c_str());

  private_nh.param<std::string>("output_cluster_array_topic", output_cluster_array_topic_, "/clustering/cloudcluster_array");
  ROS_INFO("output_cluster_array_topic: %s", output_cluster_array_topic_.c_str());

  float resolution = (float)(MAX_VERT_ANGLE - MIN_VERT_ANGLE) / (float)(VERT_SCAN - 1);
  for (int i = 0; i < VERT_SCAN; i++)
    vert_angles_.push_back(MIN_VERT_ANGLE + i * resolution);

  sub_lidar_points = nh.subscribe(non_ground_cloud_topic_, 1, &cloud_segmentation::Mainloop, this);

  pub_colored_cluster_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(colored_cloud_topic_, 1);
  pub_obsdet_clusters_ = nh.advertise<dynamic_perception::CloudClusterArray >(output_cluster_array_topic_, 1);

  pub_segment_visul_ = nh.advertise<sensor_msgs::PointCloud2> ("/clustering/colored_cluster", 1);

  time_rviz_pub_ = nh.advertise<std_msgs::Float32>("/clustering/time_rviz", 1);

  // pub_static_pcloud_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/clustering/static/points", 1);

  input_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // Dynamic Parameter Server & Function
  
  // f = boost::bind(&dynamicParamCallback, _1, _2);
  // server.setCallback(f);

  // Create point processor
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();
  nanPoint.intensity = -1;

  resetParameters();
}

int cloud_segmentation::getRowIdx(PointType pt)
{
  float angle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
  auto iter_geq = std::lower_bound(vert_angles_.begin(), vert_angles_.end(), angle);
  int row_idx;

  if (iter_geq == vert_angles_.begin())
  {
    row_idx = 0;
  }
  else
  {
    float a = *(iter_geq - 1);
    float b = *(iter_geq);
    if (fabs(angle - a) < fabs(angle - b))
    {
      row_idx = iter_geq - vert_angles_.begin() - 1;
    }
    else
    {
      row_idx = iter_geq - vert_angles_.begin();
    }
  }
  return row_idx;
}

int cloud_segmentation::getColIdx(PointType pt)
{
  float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
  static float ang_res_x = 360.0 / float(HORZ_SCAN);
  int col_idx = -round((horizonAngle - 90.0) / ang_res_x) + HORZ_SCAN / 2;
  if (col_idx >= HORZ_SCAN)
    col_idx -= HORZ_SCAN;
  return col_idx;
}

void cloud_segmentation::range_image_search(int row, int col, std::vector<int> &k_indices, bool &is_core_point)
{
  k_indices.clear();
  k_indices.push_back(col + row * HORZ_SCAN);

  float dist, cvc_dist, d_criteria;

  int fromIndX, fromIndY, thisIndX, thisIndY;

  fromIndX = row;
  fromIndY = col;

  std::pair<int8_t, int8_t> neighbor;

  // d_criteria = cvc_coef * rangeMat.at<float>(fromIndX, fromIndY) * (neighbor_col * 0.08) * M_PI / 180;

  d_criteria = 0.4;

  neighborIterator.clear();

  int neighbor_col = 10;

  for (int8_t i = -2; i <= 2; i++)
  {
    for (int8_t j = -neighbor_col; j <= neighbor_col; j++)
    {
      neighbor.first = i;
      neighbor.second = j;
      neighborIterator.push_back(neighbor);
    }
  }

  // Loop through all the neighboring grids of popped grid
  for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
  {
    // new index
    thisIndX = fromIndX + (*iter).first;
    thisIndY = fromIndY + (*iter).second;

    // index should be within the boundary
    if (thisIndX < 0 || thisIndX >= VERT_SCAN)
      continue;

    // at range image margin (left or right side)
    if (thisIndY < 0)
      thisIndY = HORZ_SCAN - 1;
    if (thisIndY >= HORZ_SCAN)
      thisIndY = 0;

    // prevent infinite loop (caused by put already examined point back)
    if (labelMat.at<int>(thisIndX, thisIndY) != -1)
      continue;

    cvc_dist = abs(rangeMat.at<float>(thisIndX, thisIndY) - rangeMat.at<float>(fromIndX, fromIndY));

    if (cvc_dist < d_criteria)
    {
      k_indices.push_back(thisIndY + thisIndX * HORZ_SCAN);
    }
  }

  if (k_indices.size() > 20)
    is_core_point = true;
}


void cloud_segmentation::expandCluster(std::vector<int> &curCluster, const std::vector<int> &neighbors,
                                       std::vector<char> &visited, std::vector<char> &isNoise)
{
  int i_, j_;
  bool is_core_point;
  std::deque<int> neighborDeque(neighbors.begin(), neighbors.end());

  while (!neighborDeque.empty()) {
    int curIdx = neighborDeque.front();
    neighborDeque.pop_front();

    if (isNoise[curIdx]) {
      curCluster.emplace_back(curIdx);
      continue;
    }

    if (!visited[curIdx]) {
      visited[curIdx] = true;
      curCluster.emplace_back(curIdx);
  
      std::vector<int> curNeighbors;
      std::vector<float> nn_distances;

      calculate_index2rc(curIdx, i_, j_);

      range_image_search(i_, j_, curNeighbors, is_core_point);

      if (!is_core_point){
        continue;
      }

      std::copy(curNeighbors.begin(), curNeighbors.end(), std::back_inserter(neighborDeque));
    }
  }
}

// segmentation process
void cloud_segmentation::RI_DBSCAN()
{
  std::vector<int> core_pt_indices;
  bool is_core_point = false;

  std::vector<char> visited(VERT_SCAN * HORZ_SCAN, false);
  std::vector<char> isNoise(VERT_SCAN * HORZ_SCAN, false);

  for (size_t i = 0; i < VERT_SCAN; ++i)
  {
    for (size_t j = 0; j < HORZ_SCAN; ++j)
    {
      int index_ = j + i * HORZ_SCAN;

      if (visited[index_]){
        continue;
      }

      visited[index_] = true;

      if (labelMat.at<int>(i, j) == -1){
        range_image_search(i, j, core_pt_indices, is_core_point);
      }
      else{continue;};

      if (!is_core_point){
        isNoise[index_] = true;
      }
      else{
        clusterIndices_ri.emplace_back(std::vector<int>{static_cast<int>(index_)});
        expandCluster(clusterIndices_ri.back(), core_pt_indices, visited, isNoise);
      }
    }
  }
}

void cloud_segmentation::calculate_index2rc(int index, int &r, int &c){
  int j;
  for (int i = 0; i < VERT_SCAN; ++i)
  {
    j = index - i * HORZ_SCAN;
    if (j <= HORZ_SCAN){
      r=i;
      c=j;
      break;
    }
  }
}

void cloud_segmentation::postSegment(dynamic_perception::CloudClusterArray &cluster_array)
{
  int intensity_mark = 1;

  pcl::PointXYZI cluster_color;

  dynamic_perception::CloudCluster cluster_pc;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pcl(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto &getIndices : clusterIndices)
  {
    // if (getIndices.size() < 10)
    //   continue;

    cluster_pcl->clear();
    for (auto &index : getIndices)
    {
      cluster_color.x = laserCloudIn->points[index].x;
      cluster_color.y = laserCloudIn->points[index].y;
      cluster_color.z = laserCloudIn->points[index].z;
      cluster_color.intensity = intensity_mark;
      segment_visul->push_back(cluster_color);

      cluster_pcl->push_back(cluster_color);
    }
    intensity_mark++;

    cloud_msg.header = ros_header;
    pcl::toROSMsg(*cluster_pcl, cloud_msg);

    cluster_pc.header = ros_header;
    cluster_pc.cloud = cloud_msg;
    cluster_array.clusters.push_back(cluster_pc);
  }

  sensor_msgs::PointCloud2 segment_visul_ros;
  pcl::toROSMsg(*segment_visul, segment_visul_ros);
  segment_visul_ros.header = ros_header;
  pub_segment_visul_.publish(segment_visul_ros);

  cluster_array.header = ros_header;
  pub_obsdet_clusters_.publish(cluster_array);
}

void cloud_segmentation::postSegment2(dynamic_perception::CloudClusterArray &cluster_array)
{
  int intensity_mark = 1;

  pcl::PointXYZI cluster_color;
  dynamic_perception::CloudCluster cluster_pc;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pcl(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto &getIndices : clusterIndices){
    cluster_pcl->clear();

    // 获取聚类的体积大小，过滤掉过大的物体
    double min_x = std::numeric_limits<double>::max(); double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max(); double max_y = -std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max(); double max_z = -std::numeric_limits<double>::max();

    for(auto &index : getIndices){  //遍历，找到聚类的box边界
      if (laserCloudIn->points[index].x <= min_x){  min_x = laserCloudIn->points[index].x;  }
      if (laserCloudIn->points[index].y <= min_y){  min_y = laserCloudIn->points[index].y;  }
      if (laserCloudIn->points[index].z <= min_z){  min_z = laserCloudIn->points[index].z;  }
      if (laserCloudIn->points[index].x >= max_x){  max_x = laserCloudIn->points[index].x;  }
      if (laserCloudIn->points[index].y >= max_y){  max_y = laserCloudIn->points[index].y;  }
      if (laserCloudIn->points[index].z >= max_z){  max_z = laserCloudIn->points[index].z;  }
    }

    double sum_x = 0.0; double sum_y = 0.0; double sum_z = 0.0;
    for (auto &index : getIndices){
      sum_x += laserCloudIn->points[index].x;
      sum_y += laserCloudIn->points[index].y;
      sum_z += laserCloudIn->points[index].z;
    }

    double mean_x = sum_x / getIndices.size();   //计算聚类中所有点的平均位置，来代表聚类的中心位置
    double mean_y = sum_y / getIndices.size();
    double mean_z = sum_z / getIndices.size();

    Eigen::Vector3d boundingbox = Eigen::Vector3d(2 * (std::max(fabs(max_x - mean_x), fabs(min_x - mean_x))),
                                                  2 * (std::max(fabs(max_y - mean_y), fabs(min_y - mean_y))),
                                                  2 * (std::max(fabs(max_z - mean_z), fabs(min_z - mean_z))));
    // 过滤体积过大的物体
    if(boundingbox[0] > 2.0 || boundingbox[1] > 2.0 || boundingbox[2] > 2.0){
      continue;
    }

    if(boundingbox[0] < 0.2 || boundingbox[1] < 0.2 || boundingbox[2] < 0.5){
      continue;
    }
    
    // 获取每个聚类的点云
    for (auto &index : getIndices){
      cluster_color.x = laserCloudIn->points[index].x;
      cluster_color.y = laserCloudIn->points[index].y;
      cluster_color.z = laserCloudIn->points[index].z;
      cluster_color.intensity = intensity_mark;
      segment_visul->push_back(cluster_color);
      cluster_pcl->push_back(cluster_color);
    }
    intensity_mark++;

    cloud_msg.header = ros_header;
    pcl::toROSMsg(*cluster_pcl, cloud_msg);

    cluster_pc.header = ros_header;
    cluster_pc.cloud = cloud_msg;
    cluster_array.clusters.push_back(cluster_pc);

    // 删除动态障碍物点云,好像没有用？？？
    // pcl::ExtractIndices<pcl::PointXYZ> extract_dynamic;
    // pcl::IndicesPtr index_ptr = boost::make_shared<std::vector<int>>(getIndices);
    // extract_dynamic.setInputCloud(input_cloud_);
    // extract_dynamic.setIndices(index_ptr);
    // extract_dynamic.setNegative(true);
    // extract_dynamic.filter(*input_cloud_);
  }

  // 发布聚类点云
  sensor_msgs::PointCloud2 segment_visul_ros;
  pcl::toROSMsg(*segment_visul, segment_visul_ros);
  segment_visul_ros.header = ros_header;
  pub_segment_visul_.publish(segment_visul_ros);

  // 发布聚类消息
  cluster_array.header = ros_header;
  pub_obsdet_clusters_.publish(cluster_array);

  // 发布静态点云
  // sensor_msgs::PointCloud2 static_pcl2_msg;
  // pcl::toROSMsg(*input_cloud_, static_pcl2_msg);
  // static_pcl2_msg.header = ros_header;
  // pub_static_pcloud_.publish(static_pcl2_msg);

}

void cloud_segmentation::sphericalProjection(const sensor_msgs::PointCloud2::ConstPtr &laserRosCloudMsg)
{
  float range;
  size_t rowIdn, columnIdn, index, cloudSize; 
  PointType curpt;

  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl(new pcl::PointCloud<pcl::PointXYZI>);

  cloudHeader = laserRosCloudMsg->header;
  cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
  pcl::fromROSMsg(*laserRosCloudMsg, *raw_pcl);

  cloudSize = raw_pcl->points.size();

  int oindex = 0;
  laserCloudIn->clear();

  for (size_t i = 0; i < cloudSize; ++i)
  {
    curpt.x = raw_pcl->points[i].x;
    curpt.y = raw_pcl->points[i].y;
    curpt.z = raw_pcl->points[i].z;
    curpt.intensity = raw_pcl->points[i].intensity;
    // curpt.ring = raw_pcl->points[i].ring;
    // curpt.id = raw_pcl->points[i].id;
    // curpt.label = raw_pcl->points[i].label;
    // curpt.cid = 9999;

    bool is_nan = std::isnan(curpt.x) || std::isnan(curpt.y) || std::isnan(curpt.z);
    if (is_nan)
      continue;

    //find the row and column index in the image for this point
    rowIdn = getRowIdx(curpt);

    if (rowIdn < 0 || rowIdn >= VERT_SCAN)
      continue;

    columnIdn = getColIdx(curpt);

    if (columnIdn < 0 || columnIdn >= HORZ_SCAN)
      continue;

    range = sqrt(curpt.x * curpt.x + curpt.y * curpt.y + curpt.z * curpt.z);

    labelMat.at<int>(rowIdn, columnIdn) = -1;
    rangeMat.at<float>(rowIdn, columnIdn) = range;

    index = columnIdn  + rowIdn * HORZ_SCAN;

    fullCloud->points[index] = curpt;

    index_v[index] = oindex;

    laserCloudIn->points.push_back(curpt);
    oindex++;
  }
}

void cloud_segmentation::clusterIndices_Trans()
{
  int row, col;
  std::vector<int> clusterIndice;

  for (auto &getIndices : clusterIndices_ri)
  {
    clusterIndice.clear();
    for (auto &index : getIndices)
    {
      calculate_index2rc(index, row, col);
      clusterIndice.push_back(index_v[col + row * HORZ_SCAN]);
    }
    clusterIndices.push_back(clusterIndice);
  }
}

void cloud_segmentation::eval_OSE()
{
  double ose_i = 0.0;

  int cluster_id = 0;

  for (auto &getIndices : clusterIndices)
  {
    for (auto &index : getIndices)
    {
      laserCloudIn->points[index].cid = cluster_id;
    }
    cluster_id++;
  }

  for (auto &getIndices : gt_clusterIndices)
  {
    int object_cluster[clusterIndices.size()] = {0};
    int N = getIndices.size();

    for (auto &index : getIndices)
    {
      if (laserCloudIn->points[index].cid != 9999)
        object_cluster[laserCloudIn->points[index].cid]++;
    }

    for (size_t i = 0; i < clusterIndices.size(); i++)
    {
      if (object_cluster[i] == 0)
        continue;

      double ose_ii = -(object_cluster[i] / (1.0*N)) * log(object_cluster[i] / (1.0*N));

      ose_i += ose_ii;
    }
  }

  ose_vector.push_back(ose_i);

  double ose_total_v = 0.0;
  double ose_sqr_sum = 0.0;
  double ose_mean;
  double ose_std;

  for (size_t i = 0; i < ose_vector.size(); i++)
  {
    ose_total_v += ose_vector[i];
  }
  ose_mean = ose_total_v / ose_vector.size();

  for (size_t i = 0; i < ose_vector.size(); i++)
  {
    ose_sqr_sum += (ose_vector[i] - ose_mean) * (ose_vector[i] - ose_mean);
  }
  ose_std = sqrt(ose_sqr_sum / ose_vector.size());

  std::cout << "current ose_i is = " << ose_i << std::endl;
  std::cout << "\033[1;34mose_mean is = " << ose_mean << "\033[0m" << std::endl;
  std::cout << "ose_std is = " << ose_std << std::endl;
}

void cloud_segmentation::eval_USE()
{
  std::vector<int> cluater_label;
  std::vector<std::vector<int> > cluaters_label;

  int label[34] = {0, 1, 10, 11, 13, 15, 16, 18, 20, 30, 31, 32, 40, 44, 48, 49, 50, 51,
                   52, 60, 70, 71, 72, 80, 81, 99, 252, 253, 254, 255, 256, 257, 258, 259};

  double use_i_sum = 0;

  for (auto& getIndices : clusterIndices)
  {
    int cluster_label[34] = {0};
    for (auto& index : getIndices)
    {
      for (size_t i = 0; i < 34; i++)
      {
        if (laserCloudIn->points[index].label == label[i])
          cluster_label[i]++;
      }
    }

    int M = getIndices.size();

    for (size_t i = 0; i < 34; i++)
    {
      if (cluster_label[i] == 0)
        continue;

      double use_i = -(cluster_label[i] / (M * 1.0)) * log(cluster_label[i] / (M * 1.0));
      use_i_sum += use_i;
    }
  }

  use_vector.push_back(use_i_sum);  

  double use_total_v = 0.0;
  double use_sqr_sum = 0.0;
  double use_mean;
  double use_std;

  for (size_t i = 0; i < use_vector.size(); i++)
  {
    use_total_v += use_vector[i];
  }
  use_mean = use_total_v / use_vector.size();

  for (size_t i = 0; i < use_vector.size(); i++)
  {
    use_sqr_sum += (use_vector[i] - use_mean) * (use_vector[i] - use_mean);
  }

  use_std = sqrt(use_sqr_sum / use_vector.size());

  std::cout << "current use_i is = " << use_i_sum << std::endl;
  std::cout << "\033[1;32muse_mean is = " << use_mean << "\033[0m" << std::endl;
  std::cout << "use_std is = " << use_std << std::endl;
}


void cloud_segmentation::eval_running_time(int running_time)
{
  double runtime_std;
  double runtime_aver;
  double runtime_total_v = 0.0;
  double runtime_sqr_sum = 0.0;

  runtime_vector.push_back(running_time);

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
  std::cout << "current running_time is = " << running_time << "ms" << std::endl;
  std::cout << "\033[1;36mruntime_aver is = " << runtime_aver << "ms"
            << "\033[0m" << std::endl;
  std::cout << "runtime_std is = " << runtime_std << "ms" << std::endl;
}


void cloud_segmentation::Mainloop(const sensor_msgs::PointCloud2::ConstPtr& lidar_points)
{
  ros_header = lidar_points->header;

  // pcl::fromROSMsg(*lidar_points, *input_cloud_);   // 从ROS的点云消息转换为PCL格式

  gt_verify.reset(new groundtruth::DepthCluster<PointType>());

  const auto start_time = std::chrono::steady_clock::now();

  sphericalProjection(lidar_points);

  RI_DBSCAN();

  // Time the whole process
  const auto end_time = std::chrono::steady_clock::now();  
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  eval_running_time(elapsed_time.count());

  // get the ground truth
  // gt_verify->GTV(laserCloudIn, gt_clusterIndices);
  clusterIndices_Trans();

  // eval_OSE();  // evaluate over-segmentation
  // eval_USE();  // evaluate under-segmentation

  //visualization, use indensity to show different color for each cluster.
  dynamic_perception::CloudClusterArray cluster_array;
  postSegment2(cluster_array);

  resetParameters();

  // Time the whole process
  const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
  time_rviz.data = exe_time;
  time_rviz_pub_.publish(time_rviz);
  std::cout << "---------------------------------" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spc_node");

  // tf::StampedTransform transform;
  // tf::TransformListener listener;

  // _transform = &transform;
  // _transform_listener = &listener;

  cloud_segmentation cloud_segmentation_node;

  ros::spin();

  return 0;
}