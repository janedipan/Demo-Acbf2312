#include <mapping.h>
#include <raycast.h>
#include <termios.h>

using namespace std;

void MappingProcess::init(const ros::NodeHandle& nh)
{
    nh_ = nh;
    /*---------------       parameters            -----------------*/
    nh_.param("vehicle/cars_num", cars_num_, 1);
    nh_.param("vehicle/car_id", car_id_, 0);
    
    nh_.param("mapping/origin_x", origin_(0), -5.0);
    nh_.param("mapping/origin_y", origin_(1), 0.0);
    nh_.param("mapping/origin_z", origin_(2), 0.0);
    nh_.param("mapping/map_size_x", map_size_(0), 10.0);
    nh_.param("mapping/map_size_y", map_size_(1), 60.0);
    nh_.param("mapping/map_size_z", map_size_(2), 10.0);

    nh_.param("mapping/resolution", resolution_, 0.1);
    nh_.param("mapping/min_ray_length", min_ray_length_, 0.0);
    nh_.param("mapping/max_ray_length", max_ray_length_, 50.0);

    nh_.param("mapping/prob_hit_log", prob_hit_log_, 0.70);
    nh_.param("mapping/prob_miss_log", prob_miss_log_, 0.35);
    nh_.param("mapping/clamp_min_log", clamp_min_log_, 0.12);
    nh_.param("mapping/clamp_max_log", clamp_max_log_, 0.97);
    nh_.param("mapping/min_occupancy_log", min_occupancy_log_, 0.80);

    nh_.param("mapping/lidar_height", lidar_height_, 3.0);
    nh_.param("mapping/odometry_topic", odom_topic_, odom_topic_);
    nh_.param("mapping/lidar_topic", lidar_topic_, lidar_topic_);
    nh_.param("mapping/frame_id", map_frame_id_, map_frame_id_);
    /*---------------------------------------------------------------*/
    /*-------------------   settings   ------------------------*/
    have_odom_ = false;
    global_map_valid_ = false;
    local_map_valid_ = false;
    has_global_cloud_ = false;
    have_received_freespaces_ = false;

    resolution_inv_ = 1 / resolution_;
    for(int i=0;i<3;i++) 
    {
        global_map_size_(i) = ceil(map_size_(i) / resolution_);
    }

    lidar2car_ << 0.0, 0.0, lidar_height_;

    curr_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    global_map_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pred_map_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    min_range_ = origin_ - map_size_; 
    max_range_ = origin_ + map_size_;

    sensor_range_ << 10.0, 10.0, 2.0;         // 初始化局部地图范围：sensor_range_的值，以机器人为中心的矩形区域

    //inititalize size of buffer
    grid_size_y_multiply_z_ = global_map_size_(1) * global_map_size_(2);
    buffer_size_ = global_map_size_(0) * grid_size_y_multiply_z_;           // 三维地图size
    buffer_size_2d_ = global_map_size_(0) * global_map_size_(1);            // 二维地图size
    occupancy_buffer_.resize(buffer_size_);
    occupancy_buffer_2d_.resize(buffer_size_2d_);
    
    cache_all_.resize(buffer_size_);
    cache_hit_.resize(buffer_size_);
    cache_rayend_.resize(buffer_size_);
    cache_traverse_.resize(buffer_size_);
    raycast_num_ = 0;

    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1.0);         // 初始化
    fill(occupancy_buffer_2d_.begin(), occupancy_buffer_2d_.end(), -1.0);
    fill(cache_all_.begin(), cache_all_.end(), 0);
    fill(cache_hit_.begin(), cache_hit_.end(), 0);
    fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
    fill(cache_traverse_.begin(), cache_traverse_.end(), -1);

    
    /*********************************************************************************/
    Local_Pointcloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/velodyne_points", 10));
    Odometry_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/odometry", 100));
    pointcloud_odom_sync_.reset(new message_filters::Synchronizer<SyncPolicyPointcloud_Odom>(SyncPolicyPointcloud_Odom(100), *Local_Pointcloud_sub_, *Odometry_sub_));
    pointcloud_odom_sync_->registerCallback(boost::bind(&MappingProcess::OdometryAndPointcloud_cb, this, _1, _2));

    // 这种方式订阅里程和点云，在车辆速度较快时，会出现地图不对齐的情况！可能是点云和里程帧不对应导致的
    // odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometry", 100, &MappingProcess::odometry_cb, this);
    // pcl_sub_  = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &MappingProcess::pointcloud_cb, this);

    local_occ_vis_timer_ = nh_.createTimer(ros::Duration(0.1), &MappingProcess::localOccVis_cb, this);
    global_occ_vis_timer_ = nh_.createTimer(ros::Duration(1.0), &MappingProcess::globalOccVis_cb, this);

    MapCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    PointCloud_Odometry_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mapping/PointCloud_Odometry", 1);  // 投影到世界坐标下的点云
    curr_view_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mapping/local_view_cloud", 1);
    global_view_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mapping/global_view_cloud", 1);

    local_GridMap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("mapping/local_gridMap", 1);     // 发布nav_msgs::OccupancyGrid类型的局部地图
    global_GridMap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("mapping/global_gridMap", 1);

    std::cout << "MappingProcess Initialized!\n" << endl;
}


void MappingProcess::odometry_cb(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    have_odom_ = true;
    latest_odom_time_ = odom_msg->header.stamp;
    curr_posi_[0] = odom_msg->pose.pose.position.x;         // 机器人当前位置
    curr_posi_[1] = odom_msg->pose.pose.position.y;
    curr_posi_[2] = odom_msg->pose.pose.position.z;
    curr_twist_[0] = odom_msg->twist.twist.linear.x;
    curr_twist_[1] = odom_msg->twist.twist.linear.y;
    curr_twist_[2] = odom_msg->twist.twist.linear.z;
    curr_q_.w() = odom_msg->pose.pose.orientation.w;
    curr_q_.x() = odom_msg->pose.pose.orientation.x;
    curr_q_.y() = odom_msg->pose.pose.orientation.y;
    curr_q_.z() = odom_msg->pose.pose.orientation.z;
}

void MappingProcess::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
{
    local_map_valid_ = true;
    sensor_msgs::PointCloud2 pcl_msg_out;
    Eigen::Quaterniond quaternion(curr_q_.w(), curr_q_.x(), curr_q_.y(), curr_q_.z());
    Eigen::Matrix3d Rotation_matrix;
    Eigen::Vector3d Position_XYZ(curr_posi_[0], curr_posi_[1], curr_posi_[2]);
    
    Rotation_matrix = quaternion.toRotationMatrix();
    center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;     // 外参变换，获得lidar所在位置

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;                        // msg转pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, laserCloudIn);

    Eigen::Vector3d LaserCloudIn_XYZ;
    Eigen::Vector3d LaserCloudTransformed_XYZ;

    number_of_points_ = laserCloudIn.points.size();

    for(int i = 0; i < number_of_points_; i++)      // 将原始点云投影到世界系
    {
        LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
        LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
        LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
        LaserCloudTransformed_XYZ = Rotation_matrix * LaserCloudIn_XYZ + center_position_;
        laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0), LaserCloudTransformed_XYZ(1), LaserCloudTransformed_XYZ(2)));
    }

    /*This part is used to publish each frame of the laser pointcloud transformed*/
    pcl::toROSMsg(*laserCloudTransformed, pcl_msg_out);
    pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    pcl_msg_out.header.frame_id = map_frame_id_;
    PointCloud_Odometry_pub_.publish(pcl_msg_out);

    /*****************************************************************************************************************************/
    local_range_min_ = center_position_ - sensor_range_;
    local_range_max_ = center_position_ + sensor_range_;
    raycastProcess(center_position_, laserCloudTransformed);  // center_position_ is the postion of the lidar
}

void MappingProcess::OdometryAndPointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    have_odom_ = true;
    local_map_valid_ = true;
    latest_odom_time_ = odom_msg->header.stamp;

    curr_posi_[0] = odom_msg->pose.pose.position.x;         // 机器人当前位置
    curr_posi_[1] = odom_msg->pose.pose.position.y;
    curr_posi_[2] = odom_msg->pose.pose.position.z;
    curr_twist_[0] = odom_msg->twist.twist.linear.x;
    curr_twist_[1] = odom_msg->twist.twist.linear.y;
    curr_twist_[2] = odom_msg->twist.twist.linear.z;
    curr_q_.w() = odom_msg->pose.pose.orientation.w;
    curr_q_.x() = odom_msg->pose.pose.orientation.x;
    curr_q_.y() = odom_msg->pose.pose.orientation.y;
    curr_q_.z() = odom_msg->pose.pose.orientation.z;

    sensor_msgs::PointCloud2 pcl_msg_out;
    Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
                                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Matrix3d Rotation_matrix;
    Eigen::Vector3d Position_XYZ(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    
    Rotation_matrix = quaternion.toRotationMatrix();
    center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;     // 外参变换，获得lidar所在位置

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;                        // msg转pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, laserCloudIn);

    Eigen::Vector3d LaserCloudIn_XYZ;
    Eigen::Vector3d LaserCloudTransformed_XYZ;

    number_of_points_ = laserCloudIn.points.size();

    for(int i = 0; i < number_of_points_; i++)      // 将原始点云投影到世界系
    {
        LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
        LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
        LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
        LaserCloudTransformed_XYZ = Rotation_matrix * LaserCloudIn_XYZ + center_position_;
        laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0), LaserCloudTransformed_XYZ(1), LaserCloudTransformed_XYZ(2)));
    }

    /*This part is used to publish each frame of the laser pointcloud transformed*/
    pcl::toROSMsg(*laserCloudTransformed, pcl_msg_out);
    pcl_msg_out.header.stamp = pcl_msg->header.stamp;
    pcl_msg_out.header.frame_id = map_frame_id_;
    PointCloud_Odometry_pub_.publish(pcl_msg_out);

    /*****************************************************************************************************************************/
    local_range_min_ = center_position_ - sensor_range_;
    local_range_max_ = center_position_ + sensor_range_;
    raycastProcess(center_position_, laserCloudTransformed);  // center_position_ is the postion of the lidar
}

/*----------This function is used to display the local grid map in rviz---------------*/
void MappingProcess::localOccVis_cb(const ros::TimerEvent& e)
{
    // TODO: 就在这个函数上发布->机器人坐标系的局部2d占据栅格地图
    curr_view_cloud_ptr_->points.clear();
    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min_, min_id);
    posToIndex(local_range_max_, max_id);

    min_id(0) = max(0, min_id(0));
    min_id(1) = max(0, min_id(1));
    min_id(2) = max(0, min_id(2));
    max_id(0) = min(global_map_size_[0], max_id(0));
    max_id(1) = min(global_map_size_[1], max_id(1));
    max_id(2) = min(global_map_size_[2], max_id(2));

    Eigen::Vector2i local_origin_idx(min_id(0), min_id(1));
    Eigen::Vector2d local_origin;
    indexToPos2d(local_origin_idx, local_origin);

    nav_msgs::OccupancyGrid localmap_msg;
    localmap_msg.header.frame_id = "world";
    localmap_msg.info.resolution = resolution_;
    localmap_msg.info.origin.position.x = local_origin(0);
    localmap_msg.info.origin.position.y = local_origin(1);
    localmap_msg.info.origin.position.z = -0.5;
    localmap_msg.info.origin.orientation.w = 1.0;
    localmap_msg.info.origin.orientation.x = 0.0;
    localmap_msg.info.origin.orientation.y = 0.0;
    localmap_msg.info.origin.orientation.z = 0.0;
    localmap_msg.info.map_load_time = ros::Time::now();
    localmap_msg.info.height = max_id(1) - min_id(1);
    localmap_msg.info.width = max_id(0) - min_id(0);

    localmap_msg.data.resize(localmap_msg.info.height * localmap_msg.info.width);  // 初始化地图data的size和值
    std::fill(localmap_msg.data.begin(), localmap_msg.data.end(), 0);

    for (int x = min_id(0); x < max_id(0); ++x)
        for (int y = min_id(1); y < max_id(1); ++y)
            for (int z = min_id(2); z < max_id(2); ++z)
            {
                if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] > min_occupancy_log_)
                {
                    Eigen::Vector3i idx(x,y,z);
                    Eigen::Vector3d pos;
                    indexToPos(idx, pos);
                    pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
                    curr_view_cloud_ptr_->points.push_back(pc);     // 发布局部地图显示，pcl2格式

                    Eigen::Vector2i idx_2d(x, y);
                    Eigen::Vector2d pos_2d;
                    indexToPos2d(idx_2d, pos_2d);
                    int index_x = floor((pos_2d[0] - local_origin(0)) * resolution_inv_);
                    int index_y = floor((pos_2d[1] - local_origin(1)) * resolution_inv_);

                    localmap_msg.data[index_x + index_y * localmap_msg.info.width] = 100;   // 设置占用
                }
            }

    local_GridMap_pub_.publish(localmap_msg);       // 发布局部栅格地图

    curr_view_cloud_ptr_->width = curr_view_cloud_ptr_->points.size();
    curr_view_cloud_ptr_->height = 1;
    curr_view_cloud_ptr_->is_dense = true;
    curr_view_cloud_ptr_->header.frame_id = map_frame_id_; 
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*curr_view_cloud_ptr_, cloud_msg);
    curr_view_cloud_pub_.publish(cloud_msg);    
}

void MappingProcess::globalOccVis_cb(const ros::TimerEvent& e)
{
    history_view_cloud_ptr_->points.clear();

    nav_msgs::OccupancyGrid globalMap_msg;
    globalMap_msg.header.frame_id = "world";
    globalMap_msg.info.resolution = resolution_;
    globalMap_msg.info.origin.position.x = origin_(0);
    globalMap_msg.info.origin.position.y = origin_(1);
    globalMap_msg.info.origin.position.z = origin_(2);
    globalMap_msg.info.origin.orientation.w = 1.0;
    globalMap_msg.info.origin.orientation.x = 0.0;
    globalMap_msg.info.origin.orientation.y = 0.0;
    globalMap_msg.info.origin.orientation.z = 0.0;
    globalMap_msg.info.map_load_time = ros::Time::now();
    globalMap_msg.info.height = global_map_size_[0];
    globalMap_msg.info.width = global_map_size_[1];

    globalMap_msg.data.resize(globalMap_msg.info.height * globalMap_msg.info.width);  // 初始化地图data的size和值
    std::fill(globalMap_msg.data.begin(), globalMap_msg.data.end(), 0);

    for (int x = 0; x < global_map_size_[0]; ++x)
        for (int y = 0; y < global_map_size_[1]; ++y)
        {
            if (occupancy_buffer_2d_.at(y * global_map_size_(0) + x) > 0.5)
            {
                Eigen::Vector2i idx(x, y);
                Eigen::Vector2d pos;
                indexToPos2d(idx, pos);
                pcl::PointXYZ pc(pos[0], pos[1], 0.2);
                history_view_cloud_ptr_->points.push_back(pc);

                Eigen::Vector2i idx_2d(x, y);
                Eigen::Vector2d pos_2d;
                indexToPos2d(idx_2d, pos_2d);
                int index_x = floor((pos_2d[0] - origin_(0)) * resolution_inv_);
                int index_y = floor((pos_2d[1] - origin_(1)) * resolution_inv_);

                globalMap_msg.data[index_x + index_y * globalMap_msg.info.width] = 100;     // 设置占用
            }
        }

    global_GridMap_pub_.publish(globalMap_msg);       // 发布全局栅格地图

    history_view_cloud_ptr_->width = 1;
    history_view_cloud_ptr_->height = history_view_cloud_ptr_->points.size();
    history_view_cloud_ptr_->is_dense = true;
    history_view_cloud_ptr_->header.frame_id = map_frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*history_view_cloud_ptr_, cloud_msg);
    global_view_cloud_pub_.publish(cloud_msg);    
}


void MappingProcess::raycastProcess(const Eigen::Vector3d& t_wc, const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed) //t_wc is the position of the lidar
{
    if(number_of_points_ == 0)
        return;

    raycast_num_ += 1;
    int set_cache_idx;
    /*----------iterate over all points of a frame of pointcloud----------*/
    for(int i = 0; i < number_of_points_; i++)  
    {
        // 点云中的第 i 个点
        Eigen::Vector3d pt_w(laserCloudTransformed->points[i].x, 
                             laserCloudTransformed->points[i].y, 
                             laserCloudTransformed->points[i].z);

        bool inside_car = false;
        if(have_received_freespaces_)
        {
            for(int car = 0; car < cars_num_; car++)
            {
                if(car == car_id_) continue;

                Eigen::MatrixXd normalVec_and_points = vec_freespaces_[car];
                if((Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(0).tail(2))).dot(normalVec_and_points.col(0).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(1).tail(2))).dot(normalVec_and_points.col(1).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(2).tail(2))).dot(normalVec_and_points.col(2).head(2)) <= 0
                && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(3).tail(2))).dot(normalVec_and_points.col(3).head(2)) <= 0)
                {
                    inside_car = true;
                    break;
                }
            }
        }
        if(inside_car)
            continue;

        double length = (pt_w - t_wc).norm();       // 计算点云第 i 点到机器人的距离


        if (length < min_ray_length_)               // 小于min_ray_length_的不进行处理
            continue;
        else if (length > max_ray_length_)          // 大于max_ray_length_的视为空闲
        {
            pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc;
            set_cache_idx = setCacheOccupancy(pt_w, 0);
        }
        else                                        // 处于min_ray_length_和max_ray_length_间的点云，视为占用
            set_cache_idx = setCacheOccupancy(pt_w, 1);

        if(set_cache_idx != INVALID_IDX)
        {
            if(cache_rayend_[set_cache_idx] == raycast_num_)  // 疑惑
            {
                continue;
            }
            else   
                cache_rayend_[set_cache_idx] = raycast_num_;
        }

        RayCaster raycaster;
        bool need_ray = raycaster.setInput(pt_w / resolution_, t_wc / resolution_);
        if(!need_ray)
            continue;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d ray_pt;
        if(!raycaster.step(ray_pt))     // 多此一举？
            continue;
        while(raycaster.step(ray_pt))
        {
            Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
            set_cache_idx = setCacheOccupancy(tmp, 0);
            if(set_cache_idx != INVALID_IDX)
            {
                if(cache_traverse_[set_cache_idx] == raycast_num_)
                    break;
                else
                    cache_traverse_[set_cache_idx] = raycast_num_;
            }
        }
    }

    while(!cache_voxel_.empty())
    {
        Eigen::Vector3i idx = cache_voxel_.front();
        int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + idx(2);
        cache_voxel_.pop();

        double log_odds_update =
            cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
        cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

        if ((log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) ||
            (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_))
            continue;

        occupancy_buffer_[idx_ctns] =
            std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
            // std::min(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_max_log_);

        if(occupancy_buffer_[idx_ctns] > min_occupancy_log_)
        {
            int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
            occupancy_buffer_2d_[idx_ctns_2d] = 1;
        }
        else
        {
            int number_of_freespaces_in_z = 0;
            for(int z = 0; z < global_map_size_(2); z++)
            {
                int idx_ctns_3d = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + z;
                if(occupancy_buffer_[idx_ctns_3d] < min_occupancy_log_)
                {
                    number_of_freespaces_in_z++;
                }
                else
                {
                    break;
                }
            }
            if(number_of_freespaces_in_z == global_map_size_(2))
            {
                int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
                occupancy_buffer_2d_[idx_ctns_2d] = 0;
            }
        }

    }
}

int MappingProcess::setCacheOccupancy(const Eigen::Vector3d& pos, int occ)
{
    if (occ != 1 && occ != 0)
    {
        return INVALID_IDX;
    }

    Eigen::Vector3i id;
    posToIndex(pos, id);

    if (!isInMap(id))
    {
        return INVALID_IDX;
    }

    int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2);

    cache_all_[idx_ctns] += 1;

    if (cache_all_[idx_ctns] == 1)
    {
        cache_voxel_.push(id);
    }

    if (occ == 1)
        cache_hit_[idx_ctns] += 1;

    return idx_ctns;    
}

inline bool MappingProcess::isInMap(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return isInMap(idx);
}

inline bool MappingProcess::isInMap(const Eigen::Vector3i &id)
{
    return ((id[0] | (global_map_size_[0] - 1 - id[0]) | id[1] | (global_map_size_[1] - 1 - id[1]) | id[2]| (global_map_size_[2] - 1 - id[2])) >= 0);
};

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idx;
  posToIndex(pos, idx);
  return isInLocalMap(idx);
}

inline bool MappingProcess::isInLocalMap(const Eigen::Vector3i &id)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min_, min_id);
  posToIndex(local_range_max_, max_id);
  min_id(0) = max(0, min_id(0));
  min_id(1) = max(0, min_id(1));
  min_id(2) = max(0, min_id(2));
  max_id(0) = min(global_map_size_[0], max_id(0));
  max_id(1) = min(global_map_size_[1], max_id(1));
  max_id(2) = min(global_map_size_[2], max_id(2));
  return (((id[0] - min_id[0]) | (max_id[0] - id[0]) | (id[1] - min_id[1]) | (max_id[1] - id[1]) | (id[2] - min_id[2]) | (max_id[2] - id[2])) >= 0);
};

bool MappingProcess::isInMap2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i idx;
    posToIndex2d(pos, idx);
    return isInMap2d(idx);
}

bool MappingProcess::isInMap2d(const Eigen::Vector2i &id)
{
    if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
    {
        return false;
    }
    else
        return true;
};

void MappingProcess::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
{
    for(int i = 0; i < 2; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
    // pos = origin_;
    for(int i = 0; i < 2; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

int MappingProcess::getVoxelState2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex2d(pos, id);
    if(!isInMap2d(id))
        return -1;
    // todo: add local map range

    return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)] > 0.5 ? 1 : 0;
}

void MappingProcess::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for(int i = 0; i < 3; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
    pos = origin_;
    for(int i = 0; i < 3; i++)
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
}

int MappingProcess::getVoxelState(const Eigen::Vector3d &pos)
{
    Eigen::Vector3i id;
    posToIndex(pos, id);
    if(!isInMap(id))
        return -1;
    if(!isInLocalMap(id))
        return 0;

    return occupancy_buffer_[id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2)] > min_occupancy_log_ ? 1 : 0;
}

void MappingProcess::setFreeSpacesForMapping(const std::vector<Eigen::MatrixXd>& vec_freespaces)
{
    vec_freespaces_ = vec_freespaces;
    have_received_freespaces_ = true;
}