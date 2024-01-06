#ifndef GROUND_TRUTH_H
#define GROUND_TRUTH_H

#pragma once
#define PCL_NO_PRECOMPILE
#include <vector>
#include <algorithm>
#include <iostream>
#include <list>
#include <numeric>
#include <random>
#include <chrono>
#include <forward_list>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include "tools/utils.hpp"

// VLP-16
const int HORZ_SCAN = 1800;
const int VERT_SCAN = 16;
const float MAX_VERT_ANGLE = 15.0;
const float MIN_VERT_ANGLE = -15.0;

// Kitti
// const int HORZ_SCAN = 2048;
// const int VERT_SCAN = 64;
// const float MAX_VERT_ANGLE = 3.0;
// const float MIN_VERT_ANGLE = -25.0;
// horizen 0.08 vetical 0.4

namespace groundtruth {
    template <typename PointT>
    class DepthCluster {
        private:
            uint16_t max_label_;

            std::vector<float> vert_angles_;
            std::vector<std::vector<int>> clusterIndices;

            std::vector<int> index_v;

            uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
            uint16_t *queueIndY;

            uint16_t *allPushedIndX; // array for tracking points of a segmented object
            uint16_t *allPushedIndY;

            int labelCount;
            std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process
            
            cv::Mat rangeMat; // range matrix for range image
            cv::Mat labelMat; // label matrix for segmentaiton marking

            PointT nanPoint; // fill in fullCloud at each iteration
            typename pcl::PointCloud<PointT>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix

        public:
            DepthCluster() {}

            ~DepthCluster() {}

            void allocateMemory()
            {
                // clusterIndices.clear();
                fullCloud.reset(new pcl::PointCloud<PointT>());
                fullCloud->points.resize(VERT_SCAN*HORZ_SCAN);
                index_v.resize(VERT_SCAN * HORZ_SCAN);
                nanPoint.x = std::numeric_limits<float>::quiet_NaN();
                nanPoint.y = std::numeric_limits<float>::quiet_NaN();
                nanPoint.z = std::numeric_limits<float>::quiet_NaN();
                nanPoint.intensity = -1;
                nanPoint.id = 0;
                nanPoint.label = 0;
                nanPoint.cid = 0;

                queueIndX = new uint16_t[VERT_SCAN*HORZ_SCAN];
                queueIndY = new uint16_t[VERT_SCAN*HORZ_SCAN];

                allPushedIndX = new uint16_t[VERT_SCAN*HORZ_SCAN];
                allPushedIndY = new uint16_t[VERT_SCAN*HORZ_SCAN];

                std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
                labelMat = cv::Mat(VERT_SCAN, HORZ_SCAN, CV_32S, cv::Scalar::all(0));
                rangeMat = cv::Mat(VERT_SCAN, HORZ_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
                labelCount = 1;

                std::pair<int8_t, int8_t> neighbor;

                int8_t neighbor_col = 60;
                
                for (int8_t i = -4; i <= 4; i++)
                {
                    for (int8_t j = -neighbor_col; j <= neighbor_col; j++)
                    {
                        neighbor.first = i;
                        neighbor.second = j;
                        neighborIterator.push_back(neighbor);
                    }
                }
                float resolution = (float)(MAX_VERT_ANGLE - MIN_VERT_ANGLE) / (float)(VERT_SCAN - 1);
                for (int i = 0; i < VERT_SCAN; i++)
                    vert_angles_.push_back(MIN_VERT_ANGLE + i * resolution);
            }

            void setParams(int _vert_scan, int _horz_scan, float _min_range, float _max_range,
                            float _min_vert_angle, float _max_vert_angle,
                            float _horz_merge_thres, float _vert_merge_thres, int _vert_scan_size,
                            int _horz_scan_size, int _horz_extension_size, int _horz_skip_size,
                            boost::optional<int> _min_cluster_size=boost::none, 
                            boost::optional<int> _max_cluster_size=boost::none) {
   
                std::cout<<""<<std::endl;
                ROS_WARN("Set ObjectSeg Parameters");

                // range_mat_.resize(VERT_SCAN, vector<Point>(HORZ_SCAN));

                // valid_cnt_.resize(VERT_SCAN, 0);
                // nodes_.resize(VERT_SCAN, vector<AOSNode>());

                // if (_min_cluster_size)
                //     MIN_CLUSTER_SIZE = _min_cluster_size;
            
                // if (_max_cluster_size)
                //     MAX_CLUSTER_SIZE = _max_cluster_size;
            }

            void GTV(boost::shared_ptr<pcl::PointCloud<PointT>> cloud_in, std::vector<std::vector<int>> &clusterIndices)
            {
                // 0. reset
                allocateMemory();
                clusterIndices.clear();

                // 1. do spherical projection
                sphericalProjection(cloud_in);

                // 2. begin clustering
                for (size_t i = 0; i < VERT_SCAN; ++i)
                {
                    for (size_t j = 0; j < HORZ_SCAN; ++j)
                    {
                        if (labelMat.at<int>(i, j) == -1)
                            labelComponents(i, j, clusterIndices);
                    }
                }
                // return clusterIndices;
            }

            void sphericalProjection(boost::shared_ptr<pcl::PointCloud<PointT>> cloud_in) 
            {
                float range;
                size_t rowIdn, columnIdn, index, cloudSize; 
                PointT cpt;

                cloudSize = cloud_in->points.size();
                int org_index = 0;

                for (size_t i = 0; i < cloudSize; ++i)
                {
                    cpt.x = cloud_in->points[i].x;
                    cpt.y = cloud_in->points[i].y;
                    cpt.z = cloud_in->points[i].z;
                    cpt.intensity = cloud_in->points[i].intensity;
                    cpt.id = cloud_in->points[i].id;
                    cpt.label = cloud_in->points[i].label;

                    bool is_nan = std::isnan(cpt.x) || std::isnan(cpt.y) || std::isnan(cpt.z);
                    if (is_nan) {continue;}

                    //find the row and column index in the iamge for this point
                    // rowIdn = getRowIdx(cpt);
                    rowIdn = cloud_in->points[i].ring;

                    if (rowIdn < 0 || rowIdn >= VERT_SCAN)
                    continue;

                    columnIdn = getColIdx(cpt);

                    if (columnIdn < 0 || columnIdn >= HORZ_SCAN)
                    continue;

                    range = sqrt(cpt.x * cpt.x + cpt.y * cpt.y + cpt.z * cpt.z);

                    labelMat.at<int>(rowIdn, columnIdn) = -1;
                    rangeMat.at<float>(rowIdn, columnIdn) = range;

                    index = columnIdn + rowIdn * HORZ_SCAN;
                    index_v[index] = org_index;

                    fullCloud->points[index] = cpt;
                    org_index++;
                }
            }

            int getRowIdx(PointT pt)
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

            int getColIdx(PointT pt)
            {
                float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
                static float ang_res_x = 360.0 / float(HORZ_SCAN);
                int col_idx = -round((horizonAngle - 90.0) / ang_res_x) + HORZ_SCAN / 2;
                if (col_idx >= HORZ_SCAN)
                    col_idx -= HORZ_SCAN;
                return col_idx;
            }

            void labelComponents(int row, int col, std::vector<std::vector<int>> &clusterIndices)
            {
                // use std::queue std::vector std::deque will slow the program down greatly
                int fromIndX, fromIndY, thisIndX, thisIndY;
                float d1, d2;
                
                queueIndX[0] = row;
                queueIndY[0] = col;
                int queueSize = 1;
                int queueStartInd = 0;
                int queueEndInd = 1;

                allPushedIndX[0] = row;
                allPushedIndY[0] = col;
                int allPushedIndSize = 1;

                std::vector<int> clusterIndice;

                //standard BFS 
                while (queueSize > 0)
                {
                    // Pop point
                    fromIndX = queueIndX[queueStartInd];
                    fromIndY = queueIndY[queueStartInd];
                    --queueSize;
                    ++queueStartInd;
                    // Mark popped point, The initial value of labelCount is 1.
                    labelMat.at<int>(fromIndX, fromIndY) = labelCount;

                    // Loop through all the neighboring grids of popped grid, neighbor=[[-1,0];[0,1];[0,-1];[1,0]]
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

                        /*---------------------------condition----------------------------*/
                        d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                                        rangeMat.at<float>(thisIndX, thisIndY));
                        d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                                        rangeMat.at<float>(thisIndX, thisIndY));

                        PointT pt_from = fullCloud->points[fromIndY + fromIndX * HORZ_SCAN];
                        PointT pt_this = fullCloud->points[thisIndY + thisIndX * HORZ_SCAN];

                        bool same_cluster = false;

                        if (pt_from.label == 10 && pt_this.label == 10)
                        {
                            if (pt_from.id == pt_this.id)
                                same_cluster = true;
                        }

                        if (pt_from.label != 10 && pt_this.label != 10)
                        {
                            float dist = (pt_from.x - pt_this.x) * (pt_from.x - pt_this.x) 
                                        + (pt_from.y - pt_this.y) * (pt_from.y - pt_this.y) 
                                        + (pt_from.z - pt_this.z) * (pt_from.z - pt_this.z);

                            if (pt_from.label == pt_this.label && dist < 0.5 * 0.5)
                                same_cluster = true;
                        }

                        if (same_cluster)
                        {
                            queueIndX[queueEndInd] = thisIndX;
                            queueIndY[queueEndInd] = thisIndY;
                            ++queueSize;
                            ++queueEndInd;

                            labelMat.at<int>(thisIndX, thisIndY) = labelCount;

                            clusterIndice.push_back(index_v[thisIndY + thisIndX * HORZ_SCAN]);

                            allPushedIndX[allPushedIndSize] = thisIndX;
                            allPushedIndY[allPushedIndSize] = thisIndY;
                            ++allPushedIndSize;
                        }
                    }
                }

                // check if this segment is valid
                bool feasibleSegment = false;

                if (allPushedIndSize >= 5)
                    feasibleSegment = true;
                
                // segment is valid, mark these points
                if (feasibleSegment == true){
                    ++labelCount;
                    clusterIndices.push_back(clusterIndice);
                }
                else{
                    // segment is invalid, mark these points
                    for (size_t i = 0; i < allPushedIndSize; ++i) {
                        labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 0;
                    }
                }
            }
    };
}

#endif