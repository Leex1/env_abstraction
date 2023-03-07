/**
 * @file cluster.hpp
 * @author Wang yifei
 * @brief
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CLUSTER_H
#define CLUSTER_H

#include <chrono>
#include <ctime>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <vector>

// entities
#include "cluster/box.h"

template <typename PointT>
class PointCloudCluster
{
public:
    // constructor
    PointCloudCluster(){};
    PointCloudCluster(ros::NodeHandle *nh) : node_handle_(*nh)
    {
        ROS_INFO("Inititalizing point cloud cluster function...");
    }

    void point_cloud_filter(pcl::PointCloud<PointT> cloud,
                            pcl::PointCloud<PointT> &cloud_roi,
                            float filter_res, Eigen::Vector4f min_point, Eigen::Vector4f max_point);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> point_cloud_cluster(
        pcl::PointCloud<PointT> cloud, float cluster_tolerance, int min_size, int max_size);

    Box bounding_box(typename pcl::PointCloud<PointT>::Ptr cluster);

    void point_cloud_voxelize(pcl::PointCloud<PointT> cloud,
                              pcl::PointCloud<PointT> &cloud_filtered,
                              float filter_res);

private:
    ros::NodeHandle node_handle_;

    // deconstructor
    ~PointCloudCluster();
};

template <typename PointT>
inline void PointCloudCluster<PointT>::point_cloud_filter(
    pcl::PointCloud<PointT> cloud,
    pcl::PointCloud<PointT> &cloud_roi,
    float filter_res, Eigen::Vector4f min_point, Eigen::Vector4f max_point)
{
    auto start_time = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(filter_res, filter_res, filter_res);
    vg.filter(*cloud_filtered);

    pcl::CropBox<PointT> roi(true);
    roi.setMin(min_point);
    roi.setMax(max_point);
    roi.setInputCloud(cloud_filtered);
    roi.filter(cloud_roi);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_roi.makeShared());
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_roi.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(cloud_roi);

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PointCloudCluster<PointT>::point_cloud_cluster(
    pcl::PointCloud<PointT> cloud,
    float cluster_tolerance, int min_size, int max_size)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance); // 2cm
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud.makeShared());
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit)
        {
            cluster->points.push_back(cloud.makeShared()->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }
    return clusters;
}

template <typename PointT>
Box PointCloudCluster<PointT>::bounding_box(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
inline void PointCloudCluster<PointT>::point_cloud_voxelize(
    pcl::PointCloud<PointT> cloud,
    pcl::PointCloud<PointT> &cloud_filtered,
    float filter_res)
{
    auto start_time = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(filter_res, filter_res, filter_res);
    vg.filter(cloud_filtered);

    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;
}
#endif