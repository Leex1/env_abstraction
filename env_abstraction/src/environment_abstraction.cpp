/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023,
 *  Harbin Institude of Techology, NROS-Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Wang Yifei
 *********************************************************************/

#include <iostream>
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>

#include "patchworkpp/patchworkpp.hpp"
#include "cluster/cluster.hpp"

using PointType = pcl::PointXYZI;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;
boost::shared_ptr<PointCloudCluster<PointType>> PointCloudProcessor;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;
ros::Publisher pub_clusters;
ros::Publisher pub_obstacles;

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_filtered;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pcl::PointCloud<PointType> pc_voxel;
    geometry_msgs::PoseArray obstacles;
    geometry_msgs::Pose obstacle_point;
    obstacles.header.frame_id = "map";
    obstacles.header.stamp = ros::Time();

    pcl::fromROSMsg(*cloud_msg, pc_curr);

    PointCloudProcessor->point_cloud_filter(pc_curr, pc_filtered, 0.118, Eigen::Vector4f(-10, -5, -2.0, 1), Eigen::Vector4f(50, 7, 10, 1));
    PatchworkppGroundSeg->estimate_ground(pc_filtered, pc_ground, pc_non_ground, time_taken);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters =
        PointCloudProcessor->point_cloud_cluster(pc_non_ground, 0.85, 20, 7000);

    PointCloudProcessor->point_cloud_voxelize(pc_non_ground, pc_voxel, 1);

    int cluster_id = 1;
    for (auto &cluster : cloud_clusters)
    {
        Box box = PointCloudProcessor->bounding_box(cluster);
        obstacle_point.position.x = box.x_min;
        obstacle_point.position.y = box.y_min;
        obstacle_point.position.z = box.z_min;
        obstacle_point.orientation.x = box.x_max;
        obstacle_point.orientation.y = box.y_max;
        obstacle_point.orientation.z = box.z_max;
        obstacle_point.orientation.w = 0;
        obstacles.poses.push_back(obstacle_point);
        ++cluster_id;
    }

    std::cout << "Result: Input PointCloud: " << pc_curr.size()
              << " -> Ground: " << pc_ground.size()
              << "/ NonGround: " << pc_non_ground.size()
              << " voxel: " << pc_voxel.size()
              << " cluster size: " << cloud_clusters.size()
              << " (running_time: " << time_taken << " sec)" << std::endl;

    // for (size_t i = 0; i < pc_voxel.size(); ++i)
    // {
    //     obstacle_point.position.x = pc_voxel.points[i].x;
    //     obstacle_point.position.y = pc_voxel.points[i].y;
    //     obstacle_point.position.z = pc_voxel.points[i].z;
    //     obstacle_point.orientation.x = 0;
    //     obstacle_point.orientation.y = 0;
    //     obstacle_point.orientation.z = 0;
    //     obstacle_point.orientation.w = 0;
    //     obstacles.poses.push_back(obstacle_point);
    // }
    pub_obstacles.publish(obstacles);
    pub_cloud.publish(cloud2msg(pc_curr));
    pub_ground.publish(cloud2msg(pc_ground));
    pub_non_ground.publish(cloud2msg(pc_non_ground));
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<string>("/cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&nh));

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/demo/cloud", 100, true);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/ground", 100, true);
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/nonground", 100, true);
    pub_obstacles = nh.advertise<geometry_msgs::PoseArray>("/obstacles", 100, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
