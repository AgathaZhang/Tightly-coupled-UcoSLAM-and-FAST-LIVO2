
#pragma once
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


#include <chrono>
#include <iostream>
#include <random>
#include <Eigen/Core>

Eigen::Matrix4d transform_teaser_method(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_transform_teaser, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_transform_teaser,float scale_value = 10);
Eigen::Matrix4d map_icp_teaser(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_merge_cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_merge_cloud,float scale_value = 10);