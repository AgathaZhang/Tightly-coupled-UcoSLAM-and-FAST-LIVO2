#pragma once
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

Eigen::Matrix4f kiss_matcher_pointcloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& src_pcl,pcl::PointCloud<pcl::PointXYZINormal>::Ptr& tgt_pcl);