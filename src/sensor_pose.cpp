/// \file sensor_pose.cpp
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-29

#include <jarvis/sensor_pose.hpp>
#include <pcl/common/common_headers.h>

template <typename PointT>
Eigen::Affine3f jarvis::sensor_pose(const pcl::PointCloud<PointT> &cloud) {
  const Eigen::Vector4f &origin = cloud.sensor_origin_;
  const Eigen::Quaternionf &rotation = cloud.sensor_orientation_;
  const Eigen::Translation3f translation(origin[0], origin[1], origin[2]);
  return translation * rotation;
}

// ==========================================
// Template instantations
// ==========================================

#include <pcl/point_types.h>

template Eigen::Affine3f
jarvis::sensor_pose(const pcl::PointCloud<pcl::PointXYZ> &cloud);
template Eigen::Affine3f
jarvis::sensor_pose(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
