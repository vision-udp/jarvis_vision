/// \file sensor_pose.hpp
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#ifndef JARVIS_SENSOR_POSE
#define JARVIS_SENSOR_POSE

#include <pcl/common/common_headers.h>

namespace jarvis {
template <typename PointT>
Eigen::Affine3f sensor_pose(const pcl::PointCloud<PointT> &cloud) {
  const Eigen::Vector4f &origin = cloud.sensor_origin_;
  const Eigen::Quaternionf &rotation = cloud.sensor_orientation_;
  const Eigen::Translation3f translation(origin[0], origin[1], origin[2]);
  return translation * rotation;
}
}

#endif
