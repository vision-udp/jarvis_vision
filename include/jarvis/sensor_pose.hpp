/// \file sensor_pose.hpp
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#ifndef JARVIS_SENSOR_POSE_HPP
#define JARVIS_SENSOR_POSE_HPP

#include <pcl/common/common_headers.h>

namespace jarvis {
template <typename PointT>
Eigen::Affine3f sensor_pose(const pcl::PointCloud<PointT> &cloud);
}

#endif
