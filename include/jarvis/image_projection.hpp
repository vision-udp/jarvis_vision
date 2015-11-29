/// \file range_image.hpp
/// \brief This file contains utilities to extract an image from a point cloud.
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#ifndef JARVIS_IMAGE_PROJECTION
#define JARVIS_IMAGE_PROJECTION

#include <pcl/common/common_headers.h>

#include <opencv/cv.hpp>

namespace jarvis {
inline cv::Mat3b
extract_image(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud) {
  auto height = static_cast<int>(cloud.height);
  auto width = static_cast<int>(cloud.width);
  auto result = cv::Mat(height, width, CV_8UC3);
  if (!cloud.isOrganized() || cloud.empty())
    return result;

  auto *image_ptr = result.ptr();
  for (const auto &point : cloud) {
    const auto &rgb = point.getRGBVector3i();
    (*image_ptr++) = static_cast<unsigned char>(rgb[0]);
    (*image_ptr++) = static_cast<unsigned char>(rgb[1]);
    (*image_ptr++) = static_cast<unsigned char>(rgb[2]);
  }

  return result;
}
}

#endif
