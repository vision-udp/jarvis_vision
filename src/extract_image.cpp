/// \file extract_image.cpp
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-29

#include <jarvis/extract_image.hpp>

#include <pcl/common/common_headers.h>

cv::Mat3b
jarvis::extract_image(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud) {
  auto height = static_cast<int>(cloud.height);
  auto width = static_cast<int>(cloud.width);
  auto result = cv::Mat(height, width, CV_8UC3);

  if (!cloud.isOrganized() || cloud.empty())
    return result;

  auto *image_ptr = result.ptr();
  for (const auto &point : cloud) {
    const auto &rgb = point.getRGBVector3i();
    (*image_ptr++) = static_cast<unsigned char>(rgb[2]);
    (*image_ptr++) = static_cast<unsigned char>(rgb[1]);
    (*image_ptr++) = static_cast<unsigned char>(rgb[0]);
  }

  return result;
}
