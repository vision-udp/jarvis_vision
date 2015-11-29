/// \file extract_image.hpp
/// \brief This file contains utilities to extract an image from a point cloud.
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#ifndef JARVIS_EXTRACT_IMAGE_HPP
#define JARVIS_EXTRACT_IMAGE_HPP

#include <jarvis/pcl_fwd.hpp>

#include <opencv/cv.hpp>

namespace jarvis {
cv::Mat3b extract_image(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud);
}

#endif
