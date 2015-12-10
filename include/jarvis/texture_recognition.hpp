#ifndef JARVIS_TEXTURE_RECOGNITION_HPP
#define JARVIS_TEXTURE_RECOGNITION_HPP

#include <iostream>

#include <jarvis/pcl_fwd.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv/cv.hpp>

#include <algorithm>

namespace jarvis {
template <typename PointT>
class texture_recognition {

public:
  using point_t = pcl::PointXYZRGBA;
  using cloud_t = pcl::PointCloud<point_t>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  void test(const cloud_const_ptr &) {}
};
}
#endif
