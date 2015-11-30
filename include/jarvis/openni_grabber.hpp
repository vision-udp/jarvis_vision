//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_OPENNI_GRABBER_HPP
#define JARVIS_OPENNI_GRABBER_HPP

#include <boost/shared_ptr.hpp>
#include <condition_variable>
#include <memory> // for unique_ptr, make_unique
#include <mutex>

namespace pcl {
template <typename PointT>
class PointCloud;

class OpenNIGrabber;
} // end namespace pcl

namespace jarvis {
template <typename PointT>
class openni_grabber {
public:
  using point_t = PointT;
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  openni_grabber(const bool enable_dr);
  ~openni_grabber();
  void grab(cloud_const_ptr &);

private:
  void config_callback();
  void config_depth_registration(bool);

private:
  std::mutex mtx;
  std::condition_variable cv;
  cloud_const_ptr *output_cloud{nullptr};
  std::unique_ptr<pcl::OpenNIGrabber> grabber;
};

} // end namespace jarvis

#endif // Header guard
