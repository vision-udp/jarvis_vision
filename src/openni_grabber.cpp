//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/openni_grabber.hpp>

#include <pcl/io/openni_grabber.h>

#include <boost/function.hpp>
#include <cassert>
#include <chrono>
#include <iostream> // std::clog, std::endl
#include <thread>   // this_thread::sleep_for

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using namespace std::chrono_literals;
using pcl::OpenNIGrabber;

// ==========================================
// openni_grabber
// ==========================================

template <typename PointT>
openni_grabber<PointT>::openni_grabber(const bool enable_dr) {
  grabber = std::make_unique<pcl::OpenNIGrabber>();
  config_callback();
  config_depth_registration(enable_dr);
  grabber->start();
}

template <typename PointT>
openni_grabber<PointT>::~openni_grabber() {
  assert(grabber->isRunning());
  grabber->stop();
  // The sleep is a work around, since sometimes callbacks are called after
  // stopping or destroying the grabber.
  std::this_thread::sleep_for(300ms);
}

template <typename PointT>
void openni_grabber<PointT>::grab(cloud_const_ptr &cloud) {
  assert(!output_cloud);
  assert(grabber->isRunning());
  std::unique_lock<std::mutex> lk(mtx);
  output_cloud = &cloud;
  cv.wait(lk, [&] { return output_cloud == nullptr; });
  assert(!output_cloud);
}

template <typename PointT>
void openni_grabber<PointT>::config_callback() {
  boost::function<void(const cloud_const_ptr &)> callback;
  callback = [this](const cloud_const_ptr &cloud) {
    std::unique_lock<std::mutex> lk(mtx);
    if (!output_cloud)
      return;
    *output_cloud = cloud;
    output_cloud = nullptr;
    lk.unlock();
    cv.notify_one();
  };
  grabber->registerCallback(callback);
}

template <typename PointT>
void openni_grabber<PointT>::config_depth_registration(bool enable_dr) {
  if (!enable_dr)
    return;
  auto device = grabber->getDevice();
  if (device->isDepthRegistrationSupported()) {
    device->setDepthRegistration(true);
    assert(device->isDepthRegistered());
    std::clog << "Depth registration enabled." << std::endl;
  } else {
    std::clog << "Error: Depth registration is not supported" << std::endl;
  }
}

// ==========================================
// Template instantations
// ==========================================

#include <pcl/point_types.h>

template class jarvis::openni_grabber<pcl::PointXYZ>;
template class jarvis::openni_grabber<pcl::PointXYZRGBA>;
