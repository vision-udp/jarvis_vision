//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/simple_visualizer.hpp>

#include <chrono> // For std::chrono_literals
#include <thread> // For std::this_thread::sleep_for
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace jarvis {

template <typename PointT>
simple_visualizer<PointT>::simple_visualizer() {
  viewer = std::make_unique<viewer_t>("3D Viewer", false);
  viewer->setFullScreen(true);
  viewer->setWindowBorders(true);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);
  viewer->createInteractor();
}

template <typename PointT>
simple_visualizer<PointT>::~simple_visualizer() {}

template <typename PointT>
void simple_visualizer<PointT>::loop() {
  using namespace std::chrono_literals;
  while (!viewer->wasStopped()) {
    viewer->spinOnce();
    std::this_thread::sleep_for(100ms);
  }
}

template <typename PointT>
void simple_visualizer<PointT>::show_cloud(const cloud_ptr &cloud,
                                           const std::string &id) {
  // using handler_t =
  //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>;

  // handler_t handler(cloud, r, g, b);
  //    if (!viewer.updatePointCloud(cloud, handler, id))
  //      viewer.addPointCloud(cloud, handler, id);

  if (!viewer->updatePointCloud(cloud, id))
    viewer->addPointCloud(cloud, id);
}

// =====================================
// Template instantiations
// =====================================

template class simple_visualizer<pcl::PointXYZ>;
template class simple_visualizer<pcl::PointXYZRGB>;
template class simple_visualizer<pcl::PointXYZRGBA>;

} // end namespace jarvis
