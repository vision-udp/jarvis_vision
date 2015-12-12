//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/simple_visualizer.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace jarvis {

template <typename PointT>
simple_visualizer<PointT>::simple_visualizer() {}

template <typename PointT>
void simple_visualizer<PointT>::start() {
  viewer = std::make_unique<viewer_t>("3D Viewer", false);
  viewer->setFullScreen(full_screen);
  viewer->setWindowBorders(true);
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);
  viewer->createInteractor();
}

template <typename PointT>
void simple_visualizer<PointT>::stop() {
  viewer->close();
}

template <typename PointT>
bool simple_visualizer<PointT>::was_stopped() const {
  return viewer->wasStopped();
}

template <typename PointT>
simple_visualizer<PointT>::~simple_visualizer() {}

template <typename PointT>
void simple_visualizer<PointT>::spin_once() {
  viewer->spinOnce();
}

template <typename PointT>
void simple_visualizer<PointT>::spin() {
  viewer->spin();
}

template <typename PointT>
void simple_visualizer<PointT>::show_cloud(const cloud_ptr &cloud,
                                           const std::string &id) {
  if (!viewer->updatePointCloud(cloud, id))
    viewer->addPointCloud(cloud, id);
}

template <typename PointT>
void simple_visualizer<PointT>::remove_all_point_clouds() {
  viewer->removeAllPointClouds();
}

template <typename PointT>
void simple_visualizer<PointT>::add_text_3d(const std::string &text,
                                            const PointT pos,
                                            const std::string &id) {
  viewer->addText3D(text, pos, 0.02, 1.0, 1.0, 1.0, id);
}

template <typename PointT>
void simple_visualizer<PointT>::remove_text_3d(const std::string &id) {
  viewer->removeText3D(id);
}

// =====================================
// Template instantiations
// =====================================

#include <pcl/point_types.h>

template class simple_visualizer<pcl::PointXYZ>;
template class simple_visualizer<pcl::PointXYZRGBA>;

} // end namespace jarvis
