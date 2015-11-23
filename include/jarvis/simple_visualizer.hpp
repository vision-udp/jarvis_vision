//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_SIMPLE_VISUALIZER_HPP
#define JARVIS_SIMPLE_VISUALIZER_HPP

#include <string>               // For std::string
#include <memory>               // For std::unique_ptr
#include <boost/shared_ptr.hpp> // For boost::shared_ptr

namespace pcl {
template <typename T>
class PointCloud;
namespace visualization {
class PCLVisualizer;
}
}

namespace jarvis {

template <typename PointT>
class simple_visualizer {
  using point_t = PointT;
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_ptr = boost::shared_ptr<cloud_t>;
  using viewer_t = pcl::visualization::PCLVisualizer;

public:
  simple_visualizer();
  ~simple_visualizer();
  void loop();
  void show_cloud(const cloud_ptr &cloud, const std::string &id = "cloud");

private:
  std::unique_ptr<viewer_t> viewer;
};

} // End namespace drd

#endif // Header guard
