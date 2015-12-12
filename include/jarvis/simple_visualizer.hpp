//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_SIMPLE_VISUALIZER_HPP
#define JARVIS_SIMPLE_VISUALIZER_HPP

#include <boost/shared_ptr.hpp> // For boost::shared_ptr
#include <memory>               // For std::unique_ptr
#include <string>               // For std::string

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
  using cloud_ptr = boost::shared_ptr<const cloud_t>;
  using viewer_t = pcl::visualization::PCLVisualizer;

public:
  simple_visualizer();
  ~simple_visualizer();
  void set_full_screen(bool value) { full_screen = value; }
  void start();
  void stop();
  bool was_stopped() const;
  void spin_once();
  void spin();
  void show_cloud(const cloud_ptr &cloud, const std::string &id = "cloud");

  void add_text_3d(const std::string &text, const PointT pos,
                   const std::string &id);
  void remove_text_3d(const std::string &id);

private:
  bool full_screen{};
  std::unique_ptr<viewer_t> viewer;
};

} // End namespace drd

#endif // Header guard
