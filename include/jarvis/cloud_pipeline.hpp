//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_CLOUD_PIPELINE_HPP
#define JARVIS_CLOUD_PIPELINE_HPP

#include <boost/shared_ptr.hpp>
#include <jarvis/pcl_fwd.hpp>
#include <jarvis/simple_visualizer.hpp>
#include <pcl/PointIndices.h>

namespace jarvis {

template <typename PointT>
class cloud_pipeline {
public:
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  void process(const cloud_const_ptr &input_cloud);

  cloud_const_ptr get_filtered_cloud() const { return filtered_cloud; }
  const std::vector<pcl::PointIndices> &get_clusters() { return clusters; }

private:
  cloud_const_ptr filtered_cloud;
  std::vector<pcl::PointIndices> clusters;
};

template <typename PointT>
class pipeline_searcher {
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  pipeline_searcher(bool full_screen) { vis.set_full_screen(full_screen); }
  void start() { vis.start(); }
  void stop() { vis.stop(); }
  void spin_once() { vis.spin_once(); }
  void spin() { vis.spin(); }

  void update_cloud(cloud_const_ptr cloud);

private:
  simple_visualizer<PointT> vis;
  size_t last_num_of_clusters{};
};

} // end namespace jarvis

#endif // Header guard
