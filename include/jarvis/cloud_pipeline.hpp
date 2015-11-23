//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_CLOUD_PIPELINE_HPP
#define JARVIS_CLOUD_PIPELINE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <pcl/PointIndices.h>

namespace pcl {
template <typename PointT>
class PointCloud;

struct PointXYZ;
struct PointXYZRGBA;
struct Normal;

namespace search {
template <typename PointT>
class Search;
}
} // end namespace pcl

namespace jarvis {

class cloud_pipeline {
  using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
  using cloud_ptr = boost::shared_ptr<const cloud_t>;
  using normals_t = pcl::PointCloud<pcl::Normal>;
  using search_t = pcl::search::Search<pcl::PointXYZ>;

public:
  cloud_pipeline();
  ~cloud_pipeline();

  void process(const cloud_ptr &input_cloud);

  auto get_colored_cloud() const { return colored_cloud; }

private:
  void set_search_method();
  void estimate_normals();
  void segment();

private:
  cloud_ptr cloud;
  boost::shared_ptr<search_t> search;
  boost::shared_ptr<normals_t> normals;
  std::vector<pcl::PointIndices> clusters;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> colored_cloud;
};

} // end namespace jarvis

#endif // Header guard
