//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_CLOUD_PIPELINE_HPP
#define JARVIS_CLOUD_PIPELINE_HPP

#include <jarvis/pcl_fwd.hpp>
#include <boost/shared_ptr.hpp>

namespace jarvis {

class cloud_pipeline {
public:
  using cloud_t = pcl::PointCloud<pcl::PointXYZ>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  void process(const cloud_const_ptr &input_cloud);
  auto get_colored_cloud() const { return colored_cloud; }

private:
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> colored_cloud;
};

} // end namespace jarvis

#endif // Header guard
