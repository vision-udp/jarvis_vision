//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_FILTERING_HPP
#define JARVIS_FILTERING_HPP

#include <boost/shared_ptr.hpp>
#include <jarvis/pcl_fwd.hpp>

namespace jarvis {

template <typename PointT>
class cloud_filter {
public:
  using cloud_t = pcl::PointCloud<PointT>;
  using cloud_ptr = boost::shared_ptr<cloud_t>;
  using cloud_const_ptr = boost::shared_ptr<const cloud_t>;

public:
  void set_input_cloud(const cloud_const_ptr &input_cloud) {
    cloud = input_cloud;
  }

  /// \brief Sets the leaf size of x, y and z
  void set_leaf_size(float value) { leaf_size = value; }

  cloud_ptr filter_input_cloud();

private:
  cloud_const_ptr cloud;
  float leaf_size{};
};

} // end namespace jarvis

#endif // Header guard
