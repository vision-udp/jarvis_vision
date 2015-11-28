//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_CLASSIFICATION_HPP
#define JARVIS_CLASSIFICATION_HPP

#include <jarvis/pcl_fwd.hpp>

#include <boost/shared_ptr.hpp>

namespace jarvis {

enum class object_type { unknown, cylinder, sphere, cube };

struct object_info {
  object_type type;
  double probability;
  double radius; // Only meaningful for objects with radius
};

template <typename PointT>
object_info
classify_object(const boost::shared_ptr<const pcl::PointCloud<PointT>> &cloud);

} // end namespace jarvis

#endif // Header guard
