//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_COLORIZE_HPP
#define JARVIS_COLORIZE_HPP

#include <jarvis/pcl_fwd.hpp>
#include <boost/shared_ptr.hpp> // for shared_ptr
#include <cstdint>              // for uint8_t

namespace jarvis {

/// \brief POD-Structure to represent RGBA colors.
///
struct rgba_color {
  rgba_color() = default;
  rgba_color(std::uint8_t r_, std::uint8_t g_, std::uint8_t b_,
             std::uint8_t a_ = 255)
      : r{r_}, g{g_}, b{b_}, a{a_} {}

  std::uint8_t r, g, b, a;
};

/// \brief Copies the points of an uncolored cloud and assign the given color to
/// each copied point.
///
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>
make_colored_cloud(const pcl::PointCloud<PointT> &cloud, rgba_color color);

/// \brief Sets the color of a subset of points to the given one.
///
void colorize(pcl::PointCloud<pcl::PointXYZRGBA> &cloud,
              const pcl::PointIndices &indices, rgba_color color);

} // end namespace jarvis

#endif // Header guard
