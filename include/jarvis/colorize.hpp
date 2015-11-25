//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef JARVIS_COLORIZE_HPP
#define JARVIS_COLORIZE_HPP

#include <boost/shared_ptr.hpp> // for shared_ptr
#include <random>               // for random_device, default_random_engine
#include <cstdint>              // for uint8_t

namespace pcl {
template <typename PointT>
class PointCloud;
struct PointXYZ;
struct PointXYZRGBA;
struct PointIndices;
} // end namespace pcl

namespace jarvis {

/// \brief POD-Structure to represent RGBA colors.
///
struct rgba_color {
  rgba_color() = default;
  rgba_color(std::uint8_t r_, std::uint8_t g_, std::uint8_t b_,
             std::uint8_t a_ = 0)
      : r{r_}, g{g_}, b{b_}, a{a_} {}

  std::uint8_t r, g, b, a;
};

/// \brief Copies the points of an uncolored cloud and assign the given color to
/// each copied point.
///
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>
make_colored_cloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                   rgba_color color);

/// \brief Sets the color of a subset of points to the given one.
///
void colorize(pcl::PointCloud<pcl::PointXYZRGBA> &cloud,
              const pcl::PointIndices &indices, rgba_color color);

/// \brief Class to generate random colors.
///
class color_generator {
public:
  rgba_color any() { return rgba_color(gen_value(), gen_value(), gen_value()); }

  rgba_color red() { return rgba_color(gen_value(), 0, 0); }
  rgba_color green() { return rgba_color(0, gen_value(), 0); }
  rgba_color blue() { return rgba_color(0, 0, gen_value()); }

  rgba_color cyan() { return rgba_color(0, gen_value(), gen_value()); }
  rgba_color magenta() { return rgba_color(gen_value(), 0, gen_value()); }
  rgba_color yellow() { return rgba_color(gen_value(), gen_value(), 0); }

private:
  std::uint8_t gen_value() { return static_cast<std::uint8_t>(dist(engine)); }

private:
  std::default_random_engine engine{std::random_device()()};
  std::uniform_int_distribution<> dist{30, 255};
};

} // end namespace jarvis

#endif // Header guard
