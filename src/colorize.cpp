//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/colorize.hpp>

#include <boost/make_shared.hpp> // for make_shared
#include <pcl/point_cloud.h>     // for point_cloud
#include <pcl/point_types.h>     // for PointXYZ, PointXYZRGBA
#include <pcl/PointIndices.h>    // for PointIndices

using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGBA;
using pcl::PointIndices;

shared_ptr<PointCloud<PointXYZRGBA>>
jarvis::make_colored_cloud(const PointCloud<PointXYZ> &cloud,
                           const rgba_color color) {

  using colored_cloud_t = PointCloud<PointXYZRGBA>;

  const size_t N = cloud.size();
  auto colored_cloud = make_shared<colored_cloud_t>();
  colored_cloud->points.resize(N);
  colored_cloud->width = cloud.width;
  colored_cloud->height = cloud.height;
  colored_cloud->is_dense = cloud.is_dense;

  for (size_t i = 0; i < N; ++i) {
    const auto &p = cloud.points[i];
    auto &cp = colored_cloud->points[i];
    cp.x = p.x, cp.y = p.y, cp.z = p.z;
    cp.r = color.r, cp.g = color.g, cp.b = color.b, cp.a = color.a;
  }

  return colored_cloud;
}

void jarvis::colorize(PointCloud<PointXYZRGBA> &cloud,
                      const PointIndices &indices, const rgba_color color) {
  for (int idx : indices.indices) {
    auto &p = cloud[static_cast<size_t>(idx)];
    p.r = color.r, p.g = color.g, p.b = color.b, p.a = color.a;
  }
}
