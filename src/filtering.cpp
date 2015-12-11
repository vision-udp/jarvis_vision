//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/filtering.hpp>

#include <boost/make_shared.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm> // for std::max
#include <array>
#include <vector>

#include <cassert>
#include <cmath>   // for std::sqrt, std::cbrt
#include <cstddef> // for std::size_t

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;
using std::size_t;

// ==========================================
// Auxiliary
// ==========================================

template <typename PointT>
static std::array<float, 3>
optimize_leaf_sizes(const PointCloud<PointT> &filtered_cloud, const float width,
                    const float height, const size_t desired_num_of_points) {
  assert(!filtered_cloud.empty());
  assert(width > 1 && height > 1);

  // Let c and d be positive real numbers, then
  // we want to find c and d, such that:
  // c / d == width / height
  // c * d == desired_num_of_points
  const float N = desired_num_of_points;
  const float d = std::sqrt((N * height) / width);
  const float c = N / d;
  // c x d represents the grid size.

  // Now find x,y,z limits
  float x_min = filtered_cloud[0].x, x_max = filtered_cloud[0].x;
  float y_min = filtered_cloud[0].y, y_max = filtered_cloud[0].y;
  float z_min = filtered_cloud[0].z, z_max = filtered_cloud[0].z;

  auto update_field = [](float &cur_min, float &cur_max, float next_val) {
    if (next_val < cur_min)
      cur_min = next_val;
    else if (next_val > cur_max)
      cur_max = next_val;
  };

  for (const auto &p : filtered_cloud) {
    update_field(x_min, x_max, p.x);
    update_field(y_min, y_max, p.y);
    update_field(z_min, z_max, p.z);
  }

  const float xleaf = (x_max - x_min) / c;
  const float yleaf = (y_max - y_min) / d;
  const float zleaf = (z_max - z_min) / std::cbrtf(desired_num_of_points);
  return {{xleaf, yleaf, zleaf}};
}

// ==========================================
// Definitions
// ==========================================

template <typename PointT>
shared_ptr<PointCloud<PointT>> cloud_filter<PointT>::filter_input_cloud() {
  assert(cloud->isOrganized());

  const auto fcloud_a = make_shared<PointCloud<PointT>>();
  const auto fcloud_b = make_shared<PointCloud<PointT>>();

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *fcloud_a, indices);

  pcl::PassThrough<PointT> passthrough;
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(0.30f, 8.00f); // 30cm, 8m
  passthrough.setInputCloud(fcloud_a);
  passthrough.filter(*fcloud_b);

  if (fcloud_b->empty())
    return fcloud_b;

  pcl::VoxelGrid<PointT> vg;
  const auto leaf_sizes = optimize_leaf_sizes(
      *fcloud_b, cloud->width, cloud->height, this->desired_num_of_points);
  vg.setLeafSize(leaf_sizes[0], leaf_sizes[1], leaf_sizes[2]);
  vg.setInputCloud(fcloud_b);
  vg.filter(*fcloud_a);
  return fcloud_a;
}

// ==========================================
// Template instantations
// ==========================================

#include <pcl/point_types.h>

template class jarvis::cloud_filter<pcl::PointXYZ>;
template class jarvis::cloud_filter<pcl::PointXYZRGBA>;
