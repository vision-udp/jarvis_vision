//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/filtering.hpp>

#include <boost/make_shared.hpp>    // for make_shared
#include <pcl/point_types.h>        // for PointXYZ, PointXYZRGBA
#include <pcl/filters/filter.h>     // for removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h> // for VoxelGrid
#include <vector>                   // for std::vector

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;

// ==========================================
// Definitions
// ==========================================

template <typename PointT>
shared_ptr<PointCloud<PointT>> cloud_filter<PointT>::filter_input_cloud() {
  // Remove nans
  std::vector<int> indices;
  const auto filtered_cloud = make_shared<PointCloud<PointT>>();
  pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);

  const auto filtered_cloud2 = make_shared<PointCloud<PointT>>();
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(filtered_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*filtered_cloud2);
  return filtered_cloud2;
}

// ==========================================
// Template instantations
// ==========================================

template class jarvis::cloud_filter<pcl::PointXYZ>;
template class jarvis::cloud_filter<pcl::PointXYZRGBA>;
