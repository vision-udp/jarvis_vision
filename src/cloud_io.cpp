//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_io.hpp>

#include <boost/make_shared.hpp> // for make_shared
#include <pcl/point_types.h>     // for PointXYZ, PointXYZRGBA
#include <pcl/io/pcd_io.h>       // for loadPCDFile
#include <stdexcept>             // for runtime_error

// ==========================================
// Global namespace visibility
// ==========================================

using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;

// ==========================================
// Function definitions
// ==========================================

template <typename PointT>
shared_ptr<PointCloud<PointT>> jarvis::load_cloud(const std::string &filename) {
  auto cloud = make_shared<PointCloud<PointT>>();

  if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    throw std::runtime_error("Cloud reading failed.");

  return cloud;
}

// ==========================================
// Template instantations
// ==========================================

template shared_ptr<PointCloud<pcl::PointXYZ>>
jarvis::load_cloud<pcl::PointXYZ>(const std::string &);

template shared_ptr<PointCloud<pcl::PointXYZRGBA>>
jarvis::load_cloud<pcl::PointXYZRGBA>(const std::string &);
