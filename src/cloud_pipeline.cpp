//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <jarvis/colorize.hpp>
#include <jarvis/classification.hpp>
#include <jarvis/plane_extraction.hpp>
#include <jarvis/steady_timer.hpp>

#include <iostream> // for clog
#include <cassert>  // for assert
#include <cstdint>  // for uint8_t

#include <boost/make_shared.hpp>               // for make_shared
#include <pcl/point_types.h>                   // for PointXYZ, Normal
#include <pcl/filters/filter.h>                // for removeNaNFromPointCloud
#include <pcl/search/kdtree.h>                 // for KdTree
#include <pcl/segmentation/extract_clusters.h> // EuclideanClusterExtraction

// ==========================================
// Using directives
// ==========================================

using namespace jarvis;

using boost::make_shared;
using boost::shared_ptr;
using pcl::ModelCoefficients;
using pcl::PointCloud;
using pcl::PointIndices;
using std::size_t;
using std::uint8_t;
using std::clog;
using std::endl;

template <typename PointT>
using cloud_ptr = boost::shared_ptr<PointCloud<PointT>>;

template <typename PointT>
using cloud_const_ptr = boost::shared_ptr<const PointCloud<PointT>>;

// ==========================================
// Auxiliary functions
// ==========================================

template <typename PointT>
static cloud_ptr<PointT> remove_nans(const PointCloud<PointT> &input_cloud) {
  std::vector<int> indices;
  const auto cloud = make_shared<PointCloud<PointT>>();
  pcl::removeNaNFromPointCloud(input_cloud, *cloud, indices);
  return cloud;
}

template <typename PointT>
static auto make_kdtree() {
  using kdtree_t = pcl::search::KdTree<PointT>;
  return make_shared<kdtree_t>(false);
}

// ==========================================
// Auxiliary class
// ==========================================

namespace {
template <typename PointT>
class pipeline_manager {
  using cloud_t = pcl::PointCloud<PointT>;

public:
  pipeline_manager(const cloud_t &input_cloud) {
    clog << "Total points: " << input_cloud.size() << endl;
    timer.run("Removing NaNs");
    cloud = remove_nans(input_cloud);
    timer.finish();
    clog << "Total NonNans points: " << cloud->size() << endl;
  }

  void run() {
    extract_planes();
    clusterize();
    classify_clusters();
  }

  cloud_ptr<pcl::PointXYZRGBA> get_colored_cloud() const;

private:
  void extract_planes() {
    timer.run("Extracting planes");
    plane_extractor.set_input_cloud(cloud);
    plane_extractor.set_min_points(25000);
    plane_extractor.extract_planes();
    timer.finish();
    const size_t num_planes = plane_extractor.get_num_planes();
    std::clog << "Number of found planes: " << num_planes << '\n';
  }

  void clusterize() {
    const auto remaining = plane_extractor.get_remaining_indices();
    clog << "Number of remaining points: " << remaining->indices.size() << '\n';
    timer.run("Clustering");
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(300);
    ec.setMaxClusterSize(15000);
    ec.setSearchMethod(make_kdtree<PointT>());
    ec.setInputCloud(cloud);
    ec.setIndices(remaining);
    ec.extract(clusters);
    timer.finish();
    clog << "Number of found clusters: " << clusters.size() << endl;
  }

  void classify_clusters() {
    timer.run("Processing clusters");
    clusters_info.resize(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
      const std::vector<int> &indices = clusters[i].indices;
      const auto cluster_cloud = boost::make_shared<cloud_t>(*cloud, indices);
      clusters_info[i] = classify_object<PointT>(cluster_cloud);
    }
    timer.finish();
  }

private:
  mutable steady_timer timer;
  boost::shared_ptr<pcl::PointCloud<PointT>> cloud;
  plane_extractor<PointT> plane_extractor;
  std::vector<PointIndices> clusters;
  std::vector<object_info> clusters_info;
};
} // end anonymous namespace

template <typename PointT>
cloud_ptr<pcl::PointXYZRGBA>
pipeline_manager<PointT>::get_colored_cloud() const {
  timer.run("Creating colored cloud");
  const rgba_color red{255, 0, 0};
  const rgba_color green{0, 255, 0};
  const rgba_color cyan{0, 255, 255};
  const rgba_color magenta{255, 0, 255};

  const auto colored_cloud = make_colored_cloud(*cloud, red);

  const size_t num_planes = plane_extractor.get_num_planes();
  for (size_t i = 0; i < num_planes; ++i) {
    const auto &inliers = plane_extractor.get_inliers(i);
    const auto blue_scale =
        static_cast<double>(inliers.indices.size()) / cloud->size();
    const uint8_t blue_val = static_cast<uint8_t>(55 + 200 * blue_scale);
    colorize(*colored_cloud, inliers, rgba_color(0, 0, blue_val));
  }

  for (size_t i = 0; i < clusters.size(); ++i) {
    clog << "Cluster " << i + 1 << ": ";
    clog << "Size=" << clusters[i].indices.size();
    const object_info info = clusters_info[i];
    rgba_color object_color{};

    switch (info.type) {
    case object_type::cylinder:
      object_color = green;
      clog << ", type=cylinder, radius=" << info.radius * 100 << "cm";
      break;

    case object_type::sphere:
      object_color = cyan;
      clog << ", type=sphere, radius=" << info.radius * 100 << "cm";
      break;

    case object_type::cube:
      object_color = magenta;
      clog << ", type=cube";
      break;

    case object_type::unknown:
      object_color = rgba_color(128, 128, 0);
      clog << ", type=unknown";
      break;
    }

    if (info.type != object_type::unknown)
      clog << ", probability=" << 100.0 * info.probability << "%";

    clog << std::endl;
    colorize(*colored_cloud, clusters[i], object_color);
  }
  clog << endl;

  timer.finish();
  return colored_cloud;
}

// ==========================================
// cloud_pipeline definitions
// ==========================================

void cloud_pipeline::process(const cloud_const_ptr &input_cloud) {
  pipeline_manager<pcl::PointXYZ> manager(*input_cloud);
  manager.run();
  colored_cloud = manager.get_colored_cloud();
}
