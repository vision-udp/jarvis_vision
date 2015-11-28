//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <jarvis/colorize.hpp>
#include <jarvis/classification.hpp>
#include <jarvis/filtering.hpp>
#include <jarvis/plane_extraction.hpp>
#include <jarvis/steady_timer.hpp>

#include <boost/make_shared.hpp>
#include <iostream> // for clog
#include <vector>
#include <cstdint> // for uint8_t

#include <pcl/point_types.h> // for PointXYZ, PointRGBA
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;

using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGBA;
using std::size_t;
using std::uint8_t;
using std::clog;
using std::endl;

// ==========================================
// cloud_pipeline_impl
// ==========================================

namespace {
template <typename PointT>
class cloud_pipeline_impl {
  using cloud_t = PointCloud<PointT>;

public:
  cloud_pipeline_impl(const shared_ptr<const cloud_t> &input_cloud) {
    original_cloud = input_cloud;
  }

  void run() {
    timer.run("Applying filters");
    apply_filters();
    timer.finish();
    clog << "Original cloud size: " << original_cloud->size() << '\n';
    clog << "Filtered cloud size: " << cloud->size() << '\n';

    timer.run("Extracting planes");
    extract_planes();
    timer.finish();
    clog << "Number of found planes: ";
    clog << plane_extractor.get_num_planes() << '\n';
    clog << "Number of remaining points: ";
    clog << plane_extractor.get_remaining_indices()->indices.size() << '\n';

    timer.run("Clustering");
    clusterize();
    timer.finish();
    clog << "Number of found clusters: " << clusters.size() << endl;

    timer.run("Processing clusters");
    classify_clusters();
    timer.finish();
  }

  shared_ptr<PointCloud<PointXYZRGBA>> make_colored_cloud() const;

private:
  void apply_filters() {
    cloud_filter<PointT> filter;
    filter.set_input_cloud(original_cloud);
    filter.set_leaf_size(0.005f);
    cloud = filter.filter_input_cloud();
  }

  void extract_planes() {
    plane_extractor.set_input_cloud(cloud);
    plane_extractor.set_min_points(10000);
    plane_extractor.extract_planes();
  }

  void clusterize() {
    using kdtree_t = pcl::search::KdTree<PointT>;
    const auto search = make_shared<kdtree_t>(false);

    const auto remaining = plane_extractor.get_remaining_indices();
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(search);
    ec.setInputCloud(cloud);
    ec.setIndices(remaining);
    ec.extract(clusters);
  }

  void classify_clusters() {
    clusters_info.resize(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
      const std::vector<int> &indices = clusters[i].indices;
      const auto cluster_cloud = boost::make_shared<cloud_t>(*cloud, indices);
      clusters_info[i] = classify_object<PointT>(cluster_cloud);
    }
  }

private:
  mutable steady_timer timer;
  shared_ptr<const cloud_t> original_cloud;
  shared_ptr<cloud_t> cloud;
  plane_extractor<PointT> plane_extractor;
  std::vector<PointIndices> clusters;
  std::vector<object_info> clusters_info;
};
} // end anonymous namespace

template <typename PointT>
shared_ptr<PointCloud<PointXYZRGBA>>
cloud_pipeline_impl<PointT>::make_colored_cloud() const {
  timer.run("Creating colored cloud");
  const rgba_color red{255, 0, 0};
  const rgba_color green{0, 255, 0};
  const rgba_color cyan{0, 255, 255};
  const rgba_color magenta{255, 0, 255};

  const auto colored_cloud = jarvis::make_colored_cloud(*cloud, red);

  const size_t num_planes = plane_extractor.get_num_planes();
  for (size_t i = 0; i < num_planes; ++i) {
    const auto &inliers = plane_extractor.get_inliers(i);
    const auto blue_scale =
        static_cast<double>(inliers.indices.size()) / cloud->size();
    const auto blue_val = static_cast<uint8_t>(55 + 200 * blue_scale);
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
// cloud_pipeline
// ==========================================

template <typename PointT>
void cloud_pipeline<PointT>::process(const cloud_const_ptr &input_cloud) {
  cloud_pipeline_impl<PointT> pipeline(input_cloud);
  pipeline.run();
  colored_cloud = pipeline.make_colored_cloud();
}

// ==========================================
// Template instantations
// ==========================================

template class jarvis::cloud_pipeline<pcl::PointXYZ>;
template class jarvis::cloud_pipeline<pcl::PointXYZRGBA>;
