//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <jarvis/classification.hpp>
#include <jarvis/colorize.hpp>
#include <jarvis/filtering.hpp>
#include <jarvis/plane_extraction.hpp>
#include <jarvis/steady_timer.hpp>

#include <boost/make_shared.hpp>
#include <cstdint>  // for uint8_t
#include <iostream> // for clog
#include <sstream>
#include <vector>

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

// ==========================================
// Auxiliary
// ==========================================

template <typename T>
static void print_as(const boost::any &any, std::ostream &os) {
  os << boost::any_cast<const T &>(any);
}

static void print_any(const boost::any &any, std::ostream &os) {
  const auto &type = any.type();
  if (type == typeid(std::string))
    print_as<std::string>(any, os);
  else if (type == typeid(const char *))
    print_as<const char *>(any, os);
  else if (type == typeid(float))
    print_as<float>(any, os);
  else if (type == typeid(double))
    print_as<double>(any, os);
  else
    os << "UNPRINTABLE";
}

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
    using std::clog;
    using std::endl;
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

    timer.run("clustering");
    clusterize();
    timer.finish();
    clog << "Number of found clusters: " << clusters.size() << endl;

    timer.run("classifying");
    classify_clusters();
    timer.finish();
  }

  // shared_ptr<PointCloud<PointXYZRGBA>> make_colored_cloud() const;
  const std::vector<PointIndices> &get_clusters() const { return clusters; }
  const std::vector<any_map> &get_clusters_info() const {
    return clusters_info;
  }

  shared_ptr<const cloud_t> get_filtered_cloud() { return cloud; }

private:
  void apply_filters() {
    cloud_filter<PointT> filter;
    filter.set_input_cloud(original_cloud);
    filter.set_desired_num_of_points(50000);
    cloud = filter.filter_input_cloud();
  }

  void extract_planes() {
    plane_extractor.set_input_cloud(cloud);
    plane_extractor.set_min_points(3500);
    plane_extractor.extract_planes();
  }

  void clusterize() {
    using kdtree_t = pcl::search::KdTree<PointT>;
    const auto search = make_shared<kdtree_t>(false);

    const auto remaining = plane_extractor.get_remaining_indices();
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(3500);
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
  shared_ptr<const cloud_t> cloud;
  plane_extractor<PointT> plane_extractor;
  std::vector<PointIndices> clusters;
  std::vector<any_map> clusters_info;
};
} // end anonymous namespace

// template <typename PointT>
// shared_ptr<PointCloud<PointXYZRGBA>>
// cloud_pipeline_impl<PointT>::make_colored_cloud() const {
//  const rgba_color red{255, 0, 0};
//  return jarvis::make_colored_cloud(*cloud, red);
//}
//
// template <>
// shared_ptr<PointCloud<PointXYZRGBA>>
// cloud_pipeline_impl<PointXYZRGBA>::make_colored_cloud() const {
//  using std::clog;
//  using std::endl;
//  timer.run("Creating colored cloud");
//  const rgba_color green{0, 255, 0};
//  const rgba_color cyan{0, 255, 255};
//  const rgba_color magenta{255, 0, 255};
//  const rgba_color yellow{128, 128, 0};
//  std::map<std::string, rgba_color> color_map;
//  color_map["cylinder"] = green;
//  color_map["sphere"] = cyan;
//  color_map["cuboid"] = magenta;
//
//  const auto colored_cloud = make_shared<cloud_t>(*cloud);
//
//  // No plane coloring:
//  //  const size_t num_planes = plane_extractor.get_num_planes();
//  //  for (size_t i = 0; i < num_planes; ++i) {
//  //    const auto &inliers = plane_extractor.get_inliers(i);
//  //    const auto blue_scale =
//  //        static_cast<double>(inliers.indices.size()) / cloud->size();
//  //    const auto blue_val = static_cast<uint8_t>(55 + 200 * blue_scale);
//  //    colorize(*colored_cloud, inliers, rgba_color(0, 0, blue_val));
//  //  }
//
//  for (size_t i = 0; i < clusters.size(); ++i) {
//    //    clog << "Cluster " << i + 1 << ": ";
//    //    clog << "Size=" << clusters[i].indices.size();
//    const any_map &info = clusters_info[i];
//
//    //    for (const auto &elem : info) {
//    //      clog << ", " << elem.first << '=';
//    //      print_any(elem.second, clog);
//    //    }
//    //    clog << endl;
//
//    const auto &shape = info.get<std::string>("shape");
//    if (shape != "unknown")
//      colorize(*colored_cloud, clusters[i], color_map.at(shape));
//  }
//  clog << endl;
//
//  timer.finish();
//  return colored_cloud;
//}

// ==========================================
// cloud_pipeline
// ==========================================

template <typename PointT>
void cloud_pipeline<PointT>::process(const cloud_const_ptr &input_cloud) {
  cloud_pipeline_impl<PointT> pipeline(input_cloud);
  pipeline.run();
  filtered_cloud = pipeline.get_filtered_cloud();
  clusters = pipeline.get_clusters();
}

// ==========================================
// pipeline_searcher
// ==========================================

static std::string generate_info_string(const any_map &info) {
  std::ostringstream oss;
  for (const auto &elem : info) {
    oss << elem.first << '=';
    print_any(elem.second, oss);
    oss << '\n';
  }
  return oss.str();
}

template <>
void pipeline_searcher<PointXYZRGBA>::update_cloud(
    cloud_const_ptr input_cloud) {

  for (size_t i = 0; i < last_num_of_clusters; ++i)
    vis.remove_text_3d("text_" + std::to_string(i));

  cloud_pipeline_impl<PointXYZRGBA> pipeline(input_cloud);
  pipeline.run();
  const auto &cloud = pipeline.get_filtered_cloud();
  const auto &clusters = pipeline.get_clusters();
  const auto &clusters_info = pipeline.get_clusters_info();
  last_num_of_clusters = clusters.size();

  const rgba_color green{0, 255, 0};
  const rgba_color cyan{0, 255, 255};
  const rgba_color magenta{255, 0, 255};
  const rgba_color yellow{128, 128, 0};
  std::map<std::string, rgba_color> color_map;
  color_map["cylinder"] = green;
  color_map["sphere"] = cyan;
  color_map["cuboid"] = magenta;

  const auto colored_cloud = make_shared<cloud_t>(*cloud);
  for (size_t i = 0; i < clusters.size(); ++i) {
    const any_map &info = clusters_info[i];

    const auto &shape = info.get<std::string>("shape");
    if (shape == "unknown")
      continue;

    assert(!clusters[i].indices.empty());

    colorize(*colored_cloud, clusters[i], color_map.at(shape));
    const auto representative_point =
        (*cloud)[static_cast<size_t>(clusters[i].indices[0])];
    vis.add_text_3d(generate_info_string(info), representative_point,
                    "text_" + std::to_string(i));
  }

  vis.show_cloud(colored_cloud, "cloud");
}

// ==========================================
// Template instantations
// ==========================================

template class jarvis::cloud_pipeline<pcl::PointXYZ>;
template class jarvis::cloud_pipeline<pcl::PointXYZRGBA>;

template class jarvis::pipeline_searcher<pcl::PointXYZ>;
template class jarvis::pipeline_searcher<pcl::PointXYZRGBA>;
