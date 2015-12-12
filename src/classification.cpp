//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/classification.hpp>

#include <jarvis/any_map.hpp>
#include <jarvis/colorize.hpp>
#include <jarvis/model_recognition.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> // for PointXYZRGBA
#include <pcl/search/kdtree.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/make_shared.hpp>
#include <boost/range/algorithm/max_element.hpp>

#include <array>
#include <string>

#include <cstddef> // for std::size_t

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::algorithm::clamp;
using boost::shared_ptr;
using pcl::ModelCoefficients;
using pcl::PointCloud;
using pcl::Normal;
using pcl::PointXYZRGBA;
using std::size_t;

// ==========================================
// Auxiliary functions
// ==========================================

namespace {

template <typename PointT>
class classifier_impl {
  using cloud_t = PointCloud<PointT>;
  using cloud_const_ptr = shared_ptr<const cloud_t>;
  using normals_t = PointCloud<Normal>;
  using normals_ptr = shared_ptr<normals_t>;
  using search_t = pcl::search::Search<PointT>;
  using search_ptr = shared_ptr<search_t>;

public:
  any_map classify(const cloud_const_ptr &input_cloud);

private:
  void make_search_method() {
    using kdtree_t = pcl::search::KdTree<PointT>;
    search = boost::make_shared<kdtree_t>(false);
  }

  void make_normals() {
    pcl::NormalEstimation<PointT, Normal> ne;
    ne.setSearchMethod(search);
    ne.setInputCloud(cloud);

    const auto optimal_k = static_cast<int>(cloud->size() * 0.1);
    ne.setKSearch(clamp(optimal_k, 10, 80));

    normals = boost::make_shared<PointCloud<Normal>>();
    ne.compute(*normals);
  }

  std::string named_avg_color() const;

private:
  cloud_const_ptr cloud;
  search_ptr search;
  normals_ptr normals;
};
} // end anonymous namespace

template <typename PointT>
std::string classifier_impl<PointT>::named_avg_color() const {
  return "No color information";
}

template <>
std::string classifier_impl<PointXYZRGBA>::named_avg_color() const {
  unsigned long rsum{}, gsum{}, bsum{};
  for (const auto &p : *cloud) {
    rsum += p.r;
    gsum += p.g;
    bsum += p.b;
  }
  rgba_color avg_color{static_cast<uint8_t>(rsum / cloud->size()),
                       static_cast<uint8_t>(gsum / cloud->size()),
                       static_cast<uint8_t>(bsum / cloud->size())};

  // FIXME: The following approach was a quick solution to guess the color of
  // an object, use HSV based approach instead.

  using pair_t = std::pair<rgba_color, std::string>;
  std::vector<pair_t> pairs{{rgba_color(0, 0, 0), "black"},
                            {rgba_color(255, 255, 255), "white"},
                            {rgba_color(128, 0, 0), "red"},
                            {rgba_color{0, 128, 0}, "green"},
                            {rgba_color{0, 0, 128}, "blue"},
                            {rgba_color(255, 0, 0), "light-red"},
                            {rgba_color{0, 255, 0}, "light-green"},
                            {rgba_color{0, 0, 255}, "light-blue"},
                            {rgba_color{255, 255, 0}, "light-yellow"},
                            {rgba_color(128, 128, 0), "yellow"},
                            {rgba_color{0, 255, 255}, "cyan"},
                            {rgba_color(255, 0, 255), "magenta"},
                            {rgba_color{192, 192, 192}, "silver"},
                            {rgba_color(128, 128, 128), "gray"},
                            {rgba_color{128, 0, 128}, "purple"},
                            {rgba_color(0, 128, 128), "teal"}};

  auto get_dist2 = [avg_color](const rgba_color color) {
    int r_diff = int(color.r) - int(avg_color.r);
    int g_diff = int(color.g) - int(avg_color.g);
    int b_diff = int(color.b) - int(avg_color.b);
    return r_diff * r_diff + g_diff * g_diff + b_diff * b_diff;
  };

  auto near = [get_dist2](const pair_t &lhs, const pair_t &rhs) {
    return get_dist2(lhs.first) < get_dist2(rhs.first);
  };

  auto it = std::min_element(pairs.begin(), pairs.end(), near);
  return it->second;
}

template <typename PointT>
any_map classifier_impl<PointT>::classify(const cloud_const_ptr &input_cloud) {
  cloud = input_cloud;
  make_search_method();
  make_normals();

  model_recognition<PointT, Normal> mr;
  mr.set_input_cloud(cloud);
  mr.set_input_normals(normals);

  any_map res;
  ModelCoefficients cyl_coeffs, sphere_coeffs;

  // Test cylinder
  const size_t cyl_id = 0;
  const size_t sphere_id = 1;
  const size_t cube_id = 2;
  std::array<double, 3> prob{};
  std::array<ModelCoefficients, 3> coeffs{};

  prob[cyl_id] = mr.test_cylinder(coeffs[cyl_id]);
  prob[sphere_id] = mr.test_sphere(coeffs[sphere_id]);
  prob[cube_id] = mr.test_cube();

  const auto selected =
      static_cast<size_t>(boost::max_element(prob) - prob.begin());

  if (prob[selected] < 0.5) {
    res["shape"] = std::string("unknown");
    return res;
  }

  res["probability"] = prob[selected];
  res["color"] = named_avg_color();

  switch (selected) {
  case cyl_id:
    res["shape"] = std::string("cylinder");
    assert(coeffs[cyl_id].values.size() == 7);
    res["radius"] = coeffs[cyl_id].values.back();
    break;
  case sphere_id:
    res["shape"] = std::string("sphere");
    assert(coeffs[sphere_id].values.size() == 4);
    res["radius"] = coeffs[sphere_id].values.back();
    break;
  case cube_id:
    res["shape"] = std::string("cuboid");
    break;
  }

  assert(res.at("shape").type() == typeid(std::string));
  return res;
}

// ==========================================
// classify_object definition
// ==========================================

template <typename PointT>
any_map
jarvis::classify_object(const shared_ptr<const PointCloud<PointT>> &cloud) {
  classifier_impl<PointT> classifier;
  return classifier.classify(cloud);
}

// ==========================================
// Template instantations
// ==========================================

#include <pcl/point_types.h> // for PointXYZ, PointXYZRGBA

template any_map jarvis::classify_object(
    const shared_ptr<const PointCloud<pcl::PointXYZ>> &cloud);

template any_map jarvis::classify_object(
    const shared_ptr<const PointCloud<pcl::PointXYZRGBA>> &cloud);
