//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/classification.hpp>

#include <jarvis/any_map.hpp>
#include <jarvis/model_recognition.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
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

private:
  cloud_const_ptr cloud;
  search_ptr search;
  normals_ptr normals;
};
} // end anonymous namespace

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
