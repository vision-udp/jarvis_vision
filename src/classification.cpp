//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/classification.hpp>

#include <jarvis/model_recognition.hpp>

#include <boost/make_shared.hpp>    // for make_shared
#include <pcl/ModelCoefficients.h>  // for ModelCoefficients
#include <pcl/point_cloud.h>        // for PointCloud
#include <pcl/point_types.h>        // for PointXYZ, PointXYZRGBA
#include <pcl/features/normal_3d.h> // for NormalEstimation
#include <pcl/search/search.h>      // for Search
#include <pcl/search/kdtree.h>      // for KdTree

#include <algorithm> // For std::max_element
#include <iterator>  // For std::begin, std::end
#include <cstddef>   // For std::size_t
#include <cstdint>   // For SIZE_MAX

#include <iostream> // For std::clog

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::make_shared;
using boost::shared_ptr;
using pcl::ModelCoefficients;
using pcl::PointCloud;
using pcl::Normal;
using pcl::search::Search;
using std::size_t;

template <typename PointT>
using cloud_ptr = boost::shared_ptr<pcl::PointCloud<PointT>>;

template <typename PointT>
using cloud_const_ptr = boost::shared_ptr<const pcl::PointCloud<PointT>>;

// ==========================================
// Auxiliary functions
// ==========================================

template <typename PointT>
static shared_ptr<Search<PointT>>
make_search_method(const cloud_const_ptr<PointT> &) {
  using kdtree_t = pcl::search::KdTree<PointT>;
  return boost::make_shared<kdtree_t>(false);
}

template <typename PointT>
static cloud_ptr<Normal>
make_normals(const cloud_const_ptr<PointT> &cloud,
             const shared_ptr<Search<PointT>> &search) {

  pcl::NormalEstimation<PointT, Normal> ne;
  ne.setSearchMethod(search);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);

  auto normals = make_shared<PointCloud<Normal>>();
  ne.compute(*normals);
  return normals;
}

// ==========================================
// classify_object definition
// ==========================================

template <typename PointT>
object_info jarvis::classify_object(const cloud_const_ptr<PointT> &cloud) {
  const auto search = make_search_method(cloud);
  const auto normals = make_normals(cloud, search);

  model_recognition<PointT, Normal> mr;
  mr.set_input_cloud(cloud);
  mr.set_input_normals(normals);

  object_info res{};
  ModelCoefficients cyl_coeffs, sphere_coeffs;

  // Test cylinder
  const size_t cyl_id = 0;
  const size_t sphere_id = 1;
  const size_t cube_id = 2;
  double prob[3] = {};
  ModelCoefficients coeffs[3];

  prob[cyl_id] = mr.test_cylinder(coeffs[cyl_id]);
  prob[sphere_id] = mr.test_sphere(coeffs[sphere_id]);

  std::clog << std::fixed;
  std::clog.precision(2);
  std::clog << "Classifing cluster: ";
  std::clog << "cylinder prob=" << prob[cyl_id];
  std::clog << ", sphere prob=" << prob[sphere_id];
  std::clog << ", cube prob=" << prob[cube_id];
  std::clog << std::endl;

  const auto selected_it = std::max_element(std::begin(prob), std::end(prob));
  const size_t selected = static_cast<size_t>(selected_it - std::begin(prob));

  if (prob[selected] < 0.5)
    return res;

  res.probability = prob[selected];

  switch (selected) {
  case cyl_id:
    res.type = object_type::cylinder;
    assert(coeffs[cyl_id].values.size() == 7);
    res.radius = coeffs[cyl_id].values.back();
    break;
  case sphere_id:
    res.type = object_type::sphere;
    assert(coeffs[sphere_id].values.size() == 4);
    res.radius = coeffs[sphere_id].values.back();
    break;
  case cube_id:
    res.type = object_type::cube;
    break;
  }

  return res;
}

// ==========================================
// Template instantations
// ==========================================

template object_info
jarvis::classify_object(const cloud_const_ptr<pcl::PointXYZ> &cloud);

template object_info
jarvis::classify_object(const cloud_const_ptr<pcl::PointXYZRGBA> &cloud);
