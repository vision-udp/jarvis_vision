//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/model_recognition.hpp>

#include <boost/make_shared.hpp>
#include <boost/range/algorithm/set_algorithm.hpp>
#include <boost/range/algorithm_ext/iota.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/filter.h> // for removeNaNFromPointCloud
#include <pcl/segmentation/sac_segmentation.h>

#include <cmath>    // for std::fabs
#include <iterator> // for std::back_inserter
#include <utility>  // for std::swap

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::make_shared;
using pcl::ModelCoefficients;
using pcl::PointIndices;
using pcl::PointCloud;
using pcl::SACSegmentation;
using pcl::SACSegmentationFromNormals;
using Eigen::Vector3f;

// ==========================================
// Auxiliary functions
// ==========================================

template <typename PointT>
static double inliers_ratio(const PointIndices &inliers,
                            const PointCloud<PointT> &cloud) {
  return static_cast<double>(inliers.indices.size()) / cloud.size();
}

static Vector3f get_plane_normal(const ModelCoefficients &coeffs) {
  return {coeffs.values[0], coeffs.values[1], coeffs.values[2]};
}

// ==========================================
// model_recognition function definitions
// ==========================================

template <typename PointT, typename PointNT>
double
model_recognition<PointT, PointNT>::test_cylinder(ModelCoefficients &coeffs) {

  SACSegmentationFromNormals<PointT, PointNT> seg;
  PointIndices inliers;

  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setRadiusLimits(0.01, 0.15);
  seg.setDistanceThreshold(0.04);
  seg.setNormalDistanceWeight(0.1);

  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(500);

  seg.segment(inliers, coeffs);

  if (inliers_ratio(inliers, *cloud) < 0.68)
    return 0.0;

  return inliers_ratio(inliers, *cloud);
}

template <typename PointT, typename PointNT>
double
model_recognition<PointT, PointNT>::test_sphere(ModelCoefficients &coeffs) {

  // SACMODEL_NORMAL_SPHERE was tried and it seems that SACMODEL_SPHERE gives
  // better results. Also this has the advantage of obtaining the center of the
  // sphere directly.

  SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(false);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold(0.005);
  seg.setProbability(0.9);
  seg.setRadiusLimits(0.01, 0.10);
  seg.setInputCloud(cloud);

  PointIndices inliers;
  seg.segment(inliers, coeffs);

  if (inliers_ratio(inliers, *cloud) < 0.5)
    return 0.0;

  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(100);
  seg.segment(inliers, coeffs);

  if (inliers_ratio(inliers, *cloud) < 0.9)
    return 0.0;

  return inliers_ratio(inliers, *cloud);
}

template <typename PointT, typename PointNT>
double model_recognition<PointT, PointNT>::test_cube() {

  SACSegmentationFromNormals<PointT, PointNT> seg;
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setProbability(0.9);
  seg.setMaxIterations(60);
  seg.setOptimizeCoefficients(true);
  seg.setDistanceThreshold(0.02);
  seg.setNormalDistanceWeight(0.03);

  PointIndices inliers;
  ModelCoefficients coeffs;

  // Find first plane
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.segment(inliers, coeffs);

  const double plane1_ratio = inliers_ratio(inliers, *cloud);
  const Vector3f plane1_normal = get_plane_normal(coeffs);

  if (plane1_ratio < 0.2 || plane1_ratio > 0.8)
    return 0.0;

  // Ok, now find second plane

  // First extract remaining indices
  auto remaining = make_shared<std::vector<int>>(cloud->size());
  boost::iota(*remaining, 0);

  auto remove_inliers = [&, tmp = std::vector<int>() ]() mutable {
    tmp.clear();
    boost::set_difference(*remaining, inliers.indices, std::back_inserter(tmp));
    std::swap(*remaining, tmp);
  };

  // Find second plane
  // =================
  remove_inliers();
  seg.setMaxIterations(30);
  seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  seg.setAxis(plane1_normal);
  seg.setIndices(remaining);
  seg.segment(inliers, coeffs);

  const double plane2_ratio = inliers_ratio(inliers, *cloud);
  const Vector3f plane2_normal = get_plane_normal(coeffs);

  const auto dot = std::fabs(plane1_normal.dot(plane2_normal));
  if (dot >= 0.15)
    return 0;

  if (plane2_ratio < 0.2 || plane2_ratio > 0.8)
    return 0;

  if (plane1_ratio + plane2_ratio < 0.8)
    return 0;

  // PD: Third plane search is still remaining as still there is no sample.
  return plane1_ratio + plane2_ratio;
}

// ==========================================
// Template instantations
// ==========================================

#include <pcl/point_types.h>

template class jarvis::model_recognition<pcl::PointXYZ, pcl::Normal>;
template class jarvis::model_recognition<pcl::PointXYZRGBA, pcl::Normal>;
