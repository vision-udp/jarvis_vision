//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/model_recognition.hpp>

#include <boost/make_shared.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector> // for std::vector

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using pcl::ModelCoefficients;
using pcl::PointIndices;

// ==========================================
// model_recognition function definitions
// ==========================================

template <typename PointT, typename PointNT>
double
model_recognition<PointT, PointNT>::test_cylinder(ModelCoefficients &coeffs) {

  pcl::SACSegmentationFromNormals<PointT, PointNT> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.04);
  seg.setProbability(0.7);
  seg.setRadiusLimits(0.01, 0.08);
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  // Obtain the cylinder inliers and coefficients
  PointIndices inliers;
  seg.segment(inliers, coeffs);

  return static_cast<double>(inliers.indices.size()) / cloud->size();
}

template <typename PointT, typename PointNT>
double
model_recognition<PointT, PointNT>::test_sphere(ModelCoefficients &coeffs) {

  // SACMODEL_NORMAL_SPHERE was probbed and it seems that SACMODEL_SPHERE gives
  // better results. Also this has the advantage of obtaining the center
  // directly.

  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.005);
  seg.setProbability(0.05);
  seg.setRadiusLimits(0.01, 0.08);
  seg.setInputCloud(cloud);

  // Obtain the cylinder inliers and coefficients
  PointIndices inliers;
  seg.segment(inliers, coeffs);

  return static_cast<double>(inliers.indices.size()) / cloud->size();
}

// ==========================================
// Template instantations
// ==========================================

template class jarvis::model_recognition<pcl::PointXYZ, pcl::Normal>;
template class jarvis::model_recognition<pcl::PointXYZRGBA, pcl::Normal>;
