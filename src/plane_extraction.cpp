//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/plane_extraction.hpp>

#include <boost/make_shared.hpp>               // for make_shared
#include <pcl/point_types.h>                   // for PointXYZ, PointXYZRGBA
#include <pcl/segmentation/sac_segmentation.h> // for SACSegmentation

#include <algorithm> // for std::is_sorted, std::set_difference
#include <iterator>  // for std::back_inserter
#include <numeric>   // for std::iota
#include <utility>   // for std::swap

// ==========================================
// Global namespace visibility
// ==========================================

using namespace jarvis;
using boost::make_shared;
using pcl::ModelCoefficients;
using pcl::PointIndices;
using pcl::Normal;

// ==========================================
// plane_extractor definitions
// ==========================================

template <typename PointT>
std::size_t plane_extractor<PointT>::extract_planes() {
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setProbability(0.2);
  seg.setMaxIterations(50);
  seg.setInputCloud(cloud);

  // Clear output variables
  inliers_of.clear();
  coeffs_of.clear();

  // remaining indices
  // remaining indices
  rem = boost::make_shared<PointIndices>();
  if (indices) {
    rem->indices = indices->indices;
  } else {
    rem->indices.resize(cloud->size());
    std::iota(rem->indices.begin(), rem->indices.end(), 0);
  }

  ModelCoefficients coeffs;
  PointIndices inliers;
  std::vector<int> indices_tmp;
  std::vector<PointIndices> result;

  while (rem->indices.size() >= min_points) {
    seg.setIndices(rem);
    seg.segment(inliers, coeffs);
    if (inliers.indices.size() <= min_points)
      break;

    inliers_of.push_back(inliers);
    coeffs_of.push_back(coeffs);

    assert(std::is_sorted(inliers.indices.begin(), inliers.indices.end()));
    indices_tmp.clear();
    std::set_difference(rem->indices.begin(), rem->indices.end(),
                        inliers.indices.begin(), inliers.indices.end(),
                        std::back_inserter(indices_tmp));
    std::swap(rem->indices, indices_tmp);
  }

  assert(inliers_of.size() == coeffs_of.size());
  return get_num_planes();
}

template <typename PointT>
std::size_t plane_extractor<PointT>::extract_planes_from_normals() {
  assert(false && "Don't use");
  return 0;
  //  pcl::SACSegmentationFromNormals<PointT, Normal> seg;
  //  seg.setOptimizeCoefficients(true);
  //  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  //  seg.setMethodType(pcl::SAC_RANSAC);
  //  seg.setDistanceThreshold(0.03);
  //  seg.setProbability(0.2);
  //  seg.setMaxIterations(100);
  //  seg.setDistanceFromOrigin(1.5);
  //  seg.setInputCloud(cloud);
  //  seg.setInputNormals(normals);
  //
  //  // Clear output variables
  //  inliers_of.clear();
  //  coeffs_of.clear();
  //
  //  // remaining indices
  //  rem = boost::make_shared<PointIndices>();
  //  if (indices) {
  //    rem->indices = indices->indices;
  //  } else {
  //    rem->indices.resize(cloud->size());
  //    std::iota(rem->indices.begin(), rem->indices.end(), 0);
  //  }
  //
  //  ModelCoefficients coeffs;
  //  PointIndices inliers;
  //  std::vector<int> indices_tmp;
  //  std::vector<PointIndices> result;
  //
  //  while (rem->indices.size() >= 10000) {
  //    seg.setIndices(rem);
  //    seg.segment(inliers, coeffs);
  //    if (inliers.indices.size() <= 10000)
  //      break;
  //
  //    inliers_of.push_back(inliers);
  //    coeffs_of.push_back(coeffs);
  //
  //    assert(std::is_sorted(inliers.indices.begin(), inliers.indices.end()));
  //    indices_tmp.clear();
  //    std::set_difference(rem->indices.begin(), rem->indices.end(),
  //                        inliers.indices.begin(), inliers.indices.end(),
  //                        std::back_inserter(indices_tmp));
  //    std::swap(rem->indices, indices_tmp);
  //  }
  //
  //  assert(inliers_of.size() == coeffs_of.size());
  //  return get_num_planes();
}

// ==========================================
// Template instantations
// ==========================================

template class jarvis::plane_extractor<pcl::PointXYZ>;
template class jarvis::plane_extractor<pcl::PointXYZRGBA>;
