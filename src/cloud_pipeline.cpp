//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <jarvis/colorize.hpp>
#include <jarvis/plane_extraction.hpp>
#include <jarvis/steady_timer.hpp>

#include <iostream> // for clog
#include <cassert>  // for assert
#include <cstdint>  // for uint8_t

#include <boost/make_shared.hpp>    // for make_shared
#include <pcl/filters/filter.h>     // for removeNaNFromPointCloud
#include <pcl/point_types.h>        // for PointXYZ, Normal
#include <pcl/common/angles.h>      // for pcl::deg2rad
#include <pcl/features/normal_3d.h> // for NormalEstimation
#include <pcl/search/kdtree.h>      // For KdTree
#include <pcl/search/organized.h>   // for OrganizedNeighbor
//#include <pcl/segmentation/region_growing.h> // for RegionGrowing
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/sac_model_cylinder.h> // TEMPORAL
#include <pcl/sample_consensus/ransac.h>             // TEMPORAL

// ==========================================
// Using directives
// ==========================================

using jarvis::rgba_color;

using boost::make_shared;
using boost::shared_ptr;
using pcl::ModelCoefficients;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZ;
using pcl::PointXYZRGBA;
using pcl::Normal;
using std::size_t;
using std::uint8_t;
using std::clog;
using std::endl;

template <typename PointT>
using cloud_ptr = boost::shared_ptr<PointCloud<PointT>>;

template <typename PointT>
using cloud_const_ptr = boost::shared_ptr<const PointCloud<PointT>>;

// ==========================================
// Shape Matching
// ==========================================

template <typename PointT, typename PointNT>
static double cylinder_prob(const shared_ptr<PointCloud<PointT>> &cloud,
                            const shared_ptr<PointCloud<PointNT>> &normals,
                            const std::vector<int> &indices,
                            ModelCoefficients &coeffs) {

  using model_t = pcl::SampleConsensusModelCylinder<PointT, PointNT>;
  using ransac_t = pcl::RandomSampleConsensus<PointT>;

  auto model = boost::make_shared<model_t>(cloud, indices);
  model->setInputNormals(normals);
  model->setRadiusLimits(0.001, 0.1);

  ransac_t ransac(model);
  ransac.setMaxIterations(1000);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();

  std::vector<int> inliers;
  ransac.getInliers(inliers);
  Eigen::VectorXf model_coeffs;
  ransac.getModelCoefficients(model_coeffs);

  coeffs.values.resize(static_cast<size_t>(model_coeffs.size()));
  for (size_t i = 0; i < coeffs.values.size(); ++i) {
    auto idx = static_cast<Eigen::VectorXf::Index>(i);
    coeffs.values[i] = model_coeffs[idx];
  }

  return static_cast<double>(inliers.size()) / indices.size();
}

// ==========================================
// cloud_pipeline definitions
// ==========================================

namespace jarvis {

cloud_pipeline::cloud_pipeline() {}

cloud_pipeline::~cloud_pipeline() {}

void cloud_pipeline::process(const cloud_const_ptr &input_cloud) {
  steady_timer timer;

  clog << "Total points: " << input_cloud->size() << std::endl;
  {
    cloud = make_shared<cloud_t>();
    std::vector<int> indices;
    timer.run("Removing NaNs");
    pcl::removeNaNFromPointCloud(*input_cloud, *cloud, indices);
    timer.finish();
  }
  clog << "Total NonNans points: " << cloud->size() << endl;

  colored_cloud = make_colored_cloud(*cloud, rgba_color(255, 0, 0));
  color_generator color_gen;

  timer.run("Extracting planes");
  plane_extractor<PointXYZ> plane_extractor;
  plane_extractor.set_input_cloud(cloud);
  plane_extractor.extract_planes();
  nonplane_indices = plane_extractor.get_remaining_indices();
  const size_t num_planes = plane_extractor.get_num_planes();
  timer.finish();
  std::clog << "Number of found planes: " << num_planes << '\n';

  for (size_t i = 0; i < num_planes; ++i) {
    const auto &inliers = plane_extractor.get_inliers(i);
    colorize(*colored_cloud, inliers, color_gen.blue());
  }

  std::clog << "Remaining points: " << nonplane_indices->indices.size() << '\n';
  set_search_method();

  timer.run("Extracting clusters");
  segment();
  timer.finish();

  timer.run("Computing per cluster normals");
  estimate_normals();
  timer.finish();

  timer.run("Processing clusters");
  clog << "Number of clusters is equal to " << clusters.size() << '\n';
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto &indices = clusters[i].indices;
    ModelCoefficients coeffs;
    const auto cyl_prob = cylinder_prob(cloud, normals, indices, coeffs);
    assert(coeffs.values.size() == 7);
    clog << " Size = " << indices.size() << ", ";
    clog << " estimated radius = " << coeffs.values[6] << ", ";
    clog << " Cylinder prob = " << 100.0 * cyl_prob << "%\n";
    if (cyl_prob >= 0.85) {
      uint8_t green_intensity = static_cast<uint8_t>(255 * cyl_prob);
      colorize(*colored_cloud, clusters[i], rgba_color(0, green_intensity, 0));
    } else
      colorize(*colored_cloud, clusters[i], rgba_color(128, 128, 0));
  }
  timer.finish();
  clog << std::endl;
}

void cloud_pipeline::set_search_method() {
  using kdtree_search = pcl::search::KdTree<PointXYZ>;
  using organized_search = pcl::search::OrganizedNeighbor<PointXYZ>;
  if (cloud->isOrganized()) {
    search = make_shared<organized_search>(false, 1e-3f, 10);
  } else {
    search = make_shared<kdtree_search>(false);
  }
}

void cloud_pipeline::estimate_normals() {
  pcl::NormalEstimation<PointXYZ, Normal> ne;
  ne.setSearchMethod(search);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);

  normals = boost::make_shared<normals_t>();
  normals->points.resize(cloud->size());

  auto normals_tmp = make_shared<normals_t>();
  auto indices_tmp = make_shared<PointIndices>();

  for (const auto &cluster : clusters) {
    indices_tmp->indices = cluster.indices;
    ne.setIndices(indices_tmp);
    ne.compute(*normals_tmp);

    for (size_t i = 0; i < cluster.indices.size(); ++i) {
      const auto mapped_i = static_cast<size_t>(cluster.indices[i]);
      normals->points[mapped_i] = normals_tmp->points[i];
    }
  }
}

void cloud_pipeline::segment() {
  //  pcl::RegionGrowing<PointXYZ, Normal> reg;
  //
  //  reg.setMinClusterSize(500);
  //  reg.setMaxClusterSize(10000);
  //  reg.setSearchMethod(search);
  //  reg.setNumberOfNeighbours(500);
  //  reg.setInputCloud(cloud);
  //  reg.setIndices(nonplane_indices);
  //  reg.setInputNormals(normals);
  //  reg.setSmoothnessThreshold(pcl::deg2rad(3.0f));
  //  reg.setCurvatureThreshold(1.0f);
  //
  //  reg.extract(clusters);
  pcl::EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(1000);
  ec.setMaxClusterSize(15000);
  ec.setSearchMethod(search);
  ec.setInputCloud(cloud);
  ec.setIndices(nonplane_indices);
  ec.extract(clusters);
}

} // end namespace jarvis
