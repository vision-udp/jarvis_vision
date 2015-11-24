//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <jarvis/colorize.hpp>

#include <algorithm> // for set_difference
#include <chrono>    // for steady_clock
#include <iostream>  // for clog
#include <iterator>  // for back_inserter
#include <numeric>   // for std::iota
#include <cassert>   // for assert
#include <cstdint>   // for uint8_t

#include <boost/make_shared.hpp>             // for make_shared
#include <pcl/point_types.h>                 // for PointXYZ, Normal
#include <pcl/common/angles.h>               // for pcl::deg2rad
#include <pcl/features/normal_3d.h>          // for NormalEstimation
#include <pcl/search/kdtree.h>               // For KdTree
#include <pcl/search/organized.h>            // for OrganizedNeighbor
#include <pcl/segmentation/region_growing.h> // for RegionGrowing

#include <pcl/sample_consensus/sac_model_cylinder.h> // TEMPORAL
#include <pcl/sample_consensus/ransac.h>             // TEMPORAL
#include <pcl/segmentation/sac_segmentation.h>       // TEMPORAL

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
// Steady timer
// ==========================================

namespace {
class steady_timer {
  using clock = std::chrono::steady_clock;

public:
  void run(const char *task_name) {
    clog << "[ RUN      ] " << task_name << endl;
    start = clock::now();
  }

  void finish() {
    std::chrono::duration<double> seconds = clock::now() - start;
    clog << "[       OK ] " << seconds.count() << 's' << endl;
  }

private:
  clock::time_point start;
};
} // end anonymous namespace

// ==========================================
// Coloring utilities
// ==========================================
namespace {} // end anonymous namespace

// ==========================================
// Plane extraction
// ==========================================

template <typename PointT>
std::vector<PointIndices> extract_planes(const cloud_const_ptr<PointT> &cloud,
                                         shared_ptr<PointIndices> &outliers) {
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setProbability(0.5);
  seg.setMaxIterations(100);
  seg.setInputCloud(cloud);

  // remaining indices
  const auto rem = boost::make_shared<PointIndices>();
  rem->indices.resize(cloud->size());
  std::iota(rem->indices.begin(), rem->indices.end(), 0);

  ModelCoefficients coeffs;
  PointIndices inliers;
  std::vector<int> indices_tmp;
  std::vector<PointIndices> result;

  while (rem->indices.size() >= 10000) {
    seg.setIndices(rem);
    seg.segment(inliers, coeffs);
    if (inliers.indices.size() <= 10000)
      break;

    clog << "Found plane with " << inliers.indices.size() << " points" << endl;
    result.push_back(inliers);

    assert(std::is_sorted(inliers.indices.begin(), inliers.indices.end()));
    indices_tmp.clear();
    std::set_difference(rem->indices.begin(), rem->indices.end(),
                        inliers.indices.begin(), inliers.indices.end(),
                        std::back_inserter(indices_tmp));
    std::swap(rem->indices, indices_tmp);
  }
  outliers = std::move(rem);
  return result;
}

// ==========================================
// Shape Matching
// ==========================================

template <typename PointT, typename PointNT>
static double cylinder_prob(const shared_ptr<const PointCloud<PointT>> &cloud,
                            const shared_ptr<PointCloud<PointNT>> &normals,
                            const std::vector<int> &indices) {

  using model_t = pcl::SampleConsensusModelCylinder<PointT, PointNT>;
  using ransac_t = pcl::RandomSampleConsensus<PointT>;

  auto model = boost::make_shared<model_t>(cloud, indices);
  model->setInputNormals(normals);

  ransac_t ransac(model);
  ransac.setMaxIterations(1000);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();

  std::vector<int> inliers;
  ransac.getInliers(inliers);
  return static_cast<double>(inliers.size()) / indices.size();
}

// ==========================================
// cloud_pipeline definitions
// ==========================================

namespace jarvis {

cloud_pipeline::cloud_pipeline() {}

cloud_pipeline::~cloud_pipeline() {}

void cloud_pipeline::process(const cloud_ptr &input_cloud) {
  cloud = input_cloud;
  colored_cloud = make_colored_cloud(*cloud, rgba_color(255, 0, 0));
  color_generator color_gen;
  steady_timer timer;

  timer.run("Extracting planes");
  auto planes = extract_planes(cloud, nonplane_indices);
  timer.finish();
  for (const auto &indices : planes) {
    colorize(*colored_cloud, indices, color_gen.blue());
  }

  std::clog << "Remaining points: " << nonplane_indices->indices.size() << '\n';
  set_search_method();

  timer.run("Compute normals");
  estimate_normals();
  timer.finish();

  timer.run("Region growing");
  segment();
  timer.finish();

  clog << "Number of clusters is equal to " << clusters.size() << '\n';
  clog << "Processing clusters:\n";
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto &indices = clusters[i].indices;
    clog << "Cluster " << i + 1 << ":\n";
    clog << " Size = " << ' ' << indices.size() << '\n';

    timer.run("Computing cylinder prob");
    const auto cyl_prob = cylinder_prob(cloud, normals, indices);
    timer.finish();
    clog << " Cylinder prob = " << 100.0 * cyl_prob << "%\n";
    clog << endl;

    if (cyl_prob >= 0.85)
      colorize(*colored_cloud, clusters[i], color_gen.green());
    else
      colorize(*colored_cloud, clusters[i], rgba_color(128, 128, 0));
  }
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
  normals = make_shared<normals_t>();
  pcl::NormalEstimation<PointXYZ, Normal> ne;
  ne.setSearchMethod(search);
  ne.setInputCloud(cloud);
  // ne.setIndices(nonplane_indices);
  ne.setKSearch(50);
  ne.compute(*normals);
}

void cloud_pipeline::segment() {
  pcl::RegionGrowing<PointXYZ, Normal> reg;

  reg.setMinClusterSize(1000);
  reg.setMaxClusterSize(10000);
  reg.setSearchMethod(search);
  reg.setNumberOfNeighbours(500);
  reg.setInputCloud(cloud);
  reg.setIndices(nonplane_indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(pcl::deg2rad(3.0f));
  reg.setCurvatureThreshold(1.0f);

  reg.extract(clusters);
}

} // end namespace jarvis
