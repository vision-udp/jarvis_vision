//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <chrono>   // for steady_clock
#include <iostream> // for clog
#include <random>   // for uniform_int_distribution, default_random_engine
#include <cstdint>  // for uint8_t

#include <boost/make_shared.hpp>             // for make_shared
#include <pcl/point_types.h>                 // for PointXYZ, Normal
#include <pcl/common/angles.h>               // for pcl::deg2rad
#include <pcl/features/normal_3d.h>          // for NormalEstimation
#include <pcl/search/kdtree.h>               // For KdTree
#include <pcl/search/organized.h>            // for OrganizedNeighbor
#include <pcl/segmentation/region_growing.h> // for RegionGrowing

#include <pcl/sample_consensus/sac_model_cylinder.h> // TEMPORAL
#include <pcl/sample_consensus/ransac.h>             // TEMPORAL

// ==========================================
// Using directives
// ==========================================

using boost::make_shared;
using boost::shared_ptr;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGBA;
using pcl::Normal;
using std::size_t;
using std::uint8_t;
using std::clog;
using std::endl;

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
// Colored cloud functions
// ==========================================

namespace {
struct rgba_color {
  uint8_t r, g, b, a;
};
} // end anonymous namespace

static shared_ptr<PointCloud<PointXYZRGBA>>
make_colored_cloud(const PointCloud<PointXYZ> &cloud) {

  using colored_cloud_t = PointCloud<PointXYZRGBA>;

  const size_t N = cloud.size();
  auto colored_cloud = make_shared<colored_cloud_t>();
  colored_cloud->points.resize(N);
  colored_cloud->width = cloud.width;
  colored_cloud->height = cloud.height;
  colored_cloud->is_dense = cloud.is_dense;

  for (size_t i = 0; i < N; ++i) {
    const auto &p = cloud.points[i];
    auto &cp = colored_cloud->points[i];
    cp.x = p.x, cp.y = p.y, cp.z = p.z;
    cp.r = 255, cp.g = 0, cp.b = 0, cp.a = 0;
  }

  return colored_cloud;
}

template <typename RandomEngine>
static rgba_color random_green(RandomEngine &engine) {
  rgba_color color{};
  std::uniform_int_distribution<> gen_value(128, 255);
  color.g = static_cast<uint8_t>(gen_value(engine));
  return color;
}

template <typename RandomEngine>
static rgba_color random_blue(RandomEngine &engine) {
  rgba_color color{};
  std::uniform_int_distribution<> gen_value(128, 255);
  color.b = static_cast<uint8_t>(gen_value(engine));
  return color;
}

static void colorize(PointCloud<PointXYZRGBA> &cloud,
                     const std::vector<int> &indices, const rgba_color color) {
  for (int idx : indices) {
    auto &p = cloud[static_cast<size_t>(idx)];
    p.r = color.r, p.g = color.g, p.b = color.b, p.a = color.a;
  }
}

// ==========================================
// Internal classifiers
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

  steady_timer timer;
  set_search_method();

  timer.run("Compute normals");
  estimate_normals();
  timer.finish();

  timer.run("Region growing");
  segment();
  timer.finish();

  colored_cloud = make_colored_cloud(*cloud);
  std::default_random_engine engine;

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

    if (cyl_prob > 0.85)
      colorize(*colored_cloud, indices, random_green(engine));
    else
      colorize(*colored_cloud, indices, random_blue(engine));
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
  pcl::NormalEstimation<PointXYZ, Normal> normal_estimator;
  normal_estimator.setSearchMethod(search);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);

  normal_estimator.compute(*normals);
}

void cloud_pipeline::segment() {
  pcl::RegionGrowing<PointXYZ, Normal> reg;

  reg.setMinClusterSize(1000);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(search);
  reg.setNumberOfNeighbours(350);
  reg.setInputCloud(cloud);
  // reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(pcl::deg2rad(3.0f));
  reg.setCurvatureThreshold(1.0f);

  reg.extract(clusters);
}

} // end namespace jarvis
