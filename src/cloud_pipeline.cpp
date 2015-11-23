//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_pipeline.hpp>

#include <chrono>                            // for steady_clock
#include <iostream>                          // for clog
#include <boost/make_shared.hpp>             // for make_shared
#include <pcl/point_types.h>                 // for PointXYZ, Normal
#include <pcl/common/angles.h>               // for pcl::deg2rad
#include <pcl/features/normal_3d.h>          // for NormalEstimation
#include <pcl/search/kdtree.h>               // For KdTree
#include <pcl/search/organized.h>            // for OrganizedNeighbor
#include <pcl/segmentation/region_growing.h> // for RegionGrowing

using boost::make_shared;
using pcl::PointXYZ;
using pcl::Normal;
using std::clog;
using std::endl;

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

  clog << "Number of clusters is equal to " << clusters.size() << '\n';
  clog << "The cluster sizes are:\n";
  for (const auto &cluster : clusters)
    clog << cluster.indices.size() << '\n';
  clog << endl;
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
  colored_cloud = reg.getColoredCloudRGBA();
}

} // end namespace jarvis
