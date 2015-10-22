/// \file pcl_viewer.cpp
/// \brief Visualize .pcd files
/// \author Jorge Aguirre
/// \version 0.1
/// \date 2015-10-21

#include <chrono>
#include <thread>
#include <iostream>

#include <boost/make_shared.hpp>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char *argv[]) {
  using pcl::console::print_error;
  using namespace std::chrono_literals;

  if (argc < 2) {
    print_error("Wrong number of arguments.\n");
    return -1;
  }

  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  try {
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);
  } catch (pcl::PCLException e) {
    print_error("Error reading %s\n", argv[1]);
  }

  pcl::visualization::PCLVisualizer viewer;

  viewer.addPointCloud(cloud, "Cloud");

  while (!viewer.wasStopped()) {
    std::this_thread::sleep_for(100ms);
    viewer.spinOnce();
  }
}
