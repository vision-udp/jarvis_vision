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

template <typename PointT>
class pcd_visualizer {

public:
  using cloud_ptr = typename pcl::PointCloud<PointT>::Ptr;

public:
  pcd_visualizer(const char *filename) : cloud(new pcl::PointCloud<PointT>) {
    using pcl::console::print_error;
    using namespace std::chrono_literals;

    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    viewer.setCameraPosition(0.0, -0.3, -0.2, 0.0, -0.3, 1.0, 0.0, -1.0, 0.0);

    try {
      pcl::io::loadPCDFile<PointT>(filename, *cloud);
    } catch (pcl::PCLException e) {
      print_error("Error reading %s\n", filename);
    }

    viewer.addPointCloud(cloud, "Cloud");

    while (!viewer.wasStopped()) {
      std::this_thread::sleep_for(100ms);
      viewer.spinOnce();
    }
  }

private:
  pcl::visualization::PCLVisualizer viewer;
  cloud_ptr cloud;
};

int main(int argc, char *argv[]) {
  using pcl::console::print_error;
  if (argc < 2) {
    print_error("Wrong number of arguments.\n");
    return -1;
  }

  pcl::PCLPointCloud2 cloud_meta;
  try {
    pcl::io::loadPCDFile(argv[1], cloud_meta);
  } catch (pcl::PCLException e) {
    print_error("Error reading file %s\n", argv[1]);
  }

  std::cerr << "Fields: ";
  for (const auto &field : cloud_meta.fields)
    std::cerr << field.name << ' ';
  std::cerr << '\n';

  if (cloud_meta.fields.size() > 3)
    pcd_visualizer<pcl::PointXYZRGBA> v(argv[1]);
  else if (cloud_meta.fields.size() == 3)
    pcd_visualizer<pcl::PointXYZ> v(argv[1]);
  else
    print_error("Unknown type.\n");
}
