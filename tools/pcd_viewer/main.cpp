/// \file pcl_viewer.cpp
/// \brief Visualize .pcd files
/// \author Jorge Aguirre
/// \version 0.1
/// \date 2015-10-21

#include <chrono>
#include <thread>
#include <iostream>

#include <jarvis/simple_visualizer.hpp>

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

    try {
      pcl::io::loadPCDFile<PointT>(filename, *cloud);
    } catch (pcl::PCLException e) {
      print_error("Error reading %s\n", filename);
    }

    viewer.start();
    viewer.show_cloud(cloud, "Cloud");

    while (!viewer.was_stopped()) {
      std::this_thread::sleep_for(100ms);
      viewer.spin_once();
    }
  }

private:
  jarvis::simple_visualizer<PointT> viewer;
  cloud_ptr cloud;
};

using std::clog;

int main(int argc, char *argv[]) {
  using pcl::console::print_error;
  if (argc < 2) {
    print_error("Wrong number of arguments.\n");
    clog << "Usage: " << argv[0] << " <input>\n";
    return 1;
  }

  pcl::PCLPointCloud2 cloud_meta;
  try {
    pcl::io::loadPCDFile(argv[1], cloud_meta);
  } catch (pcl::PCLException e) {
    print_error("Error reading file %s\n", argv[1]);
    return -1;
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
