#include <iostream>              // for clog
#include <boost/make_shared.hpp> // for make_shared
#include <pcl/io/pcd_io.h>       // for loadPCDFile
#include <pcl/point_types.h>
#include <jarvis/cloud_pipeline.hpp>
#include <jarvis/simple_visualizer.hpp>

using namespace jarvis;
using boost::make_shared;
using boost::shared_ptr;
using pcl::PointXYZ;
using std::clog;
using cloud_xyz = pcl::PointCloud<PointXYZ>;

template <typename PointT>
static void print_cloud(const pcl::PointCloud<PointT> &cloud,
                        const std::string &caption = "Cloud",
                        std::ostream &os = std::clog) {
  os << caption << '\n';
  os << "Number of points: " << cloud.size() << '\n';
  os << "  Width:  " << cloud.width << '\n';
  os << "  Height: " << cloud.height << " (organized=" << cloud.isOrganized()
     << ")\n";
  os << std::endl;
}

template <typename PointT>
static shared_ptr<cloud_xyz> read_cloud(const std::string &filename) {
  auto cloud = boost::make_shared<cloud_xyz>();

  if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    throw std::runtime_error("Cloud reading failed.");

  print_cloud(*cloud, "Cloud data:");
  return cloud;
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    clog << "Usage: " << argv[0] << " <input-pcd>\n";
    return 1;
  }

  try {
    const auto cloud = read_cloud<pcl::PointXYZ>(argv[1]);
    cloud_pipeline pipeline;
    pipeline.process(cloud);
    const auto colored_cloud = pipeline.get_colored_cloud();
    simple_visualizer<pcl::PointXYZRGBA> viewer;
    viewer.show_cloud(colored_cloud, "colored");
    viewer.loop();
    return 0;
  } catch (std::exception &ex) {
    clog << "Error: " << ex.what() << '\n';
  } catch (...) {
    clog << "Unexpected exception.\n";
  }
  return 1;
}
