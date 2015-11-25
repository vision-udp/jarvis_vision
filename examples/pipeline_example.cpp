#include <iostream>              // for clog
#include <boost/make_shared.hpp> // for make_shared
#include <pcl/io/pcd_io.h>       // for loadPCDFile
#include <pcl/point_types.h>
#include <jarvis/cloud_pipeline.hpp>
#include <jarvis/steady_timer.hpp>
#include <jarvis/simple_visualizer.hpp>

#include <vector>
#include <boost/filesystem.hpp>

using namespace jarvis;
using boost::make_shared;
using boost::shared_ptr;
using pcl::PointXYZ;
using std::clog;
using cloud_xyz = pcl::PointCloud<PointXYZ>;
namespace fs = boost::filesystem;

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

static std::vector<fs::path> get_pcd_files(const fs::path &input_path) {
  std::vector<fs::path> res;
  if (fs::is_regular_file(input_path))
    res.push_back(input_path);
  else {
    fs::recursive_directory_iterator iter{input_path};
    fs::recursive_directory_iterator end{};
    std::for_each(iter, end, [&](const fs::directory_entry &entry) {
      if (!fs::is_regular_file(entry.status()))
        return;
      const auto &path = entry.path();
      if (path.extension() == ".pcd")
        res.push_back(path);
    });
  }

  return res;
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

  const auto pcd_files = get_pcd_files(argv[1]);
  std::clog << pcd_files.size() << " PCD files to be processed.\n";
  // const bool prompt_before_view = pcd_files.size() == 1;

  try {
    simple_visualizer<pcl::PointXYZRGBA> viewer;
    viewer.set_full_screen(true);
    viewer.start();
    for (const auto &path : pcd_files) {
      const auto cloud = read_cloud<pcl::PointXYZ>(path.string());
      steady_timer timer;
      cloud_pipeline pipeline;
      timer.run("Processing frame");
      pipeline.process(cloud);
      const auto elapsed = timer.finish();
      clog << "Frame computations done!\n";
      const std::chrono::duration<double> elapsed_secs(elapsed);
      const double estimated_fps = 1.0 / elapsed_secs.count();
      clog << "Estimated frame rate: " << estimated_fps << " fps\n";
      // clog << "Press enter to visualize . . ." << std::endl;
      // std::cin.get();
      const auto colored_cloud = pipeline.get_colored_cloud();
      viewer.show_cloud(colored_cloud, "colored");
      viewer.spin_once();
    }
    viewer.spin();
    return 0;
  } catch (std::exception &ex) {
    clog << "Error: " << ex.what() << '\n';
  } catch (...) {
    clog << "Unexpected exception.\n";
  }
  return 1;
}
