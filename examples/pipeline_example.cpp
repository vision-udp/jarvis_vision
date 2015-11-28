//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/cloud_io.hpp>
#include <jarvis/cloud_pipeline.hpp>
#include <jarvis/steady_timer.hpp>
#include <jarvis/simple_visualizer.hpp>

#include <boost/make_shared.hpp> // for make_shared
#include <boost/filesystem.hpp>  // for path, recursive_iterator

#include <pcl/point_types.h> // for PointXYZ

#include <algorithm> // for for_each
#include <exception> // for exception
#include <iostream>  // for clog, ostream
#include <string>    // for string
#include <vector>    // for vector

using namespace jarvis;
using pcl::PointXYZ;
using std::clog;
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

int main(int argc, char *argv[]) {
  if (argc != 2) {
    clog << "Usage: " << argv[0] << " <input>\n";
    return 1;
  }

  try {
    const auto pcd_files = get_pcd_files(argv[1]);
    clog << pcd_files.size() << " PCD files to be processed.\n";
    const bool is_regular_file = fs::is_regular_file(argv[1]);
    simple_visualizer<pcl::PointXYZRGBA> viewer;
    viewer.set_full_screen(true);
    if (!is_regular_file)
      viewer.start();
    for (const auto &path : pcd_files) {
      const auto cloud = load_cloud<pcl::PointXYZ>(path.string());
      steady_timer timer;
      cloud_pipeline pipeline;
      timer.run("Processing frame");
      pipeline.process(cloud);
      const auto elapsed = timer.finish();
      clog << "Frame computations done!\n";
      const std::chrono::duration<double> elapsed_secs(elapsed);
      const double estimated_fps = 1.0 / elapsed_secs.count();
      clog << "Estimated frame rate: " << estimated_fps << " fps\n";
      if (is_regular_file) {
        clog << "Press enter to visualize . . ." << std::endl;
        std::cin.get();
        viewer.start();
      }
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
