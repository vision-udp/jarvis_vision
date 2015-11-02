#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <regex>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

namespace {
class program_options {
public:
  program_options(const char *prog_name)
      : program_name(prog_name), visible_options("Allowed options") {

    visible_options.add_options()("help,h", "Produce help message");

    hidden_options.add_options()("input-directory", po::value<std::string>());
    options.add(visible_options).add(hidden_options);
    pos_options.add("input-directory", 1);
  }

  void print_usage(std::ostream &os) {
    os << "Usage: " << program_name << " <input-directory>\n\n";
    os << visible_options << '\n';
  }

  void parse_options(int argc, char *argv[], po::variables_map &varmap) {
    namespace st = po::command_line_style;

    const int parser_style =
        st::unix_style & ~st::allow_guessing & ~st::allow_sticky;

    po::command_line_parser parser(argc, argv);
    parser.options(options).positional(pos_options).style(parser_style);
    po::store(parser.run(), varmap);
  }

private:
  std::string program_name;
  po::options_description visible_options;
  po::options_description hidden_options;
  po::options_description options;
  po::positional_options_description pos_options;
};
} // end anonymous namespace

static bool check_directory(const fs::path &path) {
  if (path.empty() || !fs::exists(path) || !fs::is_directory(path))
    return false;
  return true;
}

static std::vector<fs::path> frames_from_path(const fs::path &directory_path) {
  std::vector<fs::path> framespaths;
  std::regex frame_regex("frame_[0-9]{1,6}.pcd");
  for (const auto &entry : fs::directory_iterator(directory_path)) {
    auto path = entry.path();
    if (fs::is_regular_file(entry.status()) &&
        std::regex_match(path.filename().c_str(), frame_regex))
      framespaths.push_back(path);
  }
  return framespaths;
}

struct recognition_result {
  std::size_t cylinders{};
  std::size_t boxes{};
  std::size_t spheres{};

  void operator+=(const recognition_result &other) {
    cylinders += other.cylinders;
    boxes += other.boxes;
    spheres += other.spheres;
  }
};

static recognition_result proccess_image(const fs::path &path) {
  //    2.1 - read image
  //    2.2 - filter image
  //    2.3 - segmentation
  //    2.5 - matching
  using point_t = pcl::PointXYZRGBA;
  recognition_result result;

  pcl::PointCloud<point_t>::Ptr cloud(new pcl::PointCloud<point_t>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  // read file and remove nan values
  pcl::io::loadPCDFile(path.c_str(), *cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // kd-tree object for searches.
  pcl::search::KdTree<point_t>::Ptr kdtree(new pcl::search::KdTree<point_t>);
  kdtree->setInputCloud(cloud);

  // Estimate the normals.
  pcl::NormalEstimation<point_t, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud);
  normalEstimation.setRadiusSearch(0.05);
  normalEstimation.setSearchMethod(kdtree);
  normalEstimation.compute(*normals);

  // Region growing clustering object.
  pcl::RegionGrowing<point_t, pcl::Normal> clustering;
  clustering.setMinClusterSize(900);
  clustering.setMaxClusterSize(10000000);
  clustering.setSearchMethod(kdtree);
  clustering.setNumberOfNeighbours(30);
  clustering.setInputCloud(cloud);
  clustering.setInputNormals(normals);
  // Set the angle in radians that will be the smoothness threshold
  // (the maximum allowable deviation of the normals).
  clustering.setSmoothnessThreshold(90.0 / 180.0 * M_PI); // 7 degrees.
  // Set the curvature threshold. The disparity between curvatures will be
  // tested after the normal deviation check has passed.
  clustering.setCurvatureThreshold(0.5);

  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);

  // For every cluster...
  int currentClusterNum = 1;
  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin();
       i != clusters.end(); ++i) {
    // ...add all its points to a new cloud...
    pcl::PointCloud<point_t>::Ptr cluster(new pcl::PointCloud<point_t>);
    for (std::vector<int>::const_iterator point = i->indices.begin();
         point != i->indices.end(); point++)
      cluster->points.push_back(cloud->points[*point]);
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    // ...and save it to disk.
    if (cluster->points.size() <= 0)
      break;
    std::cout << "Cluster " << currentClusterNum << " has "
              << cluster->points.size() << " points." << std::endl;
    std::string fileName =
        "cluster" + boost::to_string(currentClusterNum) + ".pcd";
    pcl::io::savePCDFileASCII(fileName, *cluster);

    currentClusterNum++;
  }

  return result;
}

int main(int argc, char *argv[]) {
  // 1 - get a list of file names that match frame_*
  program_options opts(argv[0]); // Initialized with program name
  po::variables_map vm;
  opts.parse_options(argc, argv, vm);

  if (vm.count("help")) {
    opts.print_usage(std::clog);
    return EXIT_SUCCESS;
  }

  if (!vm.count("input-directory")) {
    std::clog << "Error: Input directory is missing.\n";
    opts.print_usage(std::clog);
    return EXIT_FAILURE;
  }

  const auto &input_dir = vm["input-directory"].as<std::string>();
  const fs::path input_path(input_dir);

  if (!check_directory(input_path))
    return EXIT_FAILURE;

  auto framespaths = frames_from_path(input_path);

  if (framespaths.empty()) {
    std::clog << "Warning: Input directory contains no frames\n";
    return EXIT_SUCCESS;
  }

  // 2 - for each file get matching results
  recognition_result result;
  // for (const auto &framepath : framespaths)
  result += proccess_image(framespaths.front());

  // 3 - plot results
  std::clog << input_dir << " contains:\n";
  std::clog << " " << result.cylinders << " cylinders\n";
  std::clog << " " << result.spheres << " spheres\n";
  std::clog << " " << result.boxes << " boxes\n";
}
