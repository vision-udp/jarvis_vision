
#include <jarvis/extract_image.hpp>
#include <jarvis/lu_transform.hpp>
#include <jarvis/steady_timer.hpp>
#include <jarvis/plane_extraction.hpp>

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

using namespace jarvis;
using namespace std::chrono_literals;
namespace fs = boost::filesystem;

using boost::make_shared;
using boost::shared_ptr;
using pcl::PointXYZRGBA;
using std::clog;
using pcl::console::print_error;
using cloud_t = pcl::PointCloud<PointXYZRGBA>;
using cloud_ptr = boost::shared_ptr<cloud_t>;

#define window_size 8

template <typename T>
static std::vector<T> transform_image(const cv::Mat &image, size_t spacing) {
  cv::Mat image_gray(image.size(), CV_8UC1);
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  image_gray.convertTo(image_gray, CV_64FC1);
  return lu_transform<T, window_size>(
      image_gray, static_cast<ptrdiff_t>(window_size) / 2, spacing);
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
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

  if (!(cloud_meta.fields.size() > 3)) {
    print_error("Point cloud must contain rgb information.");
    return -1;
  }

  cloud_ptr cloud(new cloud_t);

  try {
    pcl::io::loadPCDFile<PointXYZRGBA>(argv[1], *cloud);
  } catch (pcl::PCLException e) {
    print_error("Error reading %s\n", argv[1]);
  }

  steady_timer timer;
  timer.run("Extract image");
  cv::Mat image = extract_image(*cloud);
  timer.finish();

  timer.run("Transform image");
  const size_t spacing = 8;
  auto params = transform_image<double>(image, spacing);
  timer.finish();

  std::vector<int> mapping;
  const auto fcloud = make_shared<pcl::PointCloud<PointXYZRGBA>>();
  pcl::removeNaNFromPointCloud(*cloud, *fcloud, mapping);

  plane_extractor<pcl::PointXYZRGBA> plane_extractor;
  plane_extractor.set_input_cloud(fcloud);
  plane_extractor.set_min_points(10000);
  plane_extractor.extract_planes();

  const auto search = make_shared<pcl::search::KdTree<PointXYZRGBA>>(false);
  const auto remaining = plane_extractor.get_remaining_indices();

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointXYZRGBA> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(10000);
  ec.setInputCloud(fcloud);
  ec.setSearchMethod(search);
  ec.setIndices(remaining);
  ec.extract(clusters);

  cv::Vec3b color;
  for (size_t i = 0; i < clusters.size(); ++i) {
    const std::vector<int> &indices = clusters[i].indices;

    if (indices.empty())
      continue;

    double sum = 0;
    for (const auto idx : indices)
      sum += params[2 * static_cast<size_t>(idx) / window_size];

    double mean = sum / indices.size();

    std::cout << "Cluster " << i << " mean: " << mean << '\n';
    if (mean < 0.005)
      color = cv::Vec3b(255, 0, 0);
    else if (mean > 1.0)
      color = cv::Vec3b(0, 0, 255);
    else
      color = cv::Vec3b(0, 255, 0);

    for (const auto idx : indices) {
      int real_idx = mapping[static_cast<size_t>(idx)];
      image.at<cv::Vec3b>(real_idx) = color;
    }
  }

  cv::imshow("Classification", image);
  cv::waitKey();
}
