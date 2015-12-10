
#include <jarvis/extract_image.hpp>
#include <jarvis/steady_timer.hpp>
#include <jarvis/plane_extraction.hpp>

#include <algorithm>
#include <cmath>
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

static cv::Vec3f bgr2hsv(const cv::Vec3f &bgr) {
  cv::Vec3f hsv(0, 0, 0);
  float min, max;

  min = bgr[2] < bgr[1] ? (bgr[2] < bgr[0] ? bgr[2] : bgr[0])
                        : (bgr[1] < bgr[0] ? bgr[1] : bgr[0]);
  max = bgr[2] > bgr[1] ? (bgr[2] > bgr[0] ? bgr[2] : bgr[0])
                        : (bgr[1] > bgr[0] ? bgr[1] : bgr[0]);

  hsv[2] = max;
  if (std::abs(max) < std::numeric_limits<float>::epsilon())
    return hsv;
  hsv[1] = 1 - min / max;
  if (std::abs(hsv[1]) < std::numeric_limits<float>::epsilon())
    return hsv;

  if (std::abs(max - bgr[2]) < std::numeric_limits<float>::epsilon()) {
    if (bgr[1] - bgr[0] < std::numeric_limits<float>::epsilon())
      hsv[0] = 260 + 60 * (bgr[1] - bgr[0]) / (max - min);
    else
      hsv[0] = 0 + 60 * (bgr[1] - bgr[0]) / (max - min);
  } else if (std::abs(max - bgr[1]) < std::numeric_limits<float>::epsilon())
    hsv[0] = 120 + 60 * (bgr[0] - bgr[2]) / (max - min);
  else
    hsv[0] = 240 + 60 * (bgr[2] - bgr[1]) / (max - min);

  return hsv;
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
  timer.run("extract image");
  cv::Mat image = extract_image(*cloud);
  timer.finish();

  std::vector<int> mapping;
  const auto fcloud = make_shared<pcl::PointCloud<PointXYZRGBA>>();
  pcl::removeNaNFromPointCloud(*cloud, *fcloud, mapping);

  plane_extractor<pcl::PointXYZRGBA> plane_extractor;
  plane_extractor.set_input_cloud(fcloud);
  plane_extractor.set_min_points(30000);
  plane_extractor.extract_planes();

  std::cout << "planes extracted: " << plane_extractor.get_num_planes() << '\n';

  const auto search = make_shared<pcl::search::KdTree<PointXYZRGBA>>(false);
  const auto remaining = plane_extractor.get_remaining_indices();

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointXYZRGBA> ec;
  ec.setClusterTolerance(0.02);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(30000);
  ec.setInputCloud(fcloud);
  ec.setSearchMethod(search);
  ec.setIndices(remaining);
  ec.extract(clusters);

  cv::Vec3f color;
  for (size_t i = 0; i < clusters.size(); ++i) {
    const std::vector<int> &indices = clusters[i].indices;
    if (indices.empty())
      continue;

    for (const auto idx : indices) {
      int real_idx = mapping[static_cast<size_t>(idx)];
      color += image.at<cv::Vec3b>(real_idx);
    }

    color[0] /= indices.size();
    color[1] /= indices.size();
    color[2] /= indices.size();

    auto hsv = bgr2hsv(color);
    std::cout << hsv << '\n';

    if (hsv[2] < 10) {
      color = cv::Vec3f(0, 0, 0);
    } else if (hsv[2] < 0.7) {
      color = cv::Vec3f(254, 254, 254);
    }
    for (const auto idx : indices) {
      int real_idx = mapping[static_cast<size_t>(idx)];
      image.at<cv::Vec3b>(real_idx) = color;
    }
  }

  cv::imshow("Color Classification", image);
  cv::waitKey();
}
