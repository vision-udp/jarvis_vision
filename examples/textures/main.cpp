/// \file main.cpp
/// \brief Example of image projection into a point cloud.
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#include <jarvis/extract_image.hpp>
#include <jarvis/lu_transform.hpp>
#include <jarvis/steady_timer.hpp>

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>

using namespace jarvis;
using namespace std::chrono_literals;
using pcl::PointXYZRGBA;
using std::clog;
using pcl::console::print_error;
using cloud_t = pcl::PointCloud<PointXYZRGBA>;
using cloud_ptr = boost::shared_ptr<cloud_t>;

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

  cv::Mat result;

  timer.run("convert to grayscale");
  cv::cvtColor(image, result, CV_BGR2GRAY);
  timer.finish();

  cv::imshow("gray scale", result);

  timer.run("saturate cast to 64bits floating point type");
  result.convertTo(result, CV_64FC1);
  timer.finish();

  const size_t window_size = 8;
  const auto height = static_cast<size_t>(result.rows) / window_size;
  const auto width = static_cast<size_t>(result.cols) / window_size;
  timer.run("lu transform");
  auto params =
      lu_transform<double, 8>(result, static_cast<ptrdiff_t>(window_size) / 2);
  timer.finish();

  for (size_t i = 0; i < height; ++i)
    for (size_t j = 0; j < width; ++j) {
      int x = static_cast<int>(j * window_size);
      int y = static_cast<int>(i * window_size);
      result(cv::Rect(x, y, window_size, window_size)) = params[i * width + j];
    }
  result.convertTo(result, CV_8UC1);

  cv::imshow("LU-transform", result);
  cv::waitKey();
}
