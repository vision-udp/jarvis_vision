/// \file main.cpp
/// \brief Example of image projection into a point cloud.
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#include <jarvis/extract_image.hpp>
#include <jarvis/steady_timer.hpp>
#include <jarvis/lu_transform.hpp>

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

  cv::Mat hsv, hue;

  timer.run("convert to HSV");
  cv::cvtColor(image, hsv, CV_BGR2HSV);
  timer.finish();

  hue.create(hsv.size(), hsv.depth());
  int ch[] = {0, 0};
  cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

  cv::imshow("hue", hue);
  cv::waitKey();
}
