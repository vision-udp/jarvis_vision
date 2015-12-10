/// \file main.cpp
/// \brief Example of usage of texture descriptor LU-transform.
/// \author Jorge Aguirre
/// \version 1.0
/// \date 2015-11-28

#include <jarvis/extract_image.hpp>
#include <jarvis/lu_transform.hpp>
#include <jarvis/steady_timer.hpp>

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

using namespace jarvis;
using namespace std::chrono_literals;
namespace fs = boost::filesystem;

using pcl::PointXYZRGBA;
using std::clog;
using pcl::console::print_error;
using cloud_t = pcl::PointCloud<PointXYZRGBA>;
using cloud_ptr = boost::shared_ptr<cloud_t>;

static void bgr2gray(const cv::Mat &src, cv::Mat &dst) {
  for (int i = 0; i < src.rows; ++i) {
    for (int j = 0; j < src.cols; ++j) {
      auto bgr = src.at<cv::Vec3b>(i, j);
      dst.at<uchar>(i, j) =
          static_cast<uchar>(0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2]);
    }
  }
}

static std::vector<fs::path> images_paths(const fs::path &input_path) {
  if (fs::is_regular_file(input_path))
    return std::vector<fs::path>({input_path});

  std::vector<fs::path> frames_paths;
  for (const auto &entry : fs::directory_iterator(input_path)) {
    const auto &path = entry.path();
    if (fs::is_regular_file(entry.status()) &&
        (path.extension() == ".png" || path.extension() == ".jpg"))
      frames_paths.push_back(path);
  }

  return frames_paths;
}

#define window_size 8

static void transform_image(const fs::path &input_path, size_t spacing,
                            bool show) {
  cv::Mat image = cv::imread(input_path.c_str());
  cv::Mat image_gray(image.size(), CV_8UC1);
  steady_timer timer;

  clog << "Image path: " << input_path << '\n';
  clog << "dimensions: " << image.size() << '\n';

  timer.run("GrayScale");
  bgr2gray(image, image_gray);
  timer.finish();
  cv::imshow("GrauScale", image_gray);
  image_gray.convertTo(image_gray, CV_64FC1);

  auto params = lu_transform<double, window_size>(
      image_gray, static_cast<ptrdiff_t>(window_size) / 2, spacing);

  int rows = image_gray.rows;
  int cols = image_gray.cols;

  if (spacing) {
    rows /= static_cast<int>(spacing);
    cols /= static_cast<int>(spacing);
  }

  cv::Mat result(rows, cols, image_gray.type(), params.data());
  cv::Scalar mean;
  cv::Scalar std_dev;
  cv::meanStdDev(result, mean, std_dev);

  std::printf("mean: %f\nstd dev: %f\nratio: %f\n", mean.val[0], std_dev.val[0],
              std_dev.val[0] / mean.val[0]);

  if (show) {
    auto result_size = image.size();
    result_size.width /= 2;
    result_size.height /= 2;
    cv::resize(result, result, result_size);
    result.convertTo(result, CV_8UC1);
    cv::imshow("LU-transform", result);
    cv::waitKey();
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    clog << "Usage: " << argv[0] << " <input>\n";
    return 1;
  }

  auto input_paths = images_paths(argv[1]);

  const size_t spacing = 8;

  for (auto &path : input_paths)
    transform_image(path, spacing, true);
}
