//          Copyright Diego Ramírez November 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <jarvis/pcl_fwd.hpp>

#include <atomic>
#include <csignal>
#include <iostream>
#include <jarvis/cloud_pipeline.hpp>
#include <jarvis/openni_grabber.hpp>
#include <jarvis/simple_visualizer.hpp>
#include <jarvis/steady_timer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace jarvis;

int main() {
  using point_t = pcl::PointXYZRGBA;
  using pcl::PointCloud;
  using std::chrono::duration;

  steady_timer timer;
  openni_grabber<point_t> grabber(true);
  boost::shared_ptr<const PointCloud<point_t>> cloud;
  pipeline_searcher<point_t> searcher(true);
  searcher.start();
  size_t num_processed_frames = 0;
  static std::atomic<bool> signaled{false};
  std::signal(SIGINT, [](int) { signaled = true; });

  timer.run("Processing frames");
  while (!signaled) {
    grabber.grab(cloud);
    searcher.update_cloud(cloud);
    ++num_processed_frames;
    searcher.spin_once();
  }
  auto elapsed = timer.finish();
  const double elapsed_seconds = duration<double>(elapsed).count();
  const double avg_fps = num_processed_frames / elapsed_seconds;
  std::clog << "Num processed frames: " << num_processed_frames << '\n';
  std::clog << "Elapsed time: " << elapsed_seconds << " seconds" << '\n';
  std::clog << "Average frame rate: " << avg_fps << " fps" << '\n';
}
