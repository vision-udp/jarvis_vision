//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "recorder.hpp"

#include <atomic>   // for std::atomic
#include <iostream> // for std::clog
#include <future>   // for std::async, std::future
#include <cassert>  // for assert
#include <cstdio>   // for std::sprintf

#include <pcl/console/print.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <tbb/concurrent_queue.h>
#include <tbb/pipeline.h>

using pcl::console::print_info;
using pcl::console::print_warn;
using jarvis::recorder;

// ============================================
// Openni Devices information functions
// ============================================

void jarvis::list_openni_devices() {
  auto &driver = openni_wrapper::OpenNIDriver::getInstance();

  const unsigned n_devices = driver.getNumberDevices();
  if (n_devices == 0) {
    print_warn("Warning: No connected devices.\n");
    return;
  }
  print_info("Found Devices: %d\n", n_devices);
  for (unsigned i = 0; i < n_devices; ++i) {
    print_info("[%u] Device id: \'#%u\', vendor: %s, product: %s, connected: "
               "%#02x@%#02x, serial "
               "number: \'%s\'\n",
               i, i + 1, driver.getVendorName(i), driver.getProductName(i),
               driver.getBus(i), driver.getAddress(i),
               driver.getSerialNumber(i));
  }
}

void jarvis::show_openni_device_info(const std::string &device_id) {
  pcl::OpenNIGrabber grabber(device_id);
  auto device = grabber.getDevice();
  print_info("Device: %s, %s\n", device->getVendorName(),
             device->getProductName());
  print_info("Supported depth modes: \n");

  for (const auto &mode : grabber.getAvailableDepthModes())
    print_info("%d : %dx%d @ %dfps\n", mode.first, mode.second.nYRes,
               mode.second.nXRes, mode.second.nFPS);

  if (!device->hasImageStream())
    return;

  print_info("Supported image modes: \n");

  for (const auto &mode : grabber.getAvailableImageModes())
    print_info("%d = %dx%d @ %dfps\n", mode.first, mode.second.nYRes,
               mode.second.nXRes, mode.second.nFPS);
}

// ============================================
// openni_grabber definitions
// ============================================

namespace {
template <typename PointT>
class openni_grabber {
public:
  using point_t = PointT;
  using cloud_ptr = typename pcl::PointCloud<point_t>::ConstPtr;

public:
  openni_grabber() {
    queue.set_capacity(5);
    grabber = std::make_unique<pcl::OpenNIGrabber>();
    boost::function<void(const cloud_ptr &)> callback;
    callback = [&](const cloud_ptr &cloud) { queue.try_push(cloud); };
    grabber->registerCallback(callback);
  }

  ~openni_grabber() { stop(); }

  bool grab(cloud_ptr &cloud) {
    if (stopped)
      return false;
    try {
      queue.pop(cloud);
      return true;
    } catch (tbb::user_abort &) {
      assert(stopped.load(std::memory_order_relaxed));
      return false;
    }
  }

  void stop() {
    if (stopped)
      return;
    stopped = true;
    grabber->stop();
    queue.abort();
    queue.clear();
  }

private:
  tbb::concurrent_bounded_queue<cloud_ptr> queue;
  std::unique_ptr<pcl::Grabber> grabber;
  std::atomic<bool> stopped{false};
};
} // end anonymous namespace

// ============================================
// pcd_recorder definitions
// ============================================

namespace {
template <typename PointT>
class pcd_recorder : public recorder {
public:
  using point_t = PointT;
  using cloud_ptr = typename pcl::PointCloud<point_t>::ConstPtr;

public:
  pcd_recorder(const std::size_t frames_to_take) {
    pending_frames = frames_to_take;
    take_infinite_frames = (frames_to_take == 0);
  }

  ~pcd_recorder() { stop(); }

  void start() override {
    pipeline_execution =
        std::async(std::launch::async, &pcd_recorder::run_pipeline, this);
  }

  bool wait_for(const std::chrono::milliseconds &time_ms) override {
    return pipeline_execution.wait_for(time_ms) == std::future_status::ready;
  }

  void stop() override {
    if (!pipeline_execution.valid())
      return;
    cloud_grabber.stop();
    pipeline_execution.get();
  }

private:
  void run_pipeline() {
    using tbb::filter;
    using tbb::make_filter;

    auto consumer = [this](tbb::flow_control &fc) -> cloud_ptr {
      if (!take_infinite_frames && pending_frames == 0) {
        fc.stop();
        return nullptr;
      }
      cloud_ptr cloud;
      if (!cloud_grabber.grab(cloud)) {
        fc.stop();
        return nullptr;
      }
      if (!take_infinite_frames) {
        assert(pending_frames > 0);
        --pending_frames;
      }
      return cloud;
    };

    size_t written_files = 0;
    auto producer = [this, &written_files](const cloud_ptr &cloud) {
      char filename[128];
      std::sprintf(filename, "frame_%06zu.pcd", ++written_files);
      cloud_writer.writeBinaryCompressed(filename, *cloud);
      std::clog << "\rNumber of written PCD files: " << written_files;
    };

    const size_t max_number_of_live_tokens = 16;
    tbb::parallel_pipeline(
        max_number_of_live_tokens,
        make_filter<void, cloud_ptr>(filter::serial_in_order, consumer) &
            make_filter<cloud_ptr, void>(filter::serial_in_order, producer));

    std::clog << std::endl;
  }

private:
  openni_grabber<point_t> cloud_grabber;
  pcl::PCDWriter cloud_writer;
  std::future<void> pipeline_execution;
  std::size_t pending_frames = 0;
  bool take_infinite_frames = true;
};
} // end anonymous namespace

// ============================================
// recorder definitions and related
// ============================================

recorder::~recorder() {}

class pcd_recorder_builder {
public:
  pcd_recorder_builder(const std::size_t frames_to_take_)
      : frames_to_take{frames_to_take_} {}

  template <typename PointT>
  std::unique_ptr<recorder> build_recorder() {
    return std::make_unique<pcd_recorder<PointT>>(frames_to_take);
  }

  std::unique_ptr<recorder> build_recorder(const std::string &point_type) {
    if (point_type == "xyz")
      return build_recorder<pcl::PointXYZ>();
    if (point_type == "xyzrgba")
      return build_recorder<pcl::PointXYZRGBA>();
    throw std::domain_error("Unknown point type: " + point_type);
  }

private:
  std::size_t frames_to_take;
};

std::unique_ptr<recorder>
jarvis::make_pcd_recorder(const std::string &point_type,
                          const std::size_t frames_to_take) {
  pcd_recorder_builder builder(frames_to_take);
  return builder.build_recorder(point_type);
}
