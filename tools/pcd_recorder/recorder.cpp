//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "recorder.hpp"

#include <atomic>             // for std::atomic
#include <chrono>             // for std::chrono_literals
#include <condition_variable> // for std::condition_variable
#include <future>             // for std::async, std::future, std::promise
#include <iostream>           // for std::clog
#include <memory>             // for std::unique_ptr
#include <mutex>              // for std::mutex
#include <thread>             // for std::this_thread::sleep_for
#include <cassert>            // for assert
#include <cstdio>             // for std::sprintf

#include <pcl/console/print.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <tbb/pipeline.h>

using pcl::console::print_info;
using pcl::console::print_warn;
using jarvis::pcd_recorder;

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
  using promise_t = std::promise<cloud_ptr>;

public:
  openni_grabber(const bool enable_dr) {
    grabber = std::make_unique<pcl::OpenNIGrabber>();
    config_depth_registration(enable_dr);
    config_callback();
    grabber->start();
  }

  ~openni_grabber() {
    assert(grabber->isRunning());
    grabber->stop();
    // The sleep is a work around, since sometimes callbacks are called after
    // stopping or destroying the grabber.
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(300ms);
  }

  void grab(cloud_ptr &cloud) {
    assert(!output_cloud);
    assert(grabber->isRunning());
    std::unique_lock<std::mutex> lk(mtx);
    output_cloud = &cloud;
    cv.wait(lk, [&] { return output_cloud == nullptr; });
    assert(!output_cloud);
  }

private:
  void config_callback() {
    boost::function<void(const cloud_ptr &)> callback;
    callback = [&](const cloud_ptr &cloud) {
      std::unique_lock<std::mutex> lk(mtx);
      if (!output_cloud)
        return;
      *output_cloud = cloud;
      output_cloud = nullptr;
      lk.unlock();
      cv.notify_one();
    };
    grabber->registerCallback(callback);
  }

  void config_depth_registration(bool enable_dr) {
    if (!enable_dr)
      return;
    auto device = grabber->getDevice();
    if (device->isDepthRegistrationSupported()) {
      device->setDepthRegistration(true);
      assert(device->isDepthRegistered());
      std::clog << "Depth registration enabled.\n";
    } else {
      std::clog << "Error: Depth registration is not supported\n";
    }
  }

private:
  std::mutex mtx;
  std::condition_variable cv;
  cloud_ptr *output_cloud{nullptr};
  std::unique_ptr<pcl::OpenNIGrabber> grabber;
};
} // end anonymous namespace

// ============================================
// pcd_recorder definitions
// ============================================

namespace {
template <typename PointT>
class typed_pcd_recorder : public pcd_recorder {
public:
  using point_t = PointT;
  using cloud_ptr = typename pcl::PointCloud<point_t>::ConstPtr;

public:
  typed_pcd_recorder(const pcd_recorder::param_type &p)
      : params(p), grabber(p.depth_registration) {}

  ~typed_pcd_recorder() {
    stop();
    if (pipeline_execution.valid())
      pipeline_execution.get();
  }

  void start() override {
    assert(!pipeline_execution.valid());
    pipeline_execution =
        std::async(std::launch::async, &typed_pcd_recorder::run_pipeline, this);
  }

  void wait() override {
    assert(pipeline_execution.valid());
    pipeline_execution.wait();
  }

  bool wait_for(const std::chrono::milliseconds &time_ms) override {
    assert(pipeline_execution.valid());
    return pipeline_execution.wait_for(time_ms) == std::future_status::ready;
  }

  void stop() override { pipeline_stopped = true; }

private:
  void run_pipeline() {
    using tbb::filter;
    using tbb::make_filter;

    pipeline_stopped = false;

    bool take_infinite_frames = (params.frames_to_take == 0);
    std::size_t pending_frames = params.frames_to_take;
    auto producer = [&](tbb::flow_control &fc) -> cloud_ptr {
      auto stop_production = [&] {
        if (pipeline_stopped)
          return true;
        if (!take_infinite_frames && pending_frames == 0)
          return true;
        return false;
      };

      if (stop_production()) {
        fc.stop();
        return nullptr;
      }
      cloud_ptr cloud;
      grabber.grab(cloud);
      if (!take_infinite_frames) {
        assert(pending_frames > 0);
        --pending_frames;
      }
      return cloud;
    };

    size_t written_files = 0;
    pcl::PCDWriter writer;
    auto consumer = [&](const cloud_ptr &cloud) {
      char filename[128];
      std::sprintf(filename, "frame_%06zu.pcd", ++written_files);
      writer.writeBinaryCompressed(filename, *cloud);
      std::clog << "\rNumber of written PCD files: " << written_files;
    };

    const size_t max_number_of_live_tokens = 16;
    tbb::parallel_pipeline(
        max_number_of_live_tokens,
        make_filter<void, cloud_ptr>(filter::serial_in_order, producer) &
            make_filter<cloud_ptr, void>(filter::serial_in_order, consumer));

    std::clog << std::endl;
  }

private:
  std::future<void> pipeline_execution;
  std::atomic<bool> pipeline_stopped{false};
  pcd_recorder::param_type params;
  openni_grabber<point_t> grabber;
};
} // end anonymous namespace

// ============================================
// pcd_recorder definitions
// ============================================

pcd_recorder::~pcd_recorder() {}

std::unique_ptr<pcd_recorder> pcd_recorder::make(const param_type &p) {
  if (p.point_type == "xyz")
    return std::make_unique<typed_pcd_recorder<pcl::PointXYZ>>(p);
  if (p.point_type == "xyzrgba")
    return std::make_unique<typed_pcd_recorder<pcl::PointXYZRGBA>>(p);
  throw std::domain_error("Unknown point type: " + p.point_type);
}
