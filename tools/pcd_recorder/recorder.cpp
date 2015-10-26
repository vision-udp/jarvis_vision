/// \file pcl_recorder.cpp
/// \brief Tool to write to disk frames grabbed by a OpenNI compatible device.
///
/// Based on:
/// https://github.com/PointCloudLibrary/pcl/blob/master/io/tools/openni_pcd_recorder.cpp
/// Some more options can be added.
///
/// \author Jorge Aguirre
/// \version 0.1
/// \date 2015-10-21

#include "recorder.hpp"

#include <csignal>
#include <ctime>
#include <mutex>
#include <thread>
#include <boost/circular_buffer.hpp>
#include <pcl/console/print.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

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
// pcd_buffer definitions
// ============================================

template <typename PointT>
class pcd_buffer : boost::noncopyable {
public:
  using point_t = PointT;
  using cloud_ptr = typename pcl::PointCloud<point_t>::ConstPtr;

public:
  pcd_buffer(std::size_t buffer_size, std::atomic_bool &is_done)
      : buffer(buffer_size), done(is_done) {}

  bool push_back(cloud_ptr cloud) {
    bool not_full = false;
    {
      std::lock_guard<std::mutex> lock(buff_mutex);
      not_full = !buffer.full();
      buffer.push_back(cloud);
    }
    buff_empty.notify_one();
    return not_full;
  }

  const cloud_ptr front() {
    using namespace std::literals;
    cloud_ptr cloud;
    std::unique_lock<std::mutex> lock(buff_mutex);

    while (buffer.empty() && !done)
      buff_empty.wait_for(lock, 100ms, [&] { return !buffer.empty() || done; });

    if (buffer.empty()) {
      lock.unlock();
      return nullptr;
    }

    cloud = buffer.front();
    buffer.pop_front();
    lock.unlock();

    return cloud;
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(buff_mutex);
    return buffer.empty();
  }

  bool full() {
    std::lock_guard<std::mutex> lock(buff_mutex);
    return buffer.full();
  }

  std::size_t size() {
    std::lock_guard<std::mutex> lock(buff_mutex);
    buffer.size();
  }

  std::size_t capacity() const { return buffer.capacity(); }

  void capacity(std::size_t buff_size) {
    std::lock_guard<std::mutex> lock(buff_mutex);
    buffer.set_capacity(buff_size);
  }

private:
  std::mutex buff_mutex;
  boost::circular_buffer<cloud_ptr> buffer;
  std::condition_variable buff_empty;
  std::atomic_bool &done;
};

// ============================================
// pcd_recorder definitions
// ============================================

namespace {
template <typename PointT>
class pcd_recorder : public recorder {
public:
  using point_t = PointT;
  using cloud_ptr = typename pcl::PointCloud<point_t>::ConstPtr;

private:
  class frames_producer {
  public:
    frames_producer(pcd_buffer<PointT> &buff, std::atomic_bool &is_done)
        : buffer(buff),
          worker(std::bind(&frames_producer::grab_and_send, this)),
          done(is_done) {}

    void stop() { worker.join(); }

  private:
    void grab_and_send() {
      using namespace std::literals;
      auto grabber = std::make_unique<pcl::OpenNIGrabber>();
      boost::function<void(const cloud_ptr &)> callback;
      callback = [&](const cloud_ptr &cloud) { buffer.push_back(cloud); };
      grabber->registerCallback(callback);

      while (!done)
        std::this_thread::sleep_for(1s);

      grabber->stop();
    }

  private:
    pcd_buffer<PointT> &buffer;
    std::thread worker;
    std::atomic_bool &done;
  };

  class frames_consumer {
  public:
    frames_consumer(pcd_buffer<point_t> &buff, std::atomic_bool &is_done)
        : buffer(buff),
          worker(std::bind(&frames_consumer::receive_and_process, this)),
          done(is_done) {}

    void stop() { worker.join(); }

  private:
    void receive_and_process() {
      while (!done) {
        write_to_disk(buffer.front());
      }
      while (!buffer.empty())
        write_to_disk(buffer.front());
    }

    void write_to_disk(const cloud_ptr &cloud) {
      if (!cloud)
        return;

      std::time_t t = std::time(nullptr);
      std::stringstream ss;
      ss << "frame_" << std::put_time(std::localtime(&t), "%H%M%S") << ".pcd";
      writer.writeBinaryCompressed(ss.str(), *cloud);
    }

  private:
    pcd_buffer<point_t> &buffer;
    std::thread worker;
    pcl::PCDWriter writer;
    std::atomic_bool &done;
  };

public:
  pcd_recorder(std::size_t buffer_size = 200)
      : done{false}, buffer(buffer_size, done), producer(buffer, done),
        consumer(buffer, done) {}

  void stop() {
    done = true;
    producer.stop();
    consumer.stop();
  }

private:
  std::atomic_bool done;
  pcd_buffer<point_t> buffer;
  frames_producer producer;
  frames_consumer consumer;
};
} // end anonymous namespace

// ============================================
// recorder definitions and related
// ============================================

recorder::~recorder() {}

void recorder::start() {}

std::unique_ptr<recorder>
jarvis::make_pcd_recorder(const std::string &point_type) {
  using pcl::PointXYZ;
  using pcl::PointXYZRGBA;
  if (point_type == "xyz")
    return std::make_unique<pcd_recorder<PointXYZ>>();
  if (point_type == "xyzrgba")
    return std::make_unique<pcd_recorder<PointXYZRGBA>>();
  throw std::domain_error("Unknown point type: " + point_type);
}
