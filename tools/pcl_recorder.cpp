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

#include <csignal>
#include <ctime>
#include <mutex>
#include <thread>

#include <boost/circular_buffer.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

static void print_usage(const char *program_name) {
  using pcl::console::print_info;

  print_info("Usage: %s (<device-id> | <path-to-oni-file>) [options]\n",
             program_name);
  print_info("Options:\n");
  print_info("-l,--list : list all available devices\n");
  print_info("-l,--list <device-id> : list all available modes for specified "
             "device\n");
  print_info("-h, --help  : show this help message\n");
}

template <typename... Args>
bool find_argument(int argc, char *argv[], const char *argument, Args... args) {
  using pcl::console::find_switch;

  auto find_argument = [](int _argc, char *_argv[], const char *_argument) {
    return find_switch(_argc, _argv, _argument);
  };

  return find_switch(argc, argv, argument) ||
         find_argument(argc, argv, args...);
}

template <typename... Args>
int find_argument_pos(int argc, char *argv[], const char *argument,
                      Args... args) {
  using pcl::console::find_argument;

  auto find_argument_pos = [](int _argc, char *_argv[], const char *_argument) {
    return find_argument(_argc, _argv, _argument);
  };

  auto position = find_argument(argc, argv, argument);
  return position > 0 ? position : find_argument_pos(argc, argv, args...);
}

static void list_devices() {
  using pcl::console::print_info;
  using pcl::console::print_warn;

  auto &driver = openni_wrapper::OpenNIDriver::getInstance();

  auto n_devices = driver.getNumberDevices();
  if (n_devices == 0) {
    print_warn("Warning: No connected devices.\n");
    return;
  }
  print_info("Devices found: %d\n", n_devices);
  for (unsigned i = 0; i < n_devices; ++i) {
    print_info("[%d] Device id: \'#%d\', vendor: %s, product: %s, connected: "
               "%#02x@%#02x, serial "
               "number: \'%s\'\n",
               i, i + 1, driver.getVendorName(i), driver.getProductName(i),
               driver.getBus(i), driver.getAddress(i),
               driver.getSerialNumber(i));
  }
}

static void list_device_modes(const char *device_id) {
  using pcl::console::print_info;

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

#include <iostream>
#include <cstdlib>

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

template <typename PointT>
class pcd_recorder {
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

static std::atomic_bool stop_recording(false);

int main(int argc, char *argv[]) {
  using pcl::console::print_error;
  using pcl::console::parse;
  using pcl::console::print_highlight;
  using namespace std::literals;

  if (argc < 2) {
    print_error("Error: wrong number of arguments\n");
    print_usage(*argv);
    return -1;
  }

  if (find_argument(argc, argv, "-h", "--help")) {
    print_usage(*argv);
    return 1;
  }

  if (find_argument(argc, argv, "-l", "--list")) {
    if (argc >= 3) {
      if (find_argument_pos(argc, argv, "-l", "--list") != 1) {
        print_error("Error: wrong arguments order\n");
        return -1;
      }
      list_device_modes(argv[2]);
    } else
      list_devices();
    return 0;
  }

  pcd_recorder<pcl::PointXYZ> recorder;
  std::signal(SIGINT, [](int) { stop_recording = true; });
  while (!stop_recording)
    std::this_thread::sleep_for(500ms);

  pcl::console::print_highlight("Exit condition set to true\n");
  recorder.stop();
}
