//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "recorder.hpp"
#include <boost/program_options.hpp>

#include <atomic>    // for std::atomic
#include <exception> // for std::exception
#include <iostream>  // for std::clog
#include <chrono>    // for std::chrono_literals
#include <thread>    // for std::sleep_for
#include <csignal>   // for std::signal

namespace po = boost::program_options;
using namespace std::chrono_literals;

static int main_program(int argc, char *argv[]) {
  po::options_description desc("Allowed options");
  desc.add_options()                                      //
      ("help,h", "Produce help message")                  // -h
      ("list-devices,l", "List available OpenNi devices") // -l
      ("get-info", po::value<std::string>(),
       "Get information about an OpenNi device") // --get-info
      ("point-type,t", po::value<std::string>()->default_value("xyz"),
       "Set point type of point cloud. Allowed values are:\n"
       "\t'xyz' for no color points\n"
       "\t'xyzrgba for colored points\n"); // --point-type

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.empty() || vm.count("help")) {
    std::clog << desc << '\n';
    return 1;
  }

  if (vm.count("list-devices")) {
    jarvis::list_openni_devices();
    return 0;
  }

  if (vm.count("get-info")) {
    const auto &device_id = vm["get-info"].as<std::string>();
    jarvis::show_openni_device_info(device_id);
    return 0;
  }

  const auto &point_type = vm["point-type"].as<std::string>();
  std::clog << "Selected point type: " << point_type << '\n';

  std::clog << "Creating recorder . . . " << std::endl;
  const auto recorder = jarvis::make_pcd_recorder(point_type);

  std::clog << "Starting recorder" << std::endl;
  recorder->start();

  // static std::atomic<bool> signaled{false};
  //  std::signal(SIGINT, [](int) { signaled = true; });
  //  while (!signaled)
  //    std::this_thread::sleep_for(500ms);
  //
  //  std::clog << "Termination signal received! Wait for resource cleaning :)"
  //            << std::endl;
  //
  //  std::signal(SIGINT, [](int) {
  //    std::clog << "Wait while the program frees acquired resources!"
  //              << std::endl;
  //  });

  std::clog << "The recorder is running!\n";
  std::clog << "Press ENTER to stop it." << std::endl;
  std::cin.get();

  std::clog << "Stopping recorder" << std::endl;
  recorder->stop();
  std::clog << "All done!" << std::endl;
  return 0;
}

int main(int argc, char *argv[]) {

  try {
    return main_program(argc, argv);
  } catch (std::exception &ex) {
    std::clog << "Exception: " << ex.what() << '\n';
  } catch (...) {
    std::clog << "Unexpected exception!\n";
  }
  return -1;
}
