//          Copyright Diego Ramírez October 2015
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "recorder.hpp"

#include <atomic>    // for std::atomic
#include <cassert>   // for assert
#include <chrono>    // for std::chrono_literals
#include <csignal>   // for std::signal
#include <cstdlib>   // for EXIT_SUCESS, EXIT_FAILURE
#include <exception> // for std::exception
#include <iostream>  // for std::clog, std::cin
#include <string>    // for std::string

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;
using jarvis::pcd_recorder;

namespace {
class options_driver {
public:
  options_driver(const char *prog_name)
      : program_name(prog_name), visible_options("Allowed options") {

    visible_options.add_options()                          //
        ("help,h", "Produce help message")                 // op1
        ("list-devices", "List available OpenNi devices")  // op2
        ("list-point-types", "List available point types") // op3
        ("get-info", po::value<std::string>(),
         "Get information about an OpenNi device") // op4
        ("point-type,t", po::value<std::string>()->default_value("xyz"),
         "Point type to use") // op5
        ("take,n", po::value<std::size_t>()->default_value(0),
         "Number of frames to take (0 = infinity).")           // op6
        ("depth-registration,r", "Enable depth registration"); // op7

    hidden_options.add_options()("output-directory", po::value<std::string>());
    options.add(visible_options).add(hidden_options);
    pos_options.add("output-directory", 1);
  }

  void print_usage(std::ostream &os) {
    os << "Usage: " << program_name << " [options] <output-directory>\n\n";
    os << visible_options << '\n';
  }

  void parse_options(int argc, char *argv[], po::variables_map &varmap) {
    namespace st = po::command_line_style;

    const int parser_style =
        st::unix_style & ~st::allow_guessing & ~st::allow_sticky;

    po::command_line_parser parser(argc, argv);
    parser.options(options).positional(pos_options).style(parser_style);
    po::store(parser.run(), varmap);
  }

private:
  std::string program_name;
  po::options_description visible_options;
  po::options_description hidden_options;
  po::options_description options;
  po::positional_options_description pos_options;
};
} // end anonymous namespace

static std::string read_line() {
  std::string line;
  std::getline(std::cin, line);
  return line;
}

static void list_point_types() {
  std::clog << "The available point types are:\n";
  std::clog << "xyz:     for no color points.\n";
  std::clog << "xyzrgba: for colored points.\n";
}

static bool create_dirs(const fs::path &path) {
  if (path.empty())
    return false;
  if (fs::exists(path)) {
    if (!fs::is_directory(path)) {
      std::clog << "Error, file exists: " << path << '\n';
      return false;
    }
    std::clog << "Directory already exists. Some files could be overwritten "
                 "by the generated pcd files.\n";

    while (true) {
      std::clog << "Continue? [y, N] " << std::flush;
      const std::string ans = read_line();
      if (ans == "y" || ans == "Y") {
        std::clog << "Ok, the existing directory will be used." << std::endl;
        return true;
      }
      if (ans.empty() || ans == "n" || ans == "N") {
        std::clog << "Ok, program aborted." << std::endl;
        return false;
      }
      std::clog << "Invalid answer. Respond 'y' or 'n'.\n";
    }
  }
  // If path does not exist...
  fs::path root = fs::absolute(path.parent_path());
  while (!fs::exists(root)) {
    root = root.parent_path();
    std::clog << root << std::endl;
  }
  if (!fs::is_directory(root)) {
    std::clog << "Error: " << root << " is not a directory." << std::endl;
    return false;
  }

  const bool ret_val = fs::create_directories(path); // Should return true
  if (ret_val)
    std::clog << "Directory " << path << " has been created." << std::endl;
  return ret_val;
}

static void run_recorder(const pcd_recorder::param_type &params,
                         const fs::path &output_dir) {
  assert(fs::is_directory(output_dir));

  const auto prev_cd = fs::current_path();
  fs::current_path(output_dir);

  std::clog << "Creating recorder . . . " << std::endl;
  auto recorder = pcd_recorder::make(params);

  std::clog << "Starting recorder" << std::endl;
  recorder->start();
  std::clog << "The recorder is running!\n";

  static std::atomic<bool> signaled{false};
  std::signal(SIGINT, [](int) { signaled = true; });

  using namespace std::chrono_literals;
  while (!recorder->wait_for(500ms)) {
    if (!signaled)
      continue;
    std::clog << "\nStopping recorder . . . " << std::endl;
    recorder->stop();
    recorder->wait();
    break;
  }

  recorder.reset(); // Destroy recorder before logging.
  fs::current_path(prev_cd);
  std::clog << "All done!" << std::endl;
}

static int run_main_program(int argc, char *argv[]) {

  options_driver opts(argv[0]); // Initialized with program name
  po::variables_map vm;
  opts.parse_options(argc, argv, vm);
  po::notify(vm);

  if (vm.count("help")) {
    opts.print_usage(std::clog);
    return EXIT_SUCCESS;
  }

  if (vm.count("list-devices")) {
    jarvis::list_openni_devices();
    return EXIT_SUCCESS;
  }

  if (vm.count("list-point-types")) {
    list_point_types();
    return EXIT_SUCCESS;
  }

  if (vm.count("get-info")) {
    const auto &device_id = vm["get-info"].as<std::string>();
    jarvis::show_openni_device_info(device_id);
    return EXIT_SUCCESS;
  }

  if (!vm.count("output-directory")) {
    std::clog << "Error: Output directory is missing.\n";
    opts.print_usage(std::clog);
    return EXIT_FAILURE;
  }

  const fs::path output_dir = vm["output-directory"].as<std::string>();
  if (!create_dirs(output_dir))
    return EXIT_FAILURE; // Couldn't create directory

  pcd_recorder::param_type params;
  params.point_type = vm["point-type"].as<std::string>();
  params.frames_to_take = vm["take"].as<std::size_t>();
  params.depth_registration = vm.count("depth-registration");
  run_recorder(params, output_dir);
  return EXIT_SUCCESS;
}

int main(int argc, char *argv[]) {
  try {
    const int ret_val = run_main_program(argc, argv);
    if (ret_val == EXIT_SUCCESS)
      return 0; // EXIT_SUCESS is not required to be zero.
    return ret_val;
  } catch (std::exception &ex) {
    std::clog << "Error: " << ex.what() << '\n';
  } catch (...) {
    std::clog << "Unexpected error!\n";
  }
  return EXIT_FAILURE;
}
