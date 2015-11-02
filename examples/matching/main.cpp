#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <regex>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

namespace {
class program_options {
public:
  program_options(const char *prog_name)
      : program_name(prog_name), visible_options("Allowed options") {

    visible_options.add_options()("help,h", "Produce help message");

    hidden_options.add_options()("input-directory", po::value<std::string>());
    options.add(visible_options).add(hidden_options);
    pos_options.add("input-directory", 1);
  }

  void print_usage(std::ostream &os) {
    os << "Usage: " << program_name << " <input-directory>\n\n";
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

static bool check_directory(const fs::path &path) {
  if (path.empty() || !fs::exists(path) || !fs::is_directory(path))
    return false;
  return true;
}

static std::vector<fs::path> frames_from_path(const fs::path &directory_path) {
  std::vector<fs::path> framespaths;
  std::regex frame_regex("frame_[0-9]{1,6}.pcd");
  for (const auto &entry : fs::directory_iterator(directory_path)) {
    auto path = entry.path();
    if (fs::is_regular_file(entry.status()) &&
        std::regex_match(path.filename().c_str(), frame_regex))
      framespaths.push_back(path);
  }
  return framespaths;
}

int main(int argc, char *argv[]) {
  // 1 - get a list of file names that match frame_*
  program_options opts(argv[0]); // Initialized with program name
  po::variables_map vm;
  opts.parse_options(argc, argv, vm);

  if (vm.count("help")) {
    opts.print_usage(std::clog);
    return EXIT_SUCCESS;
  }

  if (!vm.count("input-directory")) {
    std::clog << "Error: Input directory is missing.\n";
    opts.print_usage(std::clog);
    return EXIT_FAILURE;
  }

  const auto &input_dir = vm["input-directory"].as<std::string>();
  const fs::path input_path(input_dir);

  if (!check_directory(input_path))
    return EXIT_FAILURE;

  auto framespaths = frames_from_path(input_path);

  if (framespaths.empty()) {
    std::clog << "Warning: Input directory contains no frames\n";
    return EXIT_SUCCESS;
  }

  // 2 - for each file get matching results
  //    2.1 - read image
  //    2.2 - filter image
  //    2.3 - segmentation
  //    2.4 - descriptors
  //    2.5 - matching
  // 3 - plot results
}
