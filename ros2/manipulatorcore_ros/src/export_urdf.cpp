// Prints the URDF of a ManipulatorCore built-in robot.
//
//   export_urdf [--robot NAME] [--output FILE] [--no-visuals]
//
// On success nothing is written to stderr, so the output can be used directly as the
// robot_description of robot_state_publisher (launch Command substitutions treat stderr output
// as a failure).

#include "manipulatorcore/model/urdf.hpp"
#include "manipulatorcore_ros/robots.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {

void printUsage(std::ostream& os) {
  os << "Usage: export_urdf [--robot NAME] [--output FILE] [--no-visuals]\n"
     << "\n"
     << "Writes the URDF of a ManipulatorCore built-in robot to stdout or FILE.\n"
     << "\n"
     << "Options:\n"
     << "  --robot NAME     one of: " << manipulatorcore_ros::joinNames(manipulatorcore_ros::builtinRobotNames())
     << " (default: redundant7r)\n"
     << "  --output FILE    write to FILE instead of stdout\n"
     << "  --no-visuals     omit <visual> elements\n"
     << "  --help           show this message\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string robot = "redundant7r";
  std::string output;
  mc::UrdfExportOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    const auto value = [&](std::string& target) {
      if (i + 1 >= argc) {
        std::cerr << "export_urdf: " << arg << " needs a value\n";
        return false;
      }
      target = argv[++i];
      return true;
    };
    if (arg == "--help" || arg == "-h") {
      printUsage(std::cout);
      return 0;
    }
    if (arg == "--robot") {
      if (!value(robot)) {
        return 2;
      }
    } else if (arg == "--output" || arg == "-o") {
      if (!value(output)) {
        return 2;
      }
    } else if (arg == "--no-visuals") {
      options.include_visuals = false;
    } else if (arg.starts_with("--ros-args")) {
      // Arguments appended by ros2 launch/run are not used by this tool.
      break;
    } else {
      std::cerr << "export_urdf: unknown argument '" << arg << "'\n";
      printUsage(std::cerr);
      return 2;
    }
  }

  try {
    const std::string urdf = mc::toURDF(manipulatorcore_ros::makeRobot(robot), options);
    if (output.empty()) {
      std::cout << urdf;
      std::cout.flush();
      return std::cout ? 0 : 1;
    }
    std::ofstream file(output);
    if (!file) {
      std::cerr << "export_urdf: cannot open '" << output << "' for writing\n";
      return 1;
    }
    file << urdf;
    return file ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "export_urdf: " << e.what() << "\n";
    return 1;
  }
}
