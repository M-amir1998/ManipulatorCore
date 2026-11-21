#pragma once

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace mc::demos {

/// A demo computes data with the library and writes <output>/<name>.json for the renderer.
struct Demo {
  std::string name;
  std::string description;
  std::function<void(const std::filesystem::path& output)> run;
};

std::vector<Demo> kinematicsDemos();
std::vector<Demo> planningDemos();
std::vector<Demo> controlDemos();
std::vector<Demo> heroDemos();

}  // namespace mc::demos
