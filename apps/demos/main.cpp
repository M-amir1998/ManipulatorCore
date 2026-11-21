// mc_demos: computes the data behind every animation and figure.
//
//   mc_demos list
//   mc_demos <output-directory> [demo ...]     (no names: run all demos)

#include "demos.hpp"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  std::vector<mc::demos::Demo> all;
  for (auto group :
       {mc::demos::kinematicsDemos(), mc::demos::planningDemos(), mc::demos::controlDemos(), mc::demos::heroDemos()}) {
    all.insert(all.end(), group.begin(), group.end());
  }
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <output-directory> [demo ...] | list\n", argv[0]);
    return 2;
  }
  const std::string first = argv[1];
  if (first == "list") {
    for (const auto& d : all) {
      std::printf("%-22s %s\n", d.name.c_str(), d.description.c_str());
    }
    return 0;
  }
  std::vector<std::string> selected(argv + 2, argv + argc);
  int failures = 0;
  for (const auto& d : all) {
    if (!selected.empty() && std::find(selected.begin(), selected.end(), d.name) == selected.end()) {
      continue;
    }
    std::printf("%s\n", d.name.c_str());
    try {
      d.run(first);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "  %s failed: %s\n", d.name.c_str(), e.what());
      ++failures;
    }
  }
  return failures == 0 ? 0 : 1;
}
