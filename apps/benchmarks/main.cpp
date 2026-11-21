// mc_metric_benchmarks: algorithm-quality benchmarks (success rates, errors, path quality, tracking).
//
//   mc_metric_benchmarks <output.md> [ik] [planning] [control]    (no names: all)

#include "benchmarks.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <output.md> [ik] [planning] [control]\n", argv[0]);
    return 2;
  }
  const std::vector<std::string> selected(argv + 2, argv + argc);
  const auto wanted = [&](const std::string& name) {
    return selected.empty() || std::find(selected.begin(), selected.end(), name) != selected.end();
  };
  std::string md;
  try {
    if (wanted("ik")) {
      md += mc::bench::runIkBenchmark(300);
    }
    if (wanted("planning")) {
      md += mc::bench::runPlanningBenchmark(5);
    }
    if (wanted("control")) {
      md += mc::bench::runControlBenchmark();
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "benchmark failed: %s\n", e.what());
    return 1;
  }
  std::ofstream out(argv[1]);
  out << md;
  std::printf("%s", md.c_str());
  return 0;
}
