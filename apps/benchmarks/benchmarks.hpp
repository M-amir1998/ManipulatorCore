#pragma once

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace mc::bench {

/// Each metric benchmark returns a Markdown section (tables plus a short protocol description).
std::string runIkBenchmark(int targets);
std::string runPlanningBenchmark(int seeds);
std::string runControlBenchmark();

inline double secondsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

inline std::string fmt(const char* format, double value) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

}  // namespace mc::bench
