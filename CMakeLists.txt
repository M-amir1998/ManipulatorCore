cmake_minimum_required(VERSION 3.21)

project(
  ManipulatorCore
  VERSION 0.1.0
  DESCRIPTION "Classical robot manipulation in modern C++20: geometry, kinematics, dynamics, planning, control and simulation"
  HOMEPAGE_URL "https://github.com/M-amir1998/ManipulatorCore"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------------------------
option(MC_BUILD_TESTS "Build the GoogleTest suites" ${PROJECT_IS_TOP_LEVEL})
option(MC_BUILD_BENCHMARKS "Build the Google Benchmark kernels" ${PROJECT_IS_TOP_LEVEL})
option(MC_BUILD_APPS "Build demos and metric benchmark applications" ${PROJECT_IS_TOP_LEVEL})
option(MC_BUILD_PYTHON "Build the optional pybind11 bindings" OFF)
option(MC_USE_SYSTEM_EIGEN "Use an installed Eigen3 (>= 3.4) when available" ON)
option(MC_WARNINGS_AS_ERRORS "Treat compiler warnings in project code as errors" OFF)
option(MC_ENABLE_ASAN "Build with AddressSanitizer" OFF)
option(MC_ENABLE_UBSAN "Build with UndefinedBehaviorSanitizer" OFF)
option(MC_ENABLE_CLANG_TIDY "Run clang-tidy while compiling project code" OFF)
option(MC_INSTALL "Generate install and package-config rules" ${PROJECT_IS_TOP_LEVEL})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(PROJECT_IS_TOP_LEVEL)
  # Collect every executable in one directory so sanitizer runtimes and demo data paths are simple.
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
endif()

include(GNUInstallDirs)
include(Sanitizers)
include(CompilerWarnings)
include(Dependencies)

if(MC_ENABLE_CLANG_TIDY)
  find_program(MC_CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(MC_CLANG_TIDY_COMMAND "${MC_CLANG_TIDY_EXE}" "--quiet")
endif()

add_subdirectory(src)

if(MC_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(MC_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

if(MC_BUILD_APPS)
  add_subdirectory(apps)
endif()

if(MC_BUILD_PYTHON)
  add_subdirectory(python/bindings)
endif()

if(MC_INSTALL)
  include(Install)
endif()
