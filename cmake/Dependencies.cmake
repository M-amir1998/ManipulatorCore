# Third-party dependencies.
#
#   Eigen 3.4          required, header-only. An installed Eigen is preferred. Otherwise the pinned
#                      release tarball is downloaded (headers only, Eigen's own CMake is not run)
#                      and installed alongside ManipulatorCore.
#   GoogleTest 1.15    tests only (MC_BUILD_TESTS)
#   Google Benchmark   benchmarks only (MC_BUILD_BENCHMARKS)
#   pybind11           optional Python bindings only (MC_BUILD_PYTHON)

include(FetchContent)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(MC_EIGEN_VERSION 3.4.0)
set(MC_EIGEN_URL "https://gitlab.com/libeigen/eigen/-/archive/${MC_EIGEN_VERSION}/eigen-${MC_EIGEN_VERSION}.tar.gz")
set(MC_EIGEN_SHA256 8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72)

set(MC_EIGEN_FROM_SYSTEM OFF)
if(MC_USE_SYSTEM_EIGEN)
  find_package(Eigen3 3.4 CONFIG QUIET)
  if(TARGET Eigen3::Eigen)
    set(MC_EIGEN_FROM_SYSTEM ON)
    message(STATUS "ManipulatorCore: using installed Eigen ${Eigen3_VERSION}")
  endif()
endif()

if(NOT MC_EIGEN_FROM_SYSTEM)
  message(STATUS "ManipulatorCore: fetching Eigen ${MC_EIGEN_VERSION} (headers only)")
  FetchContent_Declare(
    mc_eigen_src
    URL "${MC_EIGEN_URL}"
    URL_HASH SHA256=${MC_EIGEN_SHA256}
    # Point at a directory without a CMakeLists.txt so Eigen's own build system is not configured.
    SOURCE_SUBDIR "headers-only")
  FetchContent_MakeAvailable(mc_eigen_src)
  set(MC_EIGEN_INCLUDE_DIR "${mc_eigen_src_SOURCE_DIR}")
endif()

add_library(mc_eigen INTERFACE)
add_library(ManipulatorCore::eigen ALIAS mc_eigen)
set_target_properties(mc_eigen PROPERTIES EXPORT_NAME eigen)
# Only the MPL2-licensed parts of Eigen are used.
target_compile_definitions(mc_eigen INTERFACE EIGEN_MPL2_ONLY)
if(MC_EIGEN_FROM_SYSTEM)
  target_link_libraries(mc_eigen INTERFACE Eigen3::Eigen)
else()
  target_include_directories(
    mc_eigen SYSTEM
    INTERFACE "$<BUILD_INTERFACE:${MC_EIGEN_INCLUDE_DIR}>"
              "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/manipulatorcore/third_party/eigen3>")
endif()

function(mc_fetch_googletest)
  if(TARGET GTest::gtest_main)
    return()
  endif()
  find_package(GTest 1.14 CONFIG QUIET)
  if(TARGET GTest::gtest_main)
    return()
  endif()
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    URL_HASH SHA256=7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926)
  FetchContent_MakeAvailable(googletest)
endfunction()

function(mc_fetch_google_benchmark)
  if(TARGET benchmark::benchmark)
    return()
  endif()
  find_package(benchmark 1.8 CONFIG QUIET)
  if(TARGET benchmark::benchmark)
    return()
  endif()
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_WERROR OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googlebenchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.9.1.tar.gz
    URL_HASH SHA256=32131c08ee31eeff2c8968d7e874f3cb648034377dfc32a4c377fa8796d84981)
  FetchContent_MakeAvailable(googlebenchmark)
endfunction()

function(mc_fetch_pybind11)
  if(TARGET pybind11::module)
    return()
  endif()
  find_package(pybind11 2.11 CONFIG QUIET)
  if(TARGET pybind11::module)
    return()
  endif()
  FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
    URL_HASH SHA256=e08cb87f4773da97fa7b5f035de8763abc656d87d5773e62f6da0587d1f0ec20)
  FetchContent_MakeAvailable(pybind11)
endfunction()
