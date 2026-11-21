#!/usr/bin/env bash
# Linux verification in a container (default image: ros:jazzy-ros-base, Ubuntu 24.04 with GCC 13 and
# CMake 3.28): Debug build with warnings as errors + all tests, then AddressSanitizer + UBSan build +
# all tests. The repository is mounted read-only; build products stay in the container.
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${LINUX_IMAGE:-ros:jazzy-ros-base}"

docker run --rm -v "${repo}:/src:ro" "${image}" bash -c '
  set -eo pipefail
  apt-get update -qq && apt-get install -y -qq ninja-build > /dev/null
  mkdir -p /tmp/repo
  tar -C /src --exclude=./build --exclude=./.venv --exclude=./_install --exclude=./.git -cf - . | tar -C /tmp/repo -xf -
  cd /tmp/repo
  g++ --version | head -n 1
  cmake --version | head -n 1
  build_and_test() {
    local preset="$1"
    echo "==== ${preset} ===="
    cmake --preset "${preset}" > /tmp/configure.log 2>&1 || { tail -n 30 /tmp/configure.log; exit 1; }
    if ! cmake --build --preset "${preset}" -j "$(nproc)" > /tmp/build.log 2>&1; then
      grep -E "error|FAILED" /tmp/build.log | head -n 40
      exit 1
    fi
    echo "warnings: $(grep -c "warning:" /tmp/build.log || true)"
    if ! ctest --test-dir "build/${preset}" -j "$(nproc)" --output-on-failure > /tmp/ctest.log 2>&1; then
      grep -E "Failed|\*\*\*|error|runtime error|AddressSanitizer" /tmp/ctest.log | head -n 40
      tail -n 5 /tmp/ctest.log
      exit 1
    fi
    tail -n 3 /tmp/ctest.log
  }
  build_and_test debug
  ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 build_and_test asan-ubsan
'
