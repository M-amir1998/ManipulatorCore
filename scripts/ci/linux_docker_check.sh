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
  echo "==== Debug build (GCC, -Werror) ===="
  cmake --preset debug > /dev/null
  cmake --build --preset debug -j "$(nproc)" 2>&1 | grep -E "warning|error|FAILED" || true
  ctest --test-dir build/debug -j "$(nproc)" --output-on-failure 2>&1 | tail -n 4
  echo "==== AddressSanitizer + UBSan (GCC) ===="
  cmake --preset asan-ubsan > /dev/null
  cmake --build --preset asan-ubsan -j "$(nproc)" 2>&1 | grep -E "warning|error|FAILED" || true
  ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
    ctest --test-dir build/asan-ubsan -j "$(nproc)" --output-on-failure 2>&1 | tail -n 4
'
