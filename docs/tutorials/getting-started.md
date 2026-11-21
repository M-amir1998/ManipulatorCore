# Getting started

## Requirements

* A C++20 compiler: GCC ≥ 12, Clang ≥ 15 or MSVC 2022 (19.3x) and newer.
* CMake ≥ 3.21 and, ideally, Ninja.
* Network access on the first configure (Eigen, GoogleTest and Google Benchmark are fetched unless
  installed).
* Optional: Python ≥ 3.11 with numpy, matplotlib and Pillow for the figures.

## Build and test

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Tests carry labels, so subsets can be run with `ctest --test-dir build -L unit` (also `numerical`,
`integration`, `smoke`). Presets are available for common configurations:

```bash
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
cmake --preset asan-ubsan      # GCC/Clang AddressSanitizer + UBSan (Linux)
```

On Windows, open a *Developer PowerShell for VS* (or run `scripts/setup/vsdevenv.ps1`, which locates
Visual Studio with `vswhere`) so that `cl`, `cmake` and `ninja` are on the `PATH`. The `msvc-asan` and
`clang-cl-ubsan` presets enable sanitizers with the Visual Studio toolchains.

## A first program

```cpp
#include "manipulatorcore/kinematics/inverse_kinematics.hpp"
#include "manipulatorcore/model/builtin_robots.hpp"

#include <cstdio>

int main() {
  using namespace mc;
  const RobotModel arm = makeRedundant7R();
  const SE3 goal = endEffectorPose(arm, redundant7RHome()) * SE3::fromTranslation(Vec3(0.05, 0.0, 0.05));
  const IKResult ik = solveIK(arm, goal, redundant7RHome());
  std::printf("%s after %d iterations\n", toString(ik.status), ik.iterations);
}
```

The in-tree examples `mc_example_basic_kinematics` and `mc_example_plan_and_execute` are built with
the project (`apps/examples`).

## Using the installed library

```bash
cmake --install build --prefix /opt/manipulatorcore
cmake -S examples/external_project -B build-consumer -DCMAKE_PREFIX_PATH=/opt/manipulatorcore
cmake --build build-consumer
```

```cmake
find_package(ManipulatorCore 0.1 CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE ManipulatorCore::planning ManipulatorCore::control)
```

## Python

```bash
python -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
pytest
```

Configure with `-DMC_BUILD_PYTHON=ON` (or `--preset python`) to build the optional `manipulatorcore`
extension module into `build/<preset>/python`; add that directory to `PYTHONPATH`.
