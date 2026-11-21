#!/usr/bin/env bash
# Builds ManipulatorCore and the optional ROS 2 package inside a ros:jazzy-ros-base container and runs
# the package tests. The repository is mounted read-only; all build products stay in the container.
#
#   scripts/ci/ros2_docker_build.sh            # build + colcon test
#   SMOKE=1 scripts/ci/ros2_docker_build.sh    # additionally launch the demo and send one goal
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="${ROS_IMAGE:-ros:jazzy-ros-base}"

docker run --rm -e SMOKE="${SMOKE:-0}" -v "${repo}:/src:ro" "${image}" bash -c '
  set -eo pipefail
  mkdir -p /tmp/repo
  tar -C /src --exclude=./build --exclude=./.venv --exclude=./_install --exclude=./.git -cf - . | tar -C /tmp/repo -xf -
  cd /tmp/repo
  cmake -S . -B /tmp/core-build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
        -DMC_BUILD_TESTS=OFF -DMC_BUILD_BENCHMARKS=OFF -DMC_BUILD_APPS=OFF
  cmake --build /tmp/core-build -j"$(nproc)"
  cmake --install /tmp/core-build --prefix /opt/manipulatorcore
  mkdir -p /tmp/ws/src
  cp -r /tmp/repo/ros2/manipulatorcore_ros /tmp/ws/src/
  source /opt/ros/jazzy/setup.bash
  cd /tmp/ws
  colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/manipulatorcore -DCMAKE_BUILD_TYPE=Release
  colcon test --event-handlers console_direct-
  colcon test-result --verbose
  if [ "${SMOKE}" = "1" ]; then
    source install/setup.bash
    ros2 launch manipulatorcore_ros demo.launch.py > /tmp/launch.log 2>&1 &
    launch_pid=$!
    sleep 8
    ros2 topic echo --once /joint_states sensor_msgs/msg/JointState > /tmp/js.txt
    echo "joint_states received: $(grep -c position /tmp/js.txt) message(s)"
    # Top-down grasp above the table of the tabletop scenario (tool z axis pointing down).
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: base_link}, pose: {position: {x: 0.52, y: -0.32, z: 0.32}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
    sleep 12
    kill "${launch_pid}" || true
    wait "${launch_pid}" 2>/dev/null || true
    echo "---- launch log (planner and simulator) ----"
    grep -Ei "plan|trajectory|goal|error|fail" /tmp/launch.log | head -n 40 || true
  fi
'
