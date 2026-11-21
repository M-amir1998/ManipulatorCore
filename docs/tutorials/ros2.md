# ROS 2 integration

The core library does not depend on ROS. The optional package
[`ros2/manipulatorcore_ros`](../../ros2) connects it to ROS 2 Jazzy: a real-time simulator node, a
planning node and URDF export for `robot_state_publisher` and RViz.

```text
            goal_pose (PoseStamped)
                   │
                   ▼
   ┌──────────────────────────────┐   joint_trajectory   ┌──────────────────────────────┐
   │  manipulatorcore_planner     │ ───────────────────▶ │  manipulatorcore_simulator    │
   │  IK + RRT-Connect + timing   │                      │  computed torque + simulator  │
   └──────────────────────────────┘ ◀─────────────────── └──────────────────────────────┘
          planning_scene_markers          joint_states             │
                                                                   ▼
                                                         robot_state_publisher → TF → RViz
```

## 1. Install the core library

```bash
cmake -S ManipulatorCore -B build-core -DCMAKE_BUILD_TYPE=Release \
      -DMC_BUILD_TESTS=OFF -DMC_BUILD_BENCHMARKS=OFF -DMC_BUILD_APPS=OFF
cmake --build build-core -j
sudo cmake --install build-core --prefix /opt/manipulatorcore
```

## 2. Build the package

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/mc_ws/src && cp -r ManipulatorCore/ros2/manipulatorcore_ros ~/mc_ws/src/
cd ~/mc_ws
colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/manipulatorcore
colcon test && colcon test-result --verbose
```

No ROS installation at hand? `scripts/ci/ros2_docker_build.sh` does everything in a
`ros:jazzy-ros-base` container.

## 3. Launch and send a goal

```bash
source ~/mc_ws/install/setup.bash
ros2 launch manipulatorcore_ros demo.launch.py scenario:=tabletop_reach rviz:=true
```

In a second terminal, ask for a top-down grasp (tool z axis pointing down, i.e. a rotation of π
about x):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.52, y: -0.32, z: 0.32}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
ros2 topic echo --once /joint_states
```

The planner logs the IK candidates, the path length and the trajectory duration; the simulator logs
the execution and the final joint error. Unreachable or colliding goals are rejected with a message;
the nodes never crash on planning failure.

## Notes

* Goals must be expressed in `base_link`; the planner rejects other frames instead of guessing a
  transform.
* The simulated robot is the ManipulatorCore model, not a hardware driver. To command a real arm,
  publish the planner's `joint_trajectory` to your controller (for example a `ros2_control` joint
  trajectory controller) and feed its `joint_states` back to the planner.
* See [ros2/README.md](../../ros2/README.md) for all parameters.
