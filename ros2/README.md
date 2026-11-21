# manipulatorcore_ros — optional ROS 2 adapter

A ROS 2 (Jazzy) package that exposes ManipulatorCore to ROS tooling. The core library stays
ROS-independent; this package consumes the **installed** library through
`find_package(ManipulatorCore CONFIG REQUIRED)`.

| Executable | Role |
| --- | --- |
| `export_urdf` | Prints the URDF of a built-in robot (`--robot redundant7r\|industrial6r\|planar2r\|planar3r`) for `robot_state_publisher` |
| `manipulatorcore_simulator` | Runs the ManipulatorCore simulator in real time with computed-torque control; holds its pose, executes received trajectories and publishes `joint_states` |
| `manipulatorcore_planner` | Plans to `goal_pose` with the pose-goal pipeline (multi-start IK, RRT-Connect, shortcutting, spline + time-optimal timing, verification) against a scenario's obstacles and publishes the result |

## Topics

| Topic | Type | Direction |
| --- | --- | --- |
| `goal_pose` | `geometry_msgs/msg/PoseStamped` (frame `base_link`) | planner input |
| `joint_states` | `sensor_msgs/msg/JointState` | simulator output, planner input |
| `joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | planner output, simulator input |
| `planning_scene_markers` | `visualization_msgs/msg/MarkerArray` (transient local) | obstacles and planned end-effector path |

## Parameters

| Node | Parameter | Default | Meaning |
| --- | --- | --- | --- |
| both | `robot` | `redundant7r` | built-in robot |
| simulator | `sim_dt` | `0.001` | simulator time step [s] |
| simulator | `publish_rate` | `100.0` | `joint_states` rate and pacing [Hz] |
| planner | `base_frame` | `base_link` | frame of goals and markers |
| planner | `planner` | `RRT-Connect` | `RRT`, `RRT-Connect`, `RRT*` or `PRM` |
| planner | `ik_attempts` | `32` | multi-start IK seeds per goal |
| planner | `max_goal_candidates` | `4` | IK solutions the planner tries to reach |
| planner | `max_iterations` | `20000` | planner sample budget per candidate |
| planner | `planning_time_limit` | `5.0` | wall-clock limit per candidate [s], 0 = none |
| planner | `clearance` | `0.02` | robot-obstacle clearance while planning [m] |
| planner | `trajectory_dt` | `0.01` | sampling interval of the published trajectory [s] |
| planner | `seed` | `1` | seed of IK restarts and planner sampling |

The launch file `demo.launch.py` accepts `robot`, `scenario` (default `tabletop_reach`, or `none`)
and `rviz` (default `false`; RViz is started only if it is installed).

## Build

```bash
# 1. Install the core library
cmake -S <repo> -B build-core -DCMAKE_BUILD_TYPE=Release -DMC_BUILD_TESTS=OFF -DMC_BUILD_BENCHMARKS=OFF -DMC_BUILD_APPS=OFF
cmake --build build-core -j
cmake --install build-core --prefix /opt/manipulatorcore

# 2. Build the package in a colcon workspace
source /opt/ros/jazzy/setup.bash
mkdir -p ws/src && cp -r <repo>/ros2/manipulatorcore_ros ws/src/
cd ws && colcon build --cmake-args -DCMAKE_PREFIX_PATH=/opt/manipulatorcore
colcon test && colcon test-result --verbose
```

`scripts/ci/ros2_docker_build.sh` performs both steps inside a `ros:jazzy-ros-base` container
(`SMOKE=1` additionally launches the demo and sends a goal).

## Run

```bash
source ws/install/setup.bash
ros2 launch manipulatorcore_ros demo.launch.py rviz:=true
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.52, y: -0.32, z: 0.32}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

## Verification status

Verified in the `ros:jazzy-ros-base` container (Ubuntu 24.04, GCC 13): the package builds, its 20
GoogleTest cases pass, and the launch smoke test plans to the goal above (about 0.04 s, goal error
below 0.1 mm), executes the trajectory in the simulator, and publishes `joint_states`. RViz was not
exercised (not part of the base image).
