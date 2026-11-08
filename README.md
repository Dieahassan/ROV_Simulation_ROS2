# ROV ROS 2 Project

ROS 2 workspace packages for ROV control, simulation, and state estimation.

## Repository layout

```text
.
├── my_rov_control/       # Keyboard/teleoperation and control nodes
├── my_rov_estimation/    # AprilTag, SLAM bridge, and estimation nodes
└── my_rov_sim/           # Stonefish scenarios, robot description, and launch files
```

## Requirements

- Ubuntu Linux with a ROS 2 distribution supported by the installed Stonefish release.
- `colcon` and `rosdep`.
- Stonefish C++ simulation library.
- **`stonefish_ros2`** (external ROS 2 dependency; it is intentionally not vendored in this repository).
- ROS packages referenced by the launch files, including `xacro`, `robot_state_publisher`,
  `robot_localization`, `joy`, `rviz2`, and the relevant message packages.
- Python dependencies used by the nodes, such as NumPy, OpenCV, `cv_bridge`, and
  `pupil-apriltags`, as applicable to the selected nodes.

> Use mutually compatible versions of Stonefish and `stonefish_ros2`. The bundled
> Stonefish ROS 2 copy was removed from this repository; install it separately from
> its upstream project and retain its original GPL-3.0 license and notices.

## Install Stonefish and stonefish_ros2

Follow the upstream installation instructions for the Stonefish library:
- https://github.com/patrykcieslak/stonefish

Then add the ROS 2 interface package to the same ROS 2 workspace:

```bash
cd ~/rov_ws/src
git clone https://github.com/patrykcieslak/stonefish_ros2.git
```

Build/install Stonefish and `stonefish_ros2` according to their upstream instructions
before building this project. Confirm that the Stonefish library version matches the
`stonefish_ros2` version you install.

## Build this project

Place this repository's three ROS packages in `~/rov_ws/src/`:

```text
~/rov_ws/
└── src/
    ├── my_rov_control/
    ├── my_rov_estimation/
    ├── my_rov_sim/
    └── stonefish_ros2/       # cloned separately from upstream
```

Then:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/rov_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the simulation with:

```bash
ros2 launch my_rov_sim my_rov_sim_teleop.launch.py
```

The launch file starts the Stonefish simulator and the configured ROS 2 nodes.
For keyboard teleoperation in a separate terminal (if not already started by the
launch file):

```bash
ros2 run my_rov_control my_rov_keybead_teleop
```

The simulation scenarios and topic names are project-specific. Review the launch
files and scenario XML before running them with a different robot configuration.

## Packages

| Package | Purpose |
|---|---|
| `my_rov_control` | ROV control and teleoperation |
| `my_rov_estimation` | AprilTag perception and state-estimation utilities |
| `my_rov_sim` | Stonefish simulation world, robot descriptions, and launch files |

## Third-party components and licensing

`stonefish_ros2` is an external dependency maintained by its upstream author:
[patrykcieslak/stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2).
It is not included in this repository. Its upstream project is distributed under
GPL-3.0; follow its license and attribution requirements when installing or using it.

This repository currently does not impose a single top-level license on all files.
Before publishing it publicly, verify the origin and license of every model, texture,
scenario, and source file, then add a root `LICENSE` only when you have the right to
license the complete project. Do not replace third-party copyright or license notices.

## Notes

- Generated build/install/log folders, Python bytecode, caches, and machine-specific
  build artifacts are excluded.
- the bundled `stonefish_ros2` source package were removed
  from this cleaned archive.
