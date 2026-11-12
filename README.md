# iit-concert-ros-pkg

This tree is the canonical deployment source for the validated Concert Isaac +
XBot2 flow in this workspace.

The Isaac-side layout now follows the Kyon pattern:

- `concert_isaac`
  - `docker/iit-concert-ros-pkg-isaac`
  - `lib/src`
  - `python/src/concert_isaac`
- `concert_urdf`
- `concert_srdf`
- `concert_usd`
- `concert_config`

The top-level mirrors under `concert_robot/concert_isaac` and
`concert_robot/concert_config` are kept temporarily as legacy backups only.

ROS packages for the Concert robot simulation and demonstration.

## Overview

This metapackage contains the following subpackages:

- **concert_urdf** - URDF/Xacro robot description
- **concert_srdf** - SRDF semantic robot description
- **concert_gazebo** - Gazebo simulation configurations
- **concert_config** - Simulation-specific XBot2 configuration
- **concert_cartesio** - CartesI/O configuration
- **concert_examples** - Modular robot descriptions and examples
- **concert_odometry_ros2** - Odometry (base estimation) configuration
- **concert_tests** - Test scripts

## Installation

Clone into your ROS 2 workspace and build with colcon:

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/ADVRHumanoids/iit-concert-ros-pkg.git
cd ..
colcon build
```

## Usage

### Launch Simulation

```bash
ros2 launch concert_gazebo concert.launch.py
```

### XBot2 with Simulation

```bash
source ~/ros2_ws/src/iit-concert-ros-pkg/concert_config/setup.sh
xbot2-core --hw sim
```

### CartesI/O

```bash
ros2 launch concert_cartesio concert.launch.py xbot:=true
```

## Packages

### concert_urdf

Contains the URDF/Xacro files describing the Concert robot's kinematics and visual/collision geometry.

### concert_srdf

Contains the SRDF files for MoveIt and XBot2 semantic description (joint groups, end-effectors, etc.).

### concert_gazebo

Gazebo simulation world files, model database, and launch files.

### concert_config

Simulation-specific XBot2 configuration files including:
- HAL configuration for Gazebo
- Joint configurations
- GUI and launcher configurations

### concert_cartesio

CartesI/O inverse kinematics configuration for Cartesian control.

### concert_tests

Test scripts, e.g. `ros2 run concert_tests repeatability.py`.

## Dependencies

- ROS 2 Jazzy
- Gazebo
- XBot2
- CartesI/O

## License

BSD
