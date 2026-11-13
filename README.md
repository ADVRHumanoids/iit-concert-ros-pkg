# iit-concert-ros-pkg

This tree is the canonical deployment source for the validated Concert Isaac +
XBot2 flow in this workspace.

The Isaac-side layout now follows the Kyon pattern:

- `concert_isaac`
  - `docker/iit-concert-ros-pkg-isaac`
  - `lib/src`
  - `python/src/concert_isaac`
- `concert_usd`

The top-level mirrors under `concert_robot/concert_isaac` and
`concert_robot/concert_config` are kept temporarily as legacy backups only.

ROS packages for the Concert robot simulation and demonstration.

## Overview

This metapackage contains the following subpackages:

- **concert_gazebo** - Gazebo simulation configurations
- **concert_mujoco** - MuJoCo simulation through xbot2_mujoco
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

### CartesI/O

```bash
ros2 launch concert_cartesio concert.launch.py xbot:=true
```

## Packages

### concert_gazebo

Gazebo simulation world files, model database, and launch files.
`config/` holds the XBot2 configuration used in simulation (`modular.yaml`,
HAL for Gazebo and dummy, GUI server configuration).

### concert_mujoco

MuJoCo simulation. The MJCF is generated at launch from the modular generator
(`concert_example.py`) by xbot2_mujoco, merged with `mjcf/concert_mujoco.xml.xacro`
(actuators, joint defaults, initial pose). `config/` holds the XBot2 configuration
for `--hw mj`.

```bash
ros2 launch concert_mujoco concert_mujoco.launch.py velodyne:=true realsense:=true ultrasound:=true
```

The VectorNav IMU (`imu`, default on), Velodyne VLP-16s, RealSense D435s and Bosch USS5
ultrasounds (5 x 5 ray lidars) publish on the topics of the Gazebo sensors.
Any `concert_examples` generator works (`modular_description:=`, or `pino.launch.py`):
actuators, joint classes and initial pose are derived from its URDF and SRDF
(`scripts/robot_mjcf.py`). `rviz:=true` opens `concert_gazebo`'s `concert_sensors.rviz`.

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
