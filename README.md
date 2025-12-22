# iit-concert-ros-pkg

ROS packages for the Concert robot simulation and demonstration.

## Overview

This metapackage contains the following subpackages:

- **concert_urdf** - URDF/Xacro robot description
- **concert_srdf** - SRDF semantic robot description
- **concert_gazebo** - Gazebo simulation configurations
- **concert_config** - Simulation-specific XBot2 configuration
- **concert_cartesio_config** - CartesI/O configuration
- **concert_demo** - Demo applications and examples

## Installation

Clone into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ADVRHumanoids/iit-concert-ros-pkg.git
cd ..
catkin build
```

## Usage

### Launch Simulation

```bash
roslaunch concert_gazebo concert_gazebo.launch
```

### XBot2 with Simulation

```bash
source ~/catkin_ws/src/iit-concert-ros-pkg/concert_config/setup.sh
xbot2-core --hw sim
```

### CartesI/O

```bash
roslaunch concert_cartesio_config concert_cartesio.launch
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

### concert_cartesio_config

CartesI/O inverse kinematics configuration for Cartesian control.

### concert_demo

Demo applications showcasing robot capabilities.

## Dependencies

- ROS Noetic / ROS2 Humble
- Gazebo
- XBot2
- CartesI/O

## License

BSD
