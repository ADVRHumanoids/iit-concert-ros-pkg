# Concert Robot Configuration

> **Canonical deployment package**: for the validated Isaac + XBot2 flow in
> this workspace, this in-tree `concert_config` package and the sibling
> `concert_robot/iit-concert-ros-pkg/concert_isaac` package are the source of
> truth. The top-level copies under `concert_robot/concert_config` and
> `concert_robot/concert_isaac` are legacy backups only.

This repository contains the comprehensive configuration files and management tools for the Concert modular robot system. It provides a complete framework for controlling and monitoring the robot through a web-based interface and managing its real-time control processes.

> **Note**: This repository (`concert_config`) is for the **real robot** configuration. For **simulation**, see [iit-concert-ros-pkg](https://github.com/ADVRHumanoids/iit-concert-ros-pkg) which contains Gazebo simulation, URDF, and simulation-specific configurations.

## Overall System Architecture

The Concert robot software stack consists of several interconnected components that work together to provide a complete robot control solution:

1. **`xbot2-core`**:
   - **Role**: The central real-time robot control software.
   - **Implementation**: C++ library/middleware.
   - **Execution**: Runs directly on the robot's hardware (or in simulation), typically within a Docker container.
   - **Functionality**:
     - Interfaces with motors and sensors via a Hardware Abstraction Layer (HAL).
     - Executes real-time control algorithms (loaded as plugins).
     - Communicates with higher-level systems via ROS2.
     - Configured by `xbot2/ModularBot.yaml` and related HAL/plugin config files.

2. **Concert Launcher (`executor.py`)**: 
   - **Role**: The execution engine responsible for launching and managing software processes.
   - **Implementation**: Python library (`concert_launcher`).
   - **Functionality**:
     - Reads process definitions from `concert_launcher_config.yaml`.
     - Starts, stops, and monitors processes.
     - Manages processes locally, remotely via SSH, and inside Docker containers.
     - Utilizes `tmux` for reliable background process management.
     - Provides status checking and output streaming capabilities.
   - **Documentation**: For detailed documentation, see the [Concert Launcher README](https://github.com/ADVRHumanoids/concert_launcher).

3. **`xbot2_gui_server`**: 
   - **Role**: The backend web server providing a Graphical User Interface for managing the robot software.
   - **Implementation**: Python web server using `aiohttp`.
   - **Functionality**:
     - Acts as a bridge between the web-based GUI and the Concert Launcher.
     - Uses the Concert Launcher to manage the processes defined in `concert_launcher_config.yaml`.
     - Provides an HTTP API and WebSocket interface for the web frontend.
     - Allows users to view process status, start/stop processes, select configuration variants, and view live console output.

---

## Repository Structure

```
concert_config/                          # Real robot configuration
├── docker/                              # Docker build and runtime configurations
│   ├── build-concert.bash              # Robot-specific build wrapper script
│   ├── bashrc-setup-host.sh            # Host environment setup script
│   ├── concert-config_ros1.env         # ROS1 environment configuration
│   ├── concert-config_ros2.env         # ROS2 environment configuration
│   ├── concert-focal-ros1/             # ROS1 runtime compose configurations
│   │   └── compose.yml
│   └── concert-noble-ros2/             # ROS2 runtime compose configurations
│       ├── compose.yml
│       └── setup.sh                    # Container environment setup
├── docker-base/                         # Submodule: generalized xbot2_docker
│   └── robot-template/                  # Generic robot build templates
├── gui/
│   ├── ros1/
│   │   ├── gui_server_config.yaml      # GUI server configuration (ROS1)
│   │   └── concert_launcher_config.yaml # Process definitions (ROS1)
│   └── ros2/
│       ├── gui_server_config.yaml      # GUI server configuration (ROS2)
│       └── concert_launcher_config.yaml # Process definitions (ROS2)
├── xbot2/
│   ├── ModularBot.yaml                  # Main configuration for xbot2-core
│   ├── ModularBot_impedance_config.yaml # CartesIO impedance configuration
│   └── ...                              # Additional xbot2 configurations
├── hal/                                 # Hardware Abstraction Layer configurations
│   ├── ModularBot_gz.yaml              # Gazebo simulation HAL
│   ├── ModularBot_dummy.yaml           # Dummy HAL for testing
│   └── ModularBot_ec_all.yaml          # EtherCAT HAL configuration
├── ecat/                                # EtherCAT configurations
│   └── ecat_config.yaml                # EtherCAT master configuration
├── joint_config/                        # Joint control configurations
├── launch/                              # ROS2 launch files
├── setup.sh                             # Main setup script
├── CMakeLists.txt                       # ROS package build configuration
└── package.xml                          # ROS package manifest
```

---

## Quick Start Guide

### Prerequisites

- Docker and Docker Compose installed
- Git with access to required repositories
- X11 server (for GUI forwarding)

### Step 1: Clone the Repository

```bash
git clone --recursive <repository-url> concert_config
cd concert_config
```

If you already cloned without `--recursive`:
```bash
git submodule update --init --recursive
```

### Step 2: Configure Host Environment

Run the host setup script to configure your `~/.bashrc`:

```bash
cd docker
./bashrc-setup-host.sh --ros2   # For ROS2 (Ubuntu Noble)
# OR
./bashrc-setup-host.sh --ros1   # For ROS1 (Ubuntu Focal)
```

Then apply the changes:
```bash
source ~/.bashrc
```

### Step 3: Build Docker Images

```bash
cd docker

# Build ROS2 images (default)
./build-concert.bash

# Or explicitly specify version
./build-concert.bash ros2    # For ROS2
./build-concert.bash ros1    # For ROS1

# Additional options
./build-concert.bash --push  # Build and push to registry
./build-concert.bash --pull  # Pull pre-built images
```

### Step 4: Start the Development Container

```bash
cd docker/concert-noble-ros2

# Source the setup script (configures environment and aliases)
source ./setup.sh

# Start the development container using the alias
ros2 dev
```

The `setup.sh` script sets up:
- X11 forwarding permissions (`xhost +local:root`)
- Docker Compose aliases (`ros2 dev`, `ros2 sim`, etc.)
- Environment variables for ROS2

### Step 5: Run the Simulation

Inside the container:
```bash
# Launch Gazebo with the Concert robot
ros2 launch concert_gazebo modular.launch.py
```

---

## Simulation Configuration (iit-concert-ros-pkg)

The simulation-specific configurations are in the **[iit-concert-ros-pkg](https://github.com/ADVRHumanoids/iit-concert-ros-pkg)** repository. Unlike the real robot config, simulation runs **locally** without SSH or remote machine definitions.

### Container Environment Setup (`bashrc-setup-container.sh`)

Inside the Docker container, you need to configure the environment for simulation by running:

```bash
# Inside the container, navigate to the simulation config
cd ~/xbot2_ws/src/iit-concert-ros-pkg/concert_config

# Run the setup script (adds config to ~/.bashrc)
./bashrc-setup-container.sh

# Apply the changes
source ~/.bashrc
```

This script adds the simulation `setup.sh` to your container's `~/.bashrc`, which configures:
- `XBOT2_DEFAULT_HW=sim` - Default hardware mode for simulation
- `CONCERT_LAUNCHER_DEFAULT_CONFIG` - Points to simulation launcher config
- `set_xbot2_config` - Configures xbot2 to use `modular.yaml`

### Simulation setup.sh (`iit-concert-ros-pkg/concert_config/setup.sh`)

```bash
export XBOT2_DEFAULT_HW=sim
export CONCERT_LAUNCHER_DEFAULT_CONFIG="$SCRIPT_DIR/gui/ros2/concert_launcher_config.yaml"
set_xbot2_config "$SCRIPT_DIR/modular.yaml"
```

### Simulation Launcher Config (`iit-concert-ros-pkg/concert_config/gui/ros2/concert_launcher_config.yaml`)

```yaml
context:
  session: concert_sim    # tmux session name
  params:
    hw_type: sim          # Default to simulation HAL

xbot2_sim:
  cmd: xbot2-core --hw sim
  ready_check: timeout 5 ros2 topic echo --once /xbotcore/joint_states
```

**Key Differences from Real Robot Config**:
- **No `machine` field**: All processes run locally (no SSH)
- **No `.defines` with IP addresses**: Not needed for local execution
- **`hw_type: sim`**: Uses Gazebo simulation HAL instead of EtherCAT
- **Simpler structure**: Only defines processes that make sense in simulation

### Simulation GUI Server Config (`iit-concert-ros-pkg/concert_config/gui/ros2/gui_server_config.yaml`)

```yaml
launcher:
  launcher_config: concert_launcher_config.yaml

cartesian:
  cmd_vel_topics:
    - /omnisteering/cmd_vel
```

---

## Real Robot Configuration (concert_config)

The real robot configuration defines how processes run across **multiple machines** via SSH and inside Docker containers.

### 1. `gui/ros2/gui_server_config.yaml`

This file tells the `xbot2_gui_server` where to find the main process configuration:

```yaml
launcher:
  launcher_config: concert_launcher_config.yaml  # Path to launcher config

cartesian:
  cmd_vel_topics:
    - /omnisteering/cmd_vel  # Topic for velocity commands
```

### 2. `gui/ros2/concert_launcher_config.yaml`

This is the core configuration file for the Concert Launcher on the **real robot**. It defines what software components run on which machines:

```yaml
context:
  session: concert_real  # tmux session name
  params:
    hw_type: ec_idle     # Default hardware type
  .defines:              # YAML Anchors for reusable aliases
    - &embedded embedded@10.24.10.100    # Robot embedded PC
    - &control concert@10.24.10.102      # Control station
    - &vision concert@10.24.10.101       # Vision computer
    - &docker-xeno concert-noble-ros2-xeno-dev-1  # Xenomai Docker container
    - &docker-dev concert-noble-ros2-dev-1        # Dev Docker container

# Process Definitions
ecat:
  cmd: ecat_master
  machine: *embedded           # Runs on embedded PC via SSH
  docker: *docker-xeno         # Inside Xenomai-enabled container

xbot2:
  cmd: xbot2-core --hw {hw_type}
  machine: *embedded
  ready_check: timeout 5 ros2 topic echo --once /xbotcore/joint_states
  docker: *docker-xeno
  variants:
    verbose:
      cmd: "{cmd} -V"
    ctrl:
      - ec_idle:
          params: { hw_type: ec_idle }
      - ec_pos:
          params: { hw_type: ec_pos }
      - ec_imp:
          params: { hw_type: ec_imp }
      - dummy:
          params: { hw_type: dummy }
```

**Key Sections**:
- **`context`**: Global settings including tmux session name and default parameters
- **`.defines`**: YAML anchors for SSH targets (user@IP) and Docker containers
- **`machine`**: Specifies which remote machine to run on via SSH
- **`docker`**: Specifies which Docker container to execute inside
- **`variants`**: Alternative configurations selectable via GUI

### 3. `xbot2/ModularBot.yaml`

The primary configuration file for `xbot2-core`:

```yaml
XBotInterface:
    urdf_path: $(rospack find modularbot)/urdf/ModularBot.urdf
    srdf_path: $(rospack find modularbot)/srdf/ModularBot.srdf
    joint_map_path: $(rospack find modularbot)/joint_map/ModularBot_joint_map.yaml

ModelInterface:
    model_type: RBDL
    is_model_floating_base: true

xbotcore_device_configs:
    sim: $(rospack find modularbot)/config/hal/ModularBot_gz.yaml
    dummy: $(rospack find modularbot)/config/hal/ModularBot_dummy.yaml
    ec_imp: $(rospack find modularbot)/config/hal/ModularBot_ec_all.yaml
    ec_idle: $(rospack find modularbot)/config/hal/ModularBot_ec_all.yaml
    ec_pos: $(rospack find modularbot)/config/hal/ModularBot_ec_all.yaml

xbotcore_threads:
    rt_main:
        sched: fifo
        prio: 60
        period: 0.001    # 1ms Real-Time thread
    nrt_main:
        sched: other
        prio: 0
        period: 0.005    # 5ms Non-Real-Time thread

xbotcore_plugins:
    ros_io:
        thread: nrt_main
        type: ros_io
    ros_ctrl:
        thread: nrt_main
        type: ros_control
    homing:
        thread: rt_main
        type: homing
    omnisteering:
        thread: nrt_main
        type: omnisteering_controller_plugin
    cartesio_imp:
        thread: rt_main
        type: albero_cartesio_rt
    # ... additional plugins
```

**Key Sections**:
- **`XBotInterface`**: Robot model paths (URDF, SRDF, joint map)
- **`xbotcore_device_configs`**: Maps `--hw` argument to HAL configuration files
- **`xbotcore_threads`**: Execution threads with scheduling properties
- **`xbotcore_plugins`**: Control plugins to load

### 4. `docker/concert-noble-ros2/compose.yml`

Docker Compose configuration for ROS2:

```yaml
services:
  base:
    image: hhcmhub/concert-noble-ros2-base:latest
    stdin_open: true
    tty: true
    privileged: true
    network_mode: host
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw  # X11 GUI forwarding
      - ~/.ssh:/home/user/.ssh             # SSH keys
    environment:
      - DISPLAY
      - HHCM_FOREST_CLONE_DEFAULT_PROTO=https

  dev:
    extends: base
    volumes:
      - ~/.cache/concert-noble-ros2-dev:/home/user/data

  sim:
    extends: base
    image: hhcmhub/concert-noble-ros2-sim:latest

  xbot2_gui_server:
    extends: base
    entrypoint: bash -ic "xbot2_gui_server gui/ros2/gui_server_config.yaml"
    restart: always
```

**Services**:
- **`base`**: Common configuration template
- **`dev`**: Development environment
- **`sim`**: Simulation environment with pre-built packages
- **`xbot2_gui_server`**: Persistent GUI server container

---

## Environment Configuration

### `docker/concert-config_ros2.env`

```bash
# Robot identification
export ROBOT_NAME=concert
export USER_NAME=user
export USER_ID=1000

# Repository configuration
export RECIPES_TAG=ros2
export RECIPES_REPO=git@github.com:advrhumanoids/multidof_recipes.git

# Packages to install
export ROBOT_PACKAGES="iit-concert-ros-pkg concert_config estimation_utils imu_simulator"
export ADDITIONAL_PACKAGES=""

# Docker image naming
export DISTRO=noble
export ROS_VERSION=ros2
export BASE_IMAGE_NAME=concert-noble-ros2
export TAGNAME=latest
export DOCKER_REGISTRY=hhcmhub
```

---

## Usage Workflows

### Simulation Workflow

```bash
# 1. On host machine - Start the development container
cd docker/concert-noble-ros2
source ./setup.sh
ros2 dev

# 2. Inside container - Configure simulation environment (first time only)
cd ~/xbot2_ws/src/iit-concert-ros-pkg/concert_config
./bashrc-setup-container.sh
source ~/.bashrc

# 3. Launch simulation (with xbot2 and GUI server included)
ros2 launch concert_gazebo modular.launch.py
```

**Step 4: Download and Launch GUI Client (on host machine, outside container)**

The GUI client is a desktop application that runs on your host machine and connects to the GUI server running inside the container.

**Download the GUI Client:**
1. Go to the [robot_monitoring releases page](https://github.com/ADVRHumanoids/robot_monitoring/releases)
2. Download the latest **XBot2 GUI Client** release for Ubuntu (e.g., `xbot2_gui_client_x86_64.tar.gz`)
3. Extract it to your home directory:

```bash
cd ~
tar -xzf xbot2_gui_client_x86_64.tar.gz
```

**Launch the GUI Client:**

```bash
# Open a NEW terminal on the host (not inside the container)
cd ~/xbot2_gui_client_x86_64/bin
./xbot2_gui
```

When the GUI opens, configure the server address:
- **Server IP**: `localhost` (or `127.0.0.1` for local simulation)
- **Port**: `8080` (default)

> **Note**: For detailed GUI documentation, see [robot_monitoring](https://github.com/ADVRHumanoids/robot_monitoring/tree/2.0-master)

**Alternative: Run GUI server separately** (for debugging or custom configuration):

```bash
# 1. Launch simulation WITHOUT xbot2_gui_server
ros2 launch concert_gazebo modular.launch.py xbot2_gui:=false

# 2. In a separate terminal inside the container, start GUI server manually
xbot2_gui_server ~/xbot2_ws/src/iit-concert-ros-pkg/concert_config/gui/ros2/gui_server_config.yaml

# 3. On host machine, launch the GUI client
cd ~/xbot2_gui_client_x86_64/bin
./xbot2_gui
```

### Real Robot Workflow

```bash
# 1. Start EtherCAT master
ecat_master

# 2. Start XBot2 with appropriate hardware mode
xbot2-core --hw ec_idle    # Idle mode (motors off)
xbot2-core --hw ec_pos     # Position control
xbot2-core --hw ec_imp     # Impedance control

# Or use concert_launcher
concert_launcher run ecat
concert_launcher run xbot2 --variant ctrl=ec_idle
```

### Using Concert Launcher

```bash
# List available processes
concert_launcher status

# Start a process
concert_launcher run xbot2

# Watch process output
concert_launcher watch xbot2

# Stop a process
concert_launcher kill xbot2
```

---

## Machine Configuration (Real Robot)

Update machine IPs in `gui/ros2/concert_launcher_config.yaml`:

| Alias | Default IP | Description |
|-------|-----------|-------------|
| `embedded` | 10.24.10.100 | Robot embedded PC (control) |
| `vision` | 10.24.10.101 | Vision computer |
| `control` | 10.24.10.102 | Control station PC |

---

## Troubleshooting

### DDS/Network Issues

If you see errors like `selected interface "lo" is not multicast-capable` or `failed to increase socket receive buffer size`:

**Increase socket buffer size (recommended, permanent fix)**

Run on the **host machine** (not inside Docker):
```bash
# Temporary (until reboot)
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=8388608

# Permanent (add to /etc/sysctl.conf)
echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_default=8388608" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_default=8388608" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### Port Already in Use

```bash
# Find and kill process using port 8080
sudo fuser -k 8080/tcp
```

### Missing rospy Errors in GUI

The `xbot2_gui_server` may show warnings about missing `rospy`:
```
ModuleNotFoundError: No module named 'rospy'
```

These are **non-critical** - the `dashboard.py` and `parameters.py` extensions require ROS1, but all core functionality works without them. The GUI will still function correctly.

---

## External Dependencies

- **Concert Launcher**: [https://github.com/ADVRHumanoids/concert_launcher](https://github.com/ADVRHumanoids/concert_launcher)
- **XBot2**: Real-time robot control framework
- **iit-concert-ros-pkg**: [Concert robot ROS packages (simulation)](https://github.com/ADVRHumanoids/iit-concert-ros-pkg)
- **docker-base**: Generalized Docker build system (submodule)

---

## Related Repositories

| Repository | Purpose |
|------------|---------|
| `iit-concert-ros-pkg` | Simulation packages (URDF, Gazebo, examples) |
| `modular` | Modular robot URDF generator |
| `xbot2_gui_server` | Web GUI for XBot2 |
| `concert_launcher` | Process management framework |
