# Docker configuration for Concert robot
# Uses docker-base submodule (xbot2_docker) for build templates

This directory contains Concert-specific Docker configuration that works with
the shared docker-base build system.

## Structure

```
docker/
├── build-concert.bash        # Main build script wrapper
├── concert-config_ros1.env   # ROS1 environment configuration
├── concert-config_ros2.env   # ROS2 environment configuration
├── concert-focal-ros1/       # ROS1 compose and setup
│   ├── compose.yml
│   └── setup.sh
└── concert-noble-ros2/       # ROS2 compose and setup
    ├── compose.yml
    └── setup.sh
```

## Prerequisites

The docker-base submodule must be initialized:

```bash
cd concert_config
git submodule update --init --recursive
```

## Building Images

### ROS2 (default)
```bash
./build-concert.bash           # Build locally
./build-concert.bash --push    # Build and push to registry
./build-concert.bash --pull    # Pull from registry
```

### ROS1
```bash
./build-concert.bash ros1           # Build locally
./build-concert.bash ros1 --push    # Build and push to registry
./build-concert.bash ros1 --pull    # Pull from registry
```

## How It Works

1. `build-concert.bash` is a thin wrapper that:
   - Loads the appropriate environment file (concert-config_ros1.env or concert-config_ros2.env)
   - Sets Concert-specific packages and configuration
   - Delegates to `docker-base/robot-template/_build/robot-{distro}-{ros}/build.bash`

2. The docker-base build system:
   - Uses Docker Bake or Docker Compose to build layered images
   - Creates base, xeno (real-time), and sim/locomotion images
   - Tags and optionally pushes to the registry

## Images Created

### ROS2 (Noble)
- `hhcmhub/concert-noble-ros2-base:latest` - Development environment
- `hhcmhub/concert-noble-ros2-xeno-v5:latest` - Real-time with Xenomai
- `hhcmhub/concert-noble-ros2-sim:latest` - Simulation with MuJoCo

### ROS1 (Focal)
- `hhcmhub/concert-focal-ros1-base:latest` - Development environment
- `hhcmhub/concert-focal-ros1-xeno-v5:latest` - Real-time with Xenomai
- `hhcmhub/concert-focal-ros1-locomotion:latest` - Locomotion stack

## Running Containers

Use the compose files in the subdirectories:

```bash
cd concert-noble-ros2
docker compose run dev  # Development container
docker compose run sim  # Simulation container
```

## Customization

Edit the environment files to customize:
- `ROBOT_PACKAGES` - Packages to install
- `RECIPES_TAG` - Forest recipes branch
- `TAGNAME` - Image version tag

## Persistent Source Sharing (Bidirectional Host ↔ Container)

To make changes to source code persist across container restarts and be editable from your host:

### Step 1: Extract files (one-time, run on HOST)

```bash
mkdir -p ~/xbot2_ws_shared

# Copy files from container (run as root to bypass permission issues)
docker run --rm --user root \
  -v ~/xbot2_ws_shared:/backup \
  hhcmhub/concert-noble-ros2-base:latest \
  bash -c "cp -r /home/user/xbot2_ws/src/* /backup/"

# Fix ownership to your user
sudo chown -R $(id -u):$(id -g) ~/xbot2_ws_shared

# Grant container user (UID 1000) access via ACL
sudo setfacl -R -m u:1000:rwx ~/xbot2_ws_shared
sudo setfacl -R -d -m u:1000:rwx ~/xbot2_ws_shared  # Default for new files
```

### Step 2: Clean up old containers

```bash
cd ~/concert_config_pattern/concert_config/docker/concert-noble-ros2
docker compose down --remove-orphans
```

### Step 3: Start normally (your usual workflow)

```bash
source setup.sh
ros2 dev
```

Now files in `~/xbot2_ws_shared/` on your host are shared bidirectionally with `/home/user/xbot2_ws/src/` in the container.

**Note:** The ACL grants both your host user and the container's user (UID 1000) read/write access.
