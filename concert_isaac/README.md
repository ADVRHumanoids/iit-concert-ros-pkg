# Concert Isaac

This package now mirrors the Kyon-style layout:

- `concert_isaac/docker/iit-concert-ros-pkg-isaac`
- `concert_isaac/lib/src`
- `concert_isaac/python/src/concert_isaac`
- sibling asset packages:
  - `concert_urdf`
  - `concert_srdf`
  - `concert_usd`

Canonical deployment assets under `iit-concert-ros-pkg`:

- `concert_usd/usd/concert_complete/concert_complete.usd`
- `concert_urdf/urdf/concert_complete.urdf`
- `concert_srdf/srdf/ModularBot.srdf`

Runtime note:

- Isaac loads the committed USD from `concert_usd`.
- XBot2 still publishes `robot_description` and `robot_description_semantic`
  through the official generator `concert_examples/src/concert_example.py`.
- The generated URDF/SRDF are post-processed only for the two previously
  validated fixes:
  - add the missing `imu_material` name in the URDF
  - remove the invalid `hands` block from the SRDF

## USD generation

In addition to the *concert_complete* example configuration that is available "out of the box", 
we provide the possibility to generate URDF/SRDF/USD from the Modular python script, as follows:

```bash
# run from the host machine!
# this will (i) invoke the provided modular script (concert_example.py in this case),
# (ii) save urdf and srdf to the assets directory,
# and (iii) generate the usd folder, also inside the assets directory
# note: replace 'myrobot' with a descriptive name for the modula robot configuration (e.g. 'concert_base_only')
python concert_isaac/lib/src/generate_modular_usd.py concert_examples/src/concert_example.py myrobot -- --headless
```

*Note* Isaac will fail if any asset name (such as .stl files) contains dash ('-') characters. Make sure t

## Package-local Docker flow

Work from the package-local docker directory:

```bash
cd ~/cosmos-isaac-project/concert_robot/iit-concert-ros-pkg/concert_isaac/docker/iit-concert-ros-pkg-isaac
mkdir -p /tmp/.xbot2_isaac
```

### Build the xbot2 image

```bash
DOCKER_BUILDKIT=0 docker compose build concert-xbot2
```

### Validate the runtime image and the generator

```bash
docker compose run --rm concert-xbot2 bash -ic "/workspace/iit-concert-ros-pkg/concert_isaac/lib/src/concert_xbot2_setup.sh"
```

Expected output includes:

```text
[concert-xbot2] image is ready
/workspace/iit-concert-ros-pkg/concert_usd/usd/concert_complete/concert_complete.usd
```

## Helper scripts

The Kyon-style host wrappers live under `concert_isaac/lib/src`:

- `run_sim_host.bash`
- `run_xbot2_host.bash`
- `run_xbot2_setup_host.bash`

### Start Isaac

From anywhere in the repo:

```bash
concert_robot/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim_host.bash --enable_cameras --headless
```

This brings up the package-local `isaac-sim` service and runs:

```bash
isaaclab -p /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py
```

### Start the xbot2 shell

```bash
concert_robot/iit-concert-ros-pkg/concert_isaac/lib/src/run_xbot2_host.bash
```

This follows the same clean pattern as the previous root-level flow:

```bash
cd ~/cosmos-isaac-project/concert_robot/iit-concert-ros-pkg/concert_isaac/docker/iit-concert-ros-pkg-isaac
docker compose run --rm concert-xbot2
```

The default command already runs the xbot2 helper shell, so inside that shell:

```bash
xbot2-core -C ModularBot_isaac.yaml -H isaac
```

### Run the xbot2 setup check

```bash
concert_robot/iit-concert-ros-pkg/concert_isaac/lib/src/run_xbot2_setup_host.bash
```

## Manual step-by-step test

### 1. Terminal 1: Isaac

```bash
cd ~/cosmos-isaac-project/concert_robot/iit-concert-ros-pkg/concert_isaac/docker/iit-concert-ros-pkg-isaac
docker compose run --rm isaac-sim
```

Inside the container:

```bash
isaaclab -p /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py --enable_cameras --headless
```

### 2. Terminal 2: XBot2

```bash
cd ~/cosmos-isaac-project/concert_robot/iit-concert-ros-pkg/concert_isaac/docker/iit-concert-ros-pkg-isaac
rm -f /tmp/.xbot2_isaac/xbot2_isaac_server.sock.client.*
docker compose run --rm concert-xbot2
```

Inside the shell:

```bash
xbot2-core -C ModularBot_isaac.yaml -H isaac
```

### 3. Terminal 3: attach to the running xbot2 container

```bash
cd ~/cosmos-isaac-project/concert_robot/iit-concert-ros-pkg/concert_isaac/docker/iit-concert-ros-pkg-isaac
docker compose exec concert-xbot2 bash
```

If `ros2` or `xbot2-core` is not found after attaching, source manually:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh
source /home/user/xbot2_ws/setup.bash
source /home/user/env/bin/activate
export PYTHONPATH=/home/user/xbot2_ws/src/modular/src${PYTHONPATH:+:$PYTHONPATH}
```

Verify descriptions:

```bash
timeout 5 ros2 topic echo --once /robot_description >/dev/null && echo URDF_READY
timeout 5 ros2 topic echo --once /robot_description_semantic >/dev/null && echo SRDF_READY
```

Verify XBot2 topics and services:

```bash
ros2 topic list
ros2 service list | grep xbotcore
```

### 4. Homing

```bash
ros2 service call /xbotcore/homing/switch std_srvs/srv/SetBool "{data: true}"
ros2 service call /xbotcore/homing/state xbot_msgs/srv/PluginStatus "{}"
```

Abort if needed:

```bash
ros2 service call /xbotcore/homing/abort std_srvs/srv/Trigger "{}"
```

### 5. Omnisteering

```bash
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
ros2 topic pub --once /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 6. Optional GUI server

In another shell attached to the running `concert-xbot2` container, start the
backend:

```bash
xbot2_gui_server
```

Keep that process running in the foreground.

Then in a separate terminal start the frontend:

```bash
xbot2_gui
```

If you run the frontend from another attached shell and `xbot2_gui` is not in
`PATH`, source the environment first as shown in Terminal 3.

## 10. Clean restart procedure

If `xbot2-core` fails with:

```text
Error binding local socket ... xbot2_isaac_server.sock.client.<pid>: Address already in use
```

clean the stale client sockets on the host:

```bash
rm -f /tmp/.xbot2_isaac/xbot2_isaac_server.sock.client.*
```

If you want to stop only running `concert-xbot2` containers without getting an
error when none are running:

```bash
ids=$(docker ps --filter ancestor=cosmos-concert-xbot2:local -q)
[ -n "$ids" ] && docker stop $ids
```

Then open a fresh XBot2 shell again:

```bash
docker compose run --rm concert-xbot2
```

Do not delete `/tmp/.xbot2_isaac/xbot2_isaac_server.sock` while Isaac is still running.

## 11. Minimal isolation profile

If you want to isolate model loading and the Isaac HAL path without ROS2
control or omnisteering noise, run:

```bash
xbot2-core -C ModularBot_isaac_minimal.yaml -H isaac
```

## 12. Expected success criteria

The deployment flow is working if all of the following are true:
- Isaac starts and loads `/workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py`
- `/robot_description` and `/robot_description_semantic` are readable
- `xbot2-core -C ModularBot_isaac.yaml -H isaac` reaches `started running`
- `/xbotcore/joint_states` is published
- `/xbotcore/homing/switch` works
- `/omnisteering/cmd_vel` moves the robot in Isaac
