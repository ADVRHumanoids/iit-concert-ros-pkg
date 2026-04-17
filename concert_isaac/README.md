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

## Validated changes in this checkout

These are the changes that made the current Isaac + XBot2 + CartesIO flow work:

- `concert_isaac/lib/src/run_sim.bash`
  - now launches through `isaaclab.sh -p` instead of plain `python`
- `concert_isaac/lib/src/run_sim.py`
  - enables `isaacsim.sensors.rtx` before importing `LidarRtx`
  - uses the packaged `concert_play` config path
  - no longer spawns the unrelated metal tube from the other project
- `concert_config/xbot2/ModularBot_isaac_cartesio.yaml`
  - adds a separate Isaac profile for the CartesIO bringup
  - keeps the stable xbot2 plugins and deliberately does not load the missing
    private `albero_cartesio_rt` plugin
- `concert_cartesio/launch/concert_isaac_cartesio.launch.py`
  - launches the public ROS-side CartesIO server (`cartesian_interface_ros`)
  - remaps robot descriptions from `/xbotcore/robot_description{,_semantic}`
- `concert_cartesio/concert_isaac_stack.yaml`
  - defines the arm IK task on `ee_E`
  - excludes wheel/steering joints so CartesIO does not fight omnisteering

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

### 6. CartesIO arm IK

The validated CartesIO path is ROS-side, not the private xbot2 RT plugin path.
In other words:

- keep `xbot2-core` running with `ModularBot_isaac_cartesio.yaml`
- launch `cartesian_interface_ros` as a separate ROS2 node
- let xbot2's `ros2_control` bridge forward the solved arm commands to Isaac

Why this route:

- the old `albero_cartesio_rt` plugin path requires
  `libxbotctrl_albero_cartesio_rt.so`
- that library is not present in the current `concert-xbot2` image
- its recipes pull private repos that are not accessible in this environment

#### Build `concert_cartesio` once

Preferred path in this workspace: use the existing Forest recipe for
`iit-concert-ros-pkg`. This was validated on 2026-04-17 and it does reach
`concert_cartesio` because the recipe explicitly lists it in the `cmakelists`
section.

```bash
docker compose exec concert-xbot2 bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /opt/xbot/setup.sh &&
  source /home/user/xbot2_ws/setup.bash &&
  cd /home/user/xbot2_ws &&
  forest grow iit-concert-ros-pkg --no-deps --force-reconfigure -j 4
'
```

Notes:

- the tested Forest command used the workspace default build type
  `RelWithDebInfo`
- if you want a strict Release build through Forest, use:

```bash
forest grow iit-concert-ros-pkg --no-deps --force-reconfigure -j 4 -t Release
```

Focused fallback: build only `concert_cartesio` with an explicit base-path.
This is still useful when you want to touch only the nested CartesIO package
without rebuilding the other `iit-concert-ros-pkg` subprojects:

```bash
docker compose exec concert-xbot2 bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /opt/xbot/setup.sh &&
  source /home/user/xbot2_ws/setup.bash &&
  cd /home/user/xbot2_ws &&
  colcon build \
    --base-paths src/iit-concert-ros-pkg/concert_cartesio \
    --packages-select concert_cartesio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
'
```

#### Terminal 2: start xbot2 with the CartesIO-friendly profile

```bash
xbot2-core -C ModularBot_isaac_cartesio.yaml -H isaac
```

This profile intentionally keeps the stable Isaac plugins:

- `homing`
- `ros2_io`
- `ros2_control`
- `omnisteering`

and does **not** try to load the unavailable private impedance plugin.

#### Terminal 3: launch the ROS-side CartesIO server

```bash
docker compose exec concert-xbot2 bash
source /opt/ros/jazzy/setup.bash
source /home/user/xbot2_ws/install/setup.bash
ros2 launch concert_cartesio concert_isaac_cartesio.launch.py markers:=false
```

Expected success lines include:

```text
[ok  ] Successfully added Cartesian task with
   BASE LINK:   base_link
   DISTAL LINK: ee_E
[ok  ] Successfully added postural task 'Postural'
[ok  ] Loaded solver 'OpenSot'
[info] ros_server_node: started looping @100.0 Hz
```

#### Send a TCP goal

Goal 1:

```bash
ros2 topic pub --once /cartesian/tcp/reference geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.55, y: 0.00, z: 0.55},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Goal 2:

```bash
ros2 topic pub --once /cartesian/tcp/reference geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.50, y: 0.10, z: 0.50},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### Verify that the goal was accepted

```bash
ros2 topic echo --once /cartesian/tcp/current_reference
ros2 topic echo --once /xbotcore/joint_states
```

On the validated run from 2026-04-17:

- Goal 1 moved the arm joints by up to `3.05 rad`
- Goal 2 moved the arm joints by up to `1.22 rad`
- `/cartesian/tcp/current_reference` matched the commanded poses exactly

So the tested path is:

```text
CartesIO -> ros2_control -> xbot2 -> Isaac
```

#### CartesIO topics and services

```bash
ros2 topic list | grep /cartesian
ros2 service list | grep /cartesian
```

Important interfaces:

- `/cartesian/tcp/reference`
- `/cartesian/tcp/current_reference`
- `/cartesian/tcp/task_error`
- `/cartesian/tcp/set_active`
- `/cartesian/tcp/set_lambda`

### 7. Optional GUI server

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

If CartesIO is waiting forever for the URDF or xbot2 hangs without reaching
`started running`, the most common cause is a stale or dead Isaac socket. A
clean recovery is:

```bash
# host
rm -f /tmp/.xbot2_isaac/xbot2_isaac_server.sock.client.*

# isaac-sim container
rm -f /tmp/.xbot2_isaac/xbot2_isaac_server.sock /tmp/.xbot2_isaac/xbot2_isaac_server.sock.client.*
```

Then relaunch Isaac first, wait for the server socket to come back, and only
after that restart `xbot2-core`.

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
- `ros2 launch concert_cartesio concert_isaac_cartesio.launch.py markers:=false` reaches `ros_server_node: started looping`
- `/cartesian/tcp/current_reference` matches commanded goals
- arm joints `J1_E..J6_E` change after a CartesIO goal is sent
