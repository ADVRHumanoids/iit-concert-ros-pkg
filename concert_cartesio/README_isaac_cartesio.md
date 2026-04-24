# CartesIO for Concert in Isaac — validated runbook

## Architecture

- xbot2 stays minimal (homing + ros2_io + ros2_control + omnisteering).
- CartesIO runs as a ROS2 node (`cartesian_interface_ros ros_server_node`).
- Joint commands flow cartesio → ros2_control → xbot2 → Isaac.
- Wheels excluded from cartesio (`joint_blacklist`) since omnisteering owns them.

Pattern matches `iit-kyon-ros-pkg/kyon_cartesio/launch/kyon.launch`. The
private xbot2 plugin `albero_cartesio_rt` is not used (not in image).

## Build (one-time, inside concert-xbot2 container)

The workspace nests `concert_cartesio` under `iit-concert-ros-pkg/`, which
is itself a CMake meta-package. Default `colcon build` will not descend into
it, so pass an explicit base-path:

```bash
cd /home/user/xbot2_ws
colcon build \
  --base-paths src/iit-concert-ros-pkg/concert_cartesio \
  --packages-select concert_cartesio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Verify:

```bash
ros2 pkg prefix concert_cartesio
ls $(ros2 pkg prefix concert_cartesio)/share/concert_cartesio/launch/
# → concert_isaac_cartesio.launch.py + legacy ROS1 launches
```

## Runtime — three terminals

### Terminal 1 — Isaac Lab (isaac-sim container)

```bash
docker compose --profile isaac run --rm isaac-sim
isaaclab -p /workspace/concert_isaac/run_sim.py --enable_cameras --headless
```

Wait for: `[Concert] Setup complete. Waiting for xbot2 connection...`

For arm-pose export only, the lighter Isaac bringup is now preferable:

```bash
isaaclab -p /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py \
  --headless --xbot2-cartesio-mode
```

That mode keeps the robot + xbot2 socket server alive but skips Isaac-side ROS2
publishers and RTX lidar, which are not needed for CartesIO pose export.

### Terminal 2 — XBot2 (concert-xbot2 container)

```bash
rm -f /tmp/.xbot2_isaac/xbot2_isaac_server.sock.client.*
docker compose --profile xbot2 run --rm concert-xbot2
xbot2-core -C ModularBot_isaac_cartesio.yaml -H isaac
```

Verify URDF/SRDF are republished by xbot2 before step 3:

```bash
ros2 topic list | grep xbotcore
ros2 topic echo --once /xbotcore/robot_description | head -c 200
```

### Terminal 3 — CartesIO (concert-xbot2 container, second shell)

```bash
docker compose exec concert-xbot2 bash
source /home/user/xbot2_ws/install/setup.bash
ros2 launch concert_cartesio concert_isaac_cartesio.launch.py
```

Expected (validated 2026-04-17):

```
[ok  ] Successfully added Cartesian task with
   BASE LINK:   base_link
   DISTAL LINK: ee_E
[ok  ] Successfully added postural task 'Postural'
[ok  ] Successfully added task 'JointLimits' with type 'JointLimits'
[ok  ] Successfully added task 'VelocityLimits' with type 'VelocityLimits'
[ok  ] Loaded solver 'OpenSot'
[info] Online trajectory generator enabled
[info] ros_server_node: started looping @100.0 Hz
```

## Smoke test (no xbot2 / no Isaac required)

For a config-only test against any source of `/robot_description`:

```bash
ros2 launch concert_cartesio concert_isaac_cartesio.launch.py \
  use_xbot_topics:=false markers:=false
```

Then send a reachable goal in `base_link` frame:

```bash
ros2 topic pub --once /cartesian/tcp/reference geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link},
    pose: {position: {x: 0.6, y: 0.0, z: 0.4},
           orientation: {w: 1.0}}}"
ros2 topic echo --once /cartesian/tcp/current_reference
```

If `current_reference` matches the goal, the IK pipeline is healthy.

## Topics and services exposed

After successful load:
- `/cartesian/tcp/reference` (sub, PoseStamped) — set goal
- `/cartesian/tcp/current_reference` (pub, PoseStamped) — echo
- `/cartesian/tcp/task_error` (pub) — solver error
- `/cartesian/tcp/reach_pose` (action, ReachPose) — timed motion
- `/cartesian/tcp/set_base_link`, `/set_lambda`, `/set_active`, ...

Use `ros2 service list | grep cartesian` for the full map.

## Position-only export for GateFit arm poses

For the passive `ee_E` benchmark carrier we only need the tool point to reach
the beam grip-band center. A dedicated position-only CartesIO problem file is
committed as:

```bash
concert_cartesio/concert_isaac_stack_position_only.yaml
```

It uses `indices: [0, 1, 2]` on the `ee_E` Cartesian task so the exported
joint references are not biased by an arbitrary absolute orientation request.

To regenerate `configs/arm_poses.json` from the live lab stack:

```bash
python3 scripts/export_arm_poses_with_cartesio.py
```

That host-side exporter:
- restarts CartesIO in position-only mode,
- samples `/xbotcore/joint_states.position_reference`,
- validates the pose back against the committed URDF FK, and
- rewrites `configs/arm_poses.json`.

## Known failure modes

| Symptom | Cause | Fix |
|---|---|---|
| `LibNotFound: libxbotctrl_albero_cartesio_rt.so` | reverted to old xbot2 profile | use `ModularBot_isaac_cartesio.yaml` from this branch (no `cartesio_imp`) |
| `link 'world' does not exist` | floating-base task referenced non-existent `world` link | already fixed in `concert_isaac_stack.yaml` (only chain_E + Postural) |
| `waiting for urdf on topic '/xbotcore/robot_description'` (forever) | step 3 started before xbot2 published | wait for `/xbotcore/robot_description` first, or pass `use_xbot_topics:=false` for offline test |
| `primal infeasible / OpenSot: unable to solve` | goal outside reachable workspace | reduce magnitude of x/y/z or rotate orientation |
| `colcon ... ignoring unknown package 'concert_cartesio'` | nested under iit-concert-ros-pkg meta-package | use `--base-paths src/iit-concert-ros-pkg/concert_cartesio` |
| `running in visual mode: no commands sent to robot` | xbot2 not publishing `/xbotcore/joint_states` | start xbot2-core first |
