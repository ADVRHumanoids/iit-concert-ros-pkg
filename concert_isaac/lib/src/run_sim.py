"""Isaac Lab simulation server for Concert robot with xbot2 socket communication.

Runs the Concert robot in Isaac Lab and exposes a UNIX DGRAM socket server
for bidirectional communication with xbot2 (via zmq_hal plugin).

Protocol:
  - Discovery: xbot2 sends {type: discovery} -> server replies with {joint_names, imu_sensors}
  - State:     server broadcasts {type: state, time, q, dq, tau, k, d, qref, vref, tauref, imu}
  - Control:   xbot2 sends {type: control, q, dq, tau} -> server applies targets

Usage (inside isaac-sim container):
    isaaclab -p /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py --enable_cameras
    isaaclab -p /workspace/iit-concert-ros-pkg/concert_isaac/lib/src/run_sim.py --enable_cameras --real-time
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os
import sys

# NOTE: we need to launch this script inside an env tweaked for ros2 jazzy
# CycloneDDS:
# export ROS_DISTRO=jazzy
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib


from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Concert robot simulation server for xbot2.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import time

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.assets.articulation import Articulation
from isaaclab.sensors.imu import Imu
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

import numpy as np
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors.rtx import LidarRtx

import socket
import yaml

from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sensors.camera import Camera, CameraCfg
# import omni.client

# def _list_usd_recursive(base_url):
#     """Recursively list all .usd files under base_url via Nucleus."""
#     result, entries = omni.client.list(base_url)
#     if result != omni.client.Result.OK:
#         print(f"[list] Cannot access {base_url}: {result}")
#         return
#     for e in entries:
#         path = f"{base_url}/{e.relative_path}"
#         if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
#             _list_usd_recursive(path.rstrip("/"))
#         elif str(e.relative_path).endswith(".usd"):
#             print(path)

# _list_usd_recursive(f"{ISAAC_NUCLEUS_DIR}/Environments")
# exit(0)

# Enable the ROS 2 bridge extension so the publish writers are available.
enable_extension("isaacsim.ros2.bridge")

##
# Import Concert config
##
SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYTHON_SRC_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "python", "src"))

if PYTHON_SRC_DIR not in sys.path:
    sys.path.insert(0, PYTHON_SRC_DIR)

from concert_isaac.assets.concert_play import (CONCERT_CFG_PLAY,
                    _CONCERT_URDF,
                    _CONCERT_SRDF)  # noqa: E402

# Config variables
# note: it is ok to use the "NoAccumulator" variant, as the 
# ros2 publisher will take care of assembling the full scans
# RTX_LIDAR_ANNOTATOR = "IsaacCreateRTXLidarScanBuffer"
RTX_LIDAR_ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"

@configclass
class ConcertSceneCfg(InteractiveSceneCfg):
    """Configuration for the Concert simulation scene."""

    # ground plane / environment
    # Simple_Warehouse is a photorealistic warehouse streamed from Nucleus.
    # Falls back to a flat ground plane if Nucleus is not reachable.
    ground = AssetBaseCfg(
        prim_path="/World/environment",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd",
        ),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Concert robot
    robot: ArticulationCfg = CONCERT_CFG_PLAY.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # RGBD camera mounted on the robot's base_link.
    # prim_path uses the scene ENV_REGEX_NS so it is created under env_0.
    # offset convention="ros": +Z forward, -Y up (standard ROS camera frame).
    rgbd_camera: CameraCfg = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link/rgbd_camera",
        width=640,
        height=480,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            focus_distance=400.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.5, 0.0, 0.5),   # 0.5 m forward, 0.5 m above base_link
            rot=(0.5, -0.5, 0.5, -0.5),  # (w,x,y,z): rotate to face forward in ROS convention
            convention="ros",
        ),
    )


def setup_ros2_description_publishers():
    """Publish robot_description (URDF) and robot_description_semantic (SRDF) on ROS 2.

    Both topics use transient-local / reliable QoS (depth=1) so that any
    subscriber that connects after the publish still receives the message.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String

    if not rclpy.ok():
        rclpy.init()

    node = Node("concert_description_publisher")

    latching_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )

    urdf_pub = node.create_publisher(String, "/robot_description", latching_qos)
    srdf_pub = node.create_publisher(String, "/robot_description_semantic", latching_qos)

    with open(_CONCERT_URDF, "r") as f:
        urdf_str = f.read()
    with open(_CONCERT_SRDF, "r") as f:
        srdf_str = f.read()

    urdf_pub.publish(String(data=urdf_str))
    srdf_pub.publish(String(data=srdf_str))

    print(f"[Concert] Published /robot_description        ({len(urdf_str)} bytes, transient_local)")
    print(f"[Concert] Published /robot_description_semantic ({len(srdf_str)} bytes, transient_local)")

    # Keep the node alive so transient-local subscribers can receive the retained message.
    # We return the node; caller must not destroy it.
    return node


def spawn_usd_object(
    usd_path: str,
    prim_path: str,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
    static: bool = True,
) -> None:
    """Spawn a USD asset on the stage at the given pose.

    Args:
        usd_path:          Absolute path or Nucleus URL to the .usd / .usda / .usdz file.
        prim_path:         Desired USD stage path for the spawned prim, e.g.
                           "/World/my_object".
        position:          (x, y, z) translation in metres.
        orientation_wxyz:  Quaternion (w, x, y, z) for the initial orientation.
        scale:             (sx, sy, sz) uniform or non-uniform scale.
        static:            If True the object is a rigid body with collision but no
                           dynamics (collides with the floor, cannot be pushed).
                           If False a rigid-body API is applied so the object
                           participates in physics simulation.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    cfg = sim_utils.UsdFileCfg(
        usd_path=usd_path,
        scale=scale,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=not static,
            disable_gravity=False,
        ) if not static else None,
        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0) if not static else None,
    )

    # Spawn onto the stage
    prim = cfg.func(prim_path, cfg)

    # Apply translation and orientation via XformCommonAPI
    stage = sim_utils.SimulationContext.instance().stage if sim_utils.SimulationContext.instance() else None
    if stage is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()

    xform_prim = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xform_api = UsdGeom.XformCommonAPI(xform_prim)
    x, y, z = position
    w, qx, qy, qz = orientation_wxyz
    xform_api.SetTranslate(Gf.Vec3d(x, y, z))
    xform_api.SetRotate(Gf.Quatf(w, qx, qy, qz), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    sx, sy, sz = scale
    xform_api.SetScale(Gf.Vec3f(sx, sy, sz))

    kind = "static" if static else "dynamic"
    print(f"[Concert] Spawned {kind} object '{prim_path}'  pos={position}  from {usd_path}")


def setup_rgbd_camera(scene: InteractiveScene):
    """Sets up ROS 2 publishers for the RGBD camera defined in the scene config.

    Creates an OmniGraph pipeline that publishes:
      - /camera/rgb          (sensor_msgs/Image, encoding bgr8)
      - /camera/depth        (sensor_msgs/Image, encoding 32FC1)
      - /camera/camera_info  (sensor_msgs/CameraInfo)

    Must be called after sim.reset() so the camera prim exists on stage.
    """
    import omni.graph.core as og

    camera: Camera = scene["rgbd_camera"]
    # IsaacLab Camera already creates a render product for each env during _initialize_impl().
    # Reuse it directly — its path is a token string, which is exactly what ROS2CameraHelper
    # expects for inputs:renderProductPath.  No IsaacCreateRenderProduct / SetCamera needed.
    render_product_path = camera._render_product_paths[0]
    frame_id = "rgbd_camera"
    graph_path = "/World/ROS_RGBDCamera"

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),
                ("RunOnce",         "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("Context",         "isaacsim.ros2.bridge.ROS2Context"),
                ("RGBPublish",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish",    "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo",      "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                # render_product_path is a token string — set it directly as a value on each helper
                ("RGBPublish.inputs:renderProductPath",         render_product_path),
                ("RGBPublish.inputs:type",                      "rgb"),
                ("RGBPublish.inputs:topicName",                 "/camera/rgb"),
                ("RGBPublish.inputs:frameId",                   frame_id),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                ("DepthPublish.inputs:renderProductPath",       render_product_path),
                ("DepthPublish.inputs:type",                    "depth"),
                ("DepthPublish.inputs:topicName",               "/camera/depth"),
                ("DepthPublish.inputs:frameId",                 frame_id),
                ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfo.inputs:renderProductPath",         render_product_path),
                ("CameraInfo.inputs:topicName",                 "/camera/camera_info"),
                ("CameraInfo.inputs:frameId",                   frame_id),
                ("CameraInfo.inputs:resetSimulationTimeOnStop", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",  "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step",         "RGBPublish.inputs:execIn"),
                ("RunOnce.outputs:step",         "DepthPublish.inputs:execIn"),
                ("RunOnce.outputs:step",         "CameraInfo.inputs:execIn"),
                ("Context.outputs:context",      "RGBPublish.inputs:context"),
                ("Context.outputs:context",      "DepthPublish.inputs:context"),
                ("Context.outputs:context",      "CameraInfo.inputs:context"),
            ],
        },
    )

    print(f"[Concert] RGBD render product:      {render_product_path}")
    print(f"[Concert] Publishing /camera/rgb, /camera/depth, /camera/camera_info  (frame: {frame_id})")


def setup_sensors(sim: SimulationContext, scene: InteractiveScene):
    """Sets up Isaac Sim RTX Lidar sensor and attaches it to the robot base."""
    

    # Parent the lidar to base_link so it moves with the robot.
    # LidarRtx delegates to IsaacSensorCreateRtxLidar which takes `path` as the
    # leaf prim name and resolves parent separately — but the public constructor
    # only accepts a combined `prim_path`. Providing the full absolute path works
    # correctly now that the config resolves (no replicator fallback path-mangling).
    lidar_prim_path = "/World/envs/env_0/Robot/base_link/rtx_lidar"

    # Create the RTX lidar.
    # config_file_name must exactly match the USD stem (case-sensitive) as listed
    # in supported_lidar_configs.py: "HESAI_XT32_SD10" or shortened "XT32_SD10".
    lidar = LidarRtx(
        prim_path=lidar_prim_path,
        name="rtx_lidar",
        # translation is relative to the parent prim (base_link).
        # Place the sensor 0.3 m above base_link, centred horizontally.
        translation=np.array([0.5, 0.0, 0.3]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # (w, x, y, z)
        config_file_name="HESAI_XT32_SD10",
    )

    # Attach the point-cloud annotator to get 3-D hit positions each frame.
    lidar.attach_annotator(RTX_LIDAR_ANNOTATOR)

    # lidar.enable_visualization()  # for debugging; can disable if not needed

    # Publish sensor_msgs/PointCloud2 on ROS 2.
    # frameId should match the TF frame that consumers (e.g. RViz) expect.
    # NOTE: the writer name is split to avoid an early WriterRegistry scan that
    # would fire before the ROS 2 bridge extension finishes registering writers.
    lidar.attach_writer(
        "RtxLidar" + "ROS2PublishPointCloudBuffer",  # note: 
        topicName="/lidar/points",
        frameId="base_link",
    )

    print(f"[Concert] RTX Lidar prim:        {lidar_prim_path}")
    print(f"[Concert] RTX Lidar config:      HESAI_XT32_SD10")
    print(f"[Concert] Render product path:   {lidar.get_render_product_path()}")
    print(f"[Concert] Publishing PointCloud2: /lidar/points  (frame: base_link)")

    return lidar

def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, lidar: LidarRtx):
    """Runs the simulation loop with xbot2 socket communication."""

    # Extract scene entities
    robot: Articulation = scene["robot"]
    imu_sensors: dict[str, Imu] = {}
    for sn, s in scene.sensors.items():
        print(f"Sensor name: {sn}, type: {type(s)}")
        if isinstance(s, Imu):
            print(f"IMU sensor found: {sn}")
            imu_sensors[sn] = s

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    time_sim = 0
    # real-time factor tracking
    rtf_last_print = time.time()
    rtf_sim_elapsed = 0.0
    rtf_steps = 0

    # Socket for communication with xbot2 (via zmq_hal)
    server_socket_path = "/tmp/.xbot2_isaac/xbot2_isaac_server.sock"
    os.makedirs(os.path.dirname(server_socket_path), exist_ok=True)
    if os.path.exists(server_socket_path):
        os.unlink(server_socket_path)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(server_socket_path)
    sock.setblocking(False)
    os.chmod(server_socket_path, 0o777)

    print(f"[Concert] Server socket created at {server_socket_path}")
    print(f"[Concert] Joint names: {robot.joint_names}")
    print(f"[Concert] Num joints: {robot.num_joints}")

    client_sockets = set()

    # Set initial joint positions
    qinit = robot.data.default_joint_pos.clone()
    robot.write_joint_position_to_sim(qinit)
    robot.set_joint_position_target(qinit)
    print(f'[Concert] Initial joint positions: {qinit.cpu().numpy().flatten().tolist()}')

    while simulation_app.is_running():

        tic = time.time()

        # Broadcast robot state to all connected xbot2 clients
        state_msg = {'type': 'state'}
        state_msg['time'] = time_sim
        state_msg['q'] = robot.data.joint_pos.cpu().numpy().flatten().tolist()
        state_msg['dq'] = robot.data.joint_vel.cpu().numpy().flatten().tolist()
        state_msg['tau'] = robot.data.applied_torque.cpu().numpy().flatten().tolist()
        state_msg['k'] = robot.data.joint_stiffness.cpu().numpy().flatten().tolist()
        state_msg['d'] = robot.data.joint_damping.cpu().numpy().flatten().tolist()
        state_msg['qref'] = robot.data.joint_pos_target.cpu().numpy().flatten().tolist()
        state_msg['vref'] = robot.data.joint_vel_target.cpu().numpy().flatten().tolist()
        state_msg['tauref'] = robot.data.joint_effort_target.cpu().numpy().flatten().tolist()

        state_msg['imu'] = dict()
        for imu_name, imu_sensor in imu_sensors.items():
            imu_data = imu_sensor.data
            state_msg['imu'][imu_name] = {
                'quat_w': imu_data.quat_w.cpu().numpy().flatten().tolist(),
                'lin_acc_b': imu_data.lin_acc_b.cpu().numpy().flatten().tolist(),
                'ang_vel_b': imu_data.ang_vel_b.cpu().numpy().flatten().tolist(),
            }

        state_msg = yaml.dump(state_msg)
        sockets_to_remove = []
        for cli_addr in client_sockets:
            try:
                sock.sendto(state_msg.encode(), cli_addr)
            except ConnectionRefusedError:
                print(f"[Concert] Client at {cli_addr} disconnected.")
                sockets_to_remove.append(cli_addr)
            except Exception as e:
                print(f"[Concert] Error sending state to {cli_addr}: {e}")

        for s in sockets_to_remove:
            client_sockets.remove(s)
        sockets_to_remove.clear()

        # Handle client connections
        try:
            data, cli_addr = sock.recvfrom(4096)

            # Consume buffer (keep only latest message)
            while True:
                try:
                    data, cli_addr = sock.recvfrom(4096)
                except BlockingIOError:
                    break

            data = data.decode('utf-8')
            data = yaml.safe_load(data)
            data_type = data['type']

            if data_type == 'discovery':
                response = {'type': 'discovery'}
                response['joint_names'] = robot.joint_names
                response['imu_sensors'] = list(imu_sensors.keys())
                try:
                    sock.sendto(yaml.dump(response).encode('utf-8'), cli_addr)
                except Exception as e:
                    print(f"[Concert] Error sending discovery response to {cli_addr}: {e}")
                client_sockets.add(cli_addr)
                print(f"[Concert] Client at {cli_addr} connected. Sent {len(robot.joint_names)} joints.")

            elif data_type == 'control':
                joint_pos_def = torch.tensor(data['q'], device=robot.device).unsqueeze(0)
                robot.set_joint_position_target(joint_pos_def)
                joint_vel_def = torch.tensor(data['dq'], device=robot.device).unsqueeze(0)
                robot.set_joint_velocity_target(joint_vel_def)
                joint_effort_def = torch.tensor(data['tau'], device=robot.device).unsqueeze(0)
                robot.set_joint_effort_target(joint_effort_def)

            else:
                print(f"[Concert] Unknown data type received: {data_type}")

        except BlockingIOError:
            pass

        # Simulation step
        scene.write_data_to_sim()
        sim.step()
        time_sim += sim_dt
        rtf_sim_elapsed += sim_dt
        rtf_steps += 1
        count += 1
        scene.update(sim_dt)

        # Keep the viewport camera behind and above the robot base, tracking its heading.
        # root_quat_w is (num_envs, 4) as (w, x, y, z); root_pos_w is (num_envs, 3).
        robot_pos = robot.data.root_pos_w[0].cpu().numpy()   # [x, y, z]
        quat = robot.data.root_quat_w[0].cpu().numpy()       # [w, x, y, z]
        # Extract yaw from quaternion: yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
        w, x, y, z = quat
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        # Camera sits 3 m behind the robot (opposite to its forward direction) and 2 m up.
        cam_behind = np.array([-np.cos(yaw), -np.sin(yaw), 0.0]) * 6.0
        cam_eye = robot_pos + cam_behind + np.array([0.0, 0.0, 4.0])
        sim.set_camera_view(eye=cam_eye.tolist(), target=robot_pos.tolist())

        # Print real-time factor every ~1s
        now = time.time()
        real_elapsed = now - rtf_last_print
        if real_elapsed >= 1.0:
            real_time_factor = rtf_sim_elapsed / real_elapsed if real_elapsed > 0 else float('inf')
            print(
                f"[Concert] RTF: {real_time_factor:.3f}x "
                f"(sim: {rtf_sim_elapsed:.3f}s, real: {real_elapsed:.3f}s, steps: {rtf_steps})"
            )
            rtf_last_print = now
            rtf_sim_elapsed = 0.0
            rtf_steps = 0

        # # Print RTX lidar point-cloud summary
        # frame = lidar.get_current_frame()
        # pc_data = frame.get(RTX_LIDAR_ANNOTATOR, None)
        # if pc_data is not None and isinstance(pc_data, dict):
        #     points = pc_data.get("data", None)
        #     n_pts = len(points) if points is not None else 0
        # else:
        #     n_pts = 0
        # print(f"[Concert] RTX Lidar point cloud: {n_pts} points this frame")

        # Time delay for real-time evaluation
        toc = time.time()
        sleep_time = sim_dt - (toc - tic)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

    # Design scene
    scene_cfg = ConcertSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    # Setup sensors — must happen before sim.reset() so the prim exists on stage
    lidar = setup_sensors(sim, scene)

    # Publish robot_description and robot_description_semantic with transient-local QoS
    # so any subscriber (e.g. robot_state_publisher, MoveIt) that connects later still
    # receives them. The returned node must stay alive for the duration of the process.
    description_node = setup_ros2_description_publishers()

    # Play the simulator
    sim.reset()

    # initialize() wires up the per-frame data-acquisition callback; must be
    # called after sim.reset() so the physics/render context is fully ready.
    lidar.initialize()

    # Wire up the RGBD camera ROS 2 OmniGraph pipeline.
    # Must be called after sim.reset() so the camera prim is on stage.
    setup_rgbd_camera(scene)

    print("[Concert] Setup complete. Waiting for xbot2 connection...")

    # Run the simulator
    run_simulator(sim, scene, lidar)


if __name__ == "__main__":
    main()
    simulation_app.close()
