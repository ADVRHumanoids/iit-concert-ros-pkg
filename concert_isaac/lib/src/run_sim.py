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
import json
import math
import os
import sys
from pathlib import Path

# NOTE: we need to launch this script inside an env tweaked for ros2 jazzy
# CycloneDDS:
# export ROS_DISTRO=jazzy
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib


from isaaclab.app import AppLauncher

_DEFAULT_PIPE_LOG_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "logs", "acea_pipe_junction")
)

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
parser.add_argument(
    "--pipe-scene",
    choices=("seam", "no_seam"),
    default="seam",
    help="Pipe scene variant: two sections with a butt-joint gap, or one continuous no-seam negative.",
)
parser.add_argument("--pipe-gap", type=float, default=0.003, help="Rendered butt-joint gap in metres.")
parser.add_argument("--pipe-diameter", type=float, default=0.90, help="Outer pipe diameter in metres.")
parser.add_argument("--pipe-section-length", type=float, default=1.0, help="Length of each pipe section in metres.")
parser.add_argument(
    "--pipe-seam-center",
    type=float,
    nargs=3,
    default=(2.0, 0.5015, 1.0),
    metavar=("X", "Y", "Z"),
    help="World position of the nominal seam/reference plane in metres.",
)
parser.add_argument(
    "--pipe-axis-yaw-deg",
    type=float,
    default=0.0,
    help="Yaw rotation of the pipe asset about world Z. Local tube +Y is the pipe axis.",
)
parser.add_argument(
    "--rgbd-camera-pos",
    type=float,
    nargs=3,
    default=(0.5, 0.0, 0.5),
    metavar=("X", "Y", "Z"),
    help="RGB-D camera offset relative to the mounting link, in metres.",
)
parser.add_argument(
    "--rgbd-camera-rot-wxyz",
    type=float,
    nargs=4,
    default=(0.5, -0.5, 0.5, -0.5),
    metavar=("W", "X", "Y", "Z"),
    help="RGB-D camera orientation offset as a quaternion in wxyz order.",
)
parser.add_argument("--rgbd-camera-width", type=int, default=640, help="RGB-D camera image width.")
parser.add_argument("--rgbd-camera-height", type=int, default=480, help="RGB-D camera image height.")
parser.add_argument("--rgbd-camera-focal-length", type=float, default=24.0, help="RGB-D focal length in mm.")
parser.add_argument(
    "--rgbd-camera-horizontal-aperture",
    type=float,
    default=20.955,
    help="RGB-D horizontal aperture in mm.",
)
parser.add_argument(
    "--pipe-log-dir",
    type=str,
    default=_DEFAULT_PIPE_LOG_DIR,
    help="Directory where ACEA pipe-junction debug captures are written.",
)
parser.add_argument(
    "--pipe-log-run-id",
    type=str,
    default="latest",
    help="Deterministic subdirectory name under --pipe-log-dir. Re-running overwrites matching frame files.",
)
parser.add_argument(
    "--pipe-log-start-frame",
    type=int,
    default=60,
    help="First simulation frame used for deterministic RGB-D/pose logging.",
)
parser.add_argument(
    "--pipe-log-interval",
    type=int,
    default=0,
    help="If >0, keep logging every N frames after --pipe-log-start-frame. If 0, log one frame.",
)
parser.add_argument(
    "--disable-pipe-logging",
    action="store_true",
    default=False,
    help="Disable ACEA pipe-junction debug file outputs.",
)
parser.add_argument(
    "--arm-scan-pose",
    choices=("retracted", "config_home"),
    default="retracted",
    help="Initial arm posture for scanning. 'retracted' keeps the arm away from the RGB-D view.",
)

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
try:
    from isaacsim.sensors.rtx import LidarRtx
except ModuleNotFoundError:
    LidarRtx = None

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

from concert_isaac.assets.concert_complete_play import (CONCERT_CFG_PLAY,
                    _CONCERT_URDF,
                    _CONCERT_SRDF)  # noqa: E402

# Config variables
# note: it is ok to use the "NoAccumulator" variant, as the 
# ros2 publisher will take care of assembling the full scans
# RTX_LIDAR_ANNOTATOR = "IsaacCreateRTXLidarScanBuffer"
RTX_LIDAR_ANNOTATOR = "IsaacExtractRTXSensorPointCloudNoAccumulator"

PIPE_ASSET_OUTER_RADIUS_M = 0.45
PIPE_ASSET_LENGTH_M = 1.0
PIPE_SCENE_ROOT = "/World/acea_pipe_junction"

ARM_RETRACTED_SCAN_POSE = {
    "J1_E": 0.0,
    "J2_E": 0.40,
    "J3_E": 0.0,
    "J4_E": -1.15,
    "J5_E": 0.0,
    "J6_E": 0.0,
}


def _round_float(value: float, ndigits: int = 6) -> float:
    return round(float(value), ndigits)


def _round_list(values, ndigits: int = 6) -> list[float]:
    return [_round_float(v, ndigits) for v in values]


def _yaw_quat_wxyz(yaw_rad: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw_rad
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _pipe_axis_from_yaw(yaw_rad: float) -> np.ndarray:
    # The tube asset is authored along local +Y.
    return np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0], dtype=float)


def _pose_from_usd_prim(prim_path: str) -> dict[str, list[float]] | None:
    try:
        from pxr import UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return None

        world_tf = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
        translation = world_tf.ExtractTranslation()
        quat = world_tf.ExtractRotationQuat()
        imag = quat.GetImaginary()
        return {
            "position_world": _round_list([translation[0], translation[1], translation[2]]),
            "quaternion_wxyz_world": _round_list([quat.GetReal(), imag[0], imag[1], imag[2]]),
        }
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def _tensor_to_list(tensor, ndigits: int = 6):
    if tensor is None:
        return None
    try:
        return _round_list(tensor.detach().cpu().numpy().flatten().tolist(), ndigits)
    except Exception:
        try:
            return _round_list(tensor.cpu().numpy().flatten().tolist(), ndigits)
        except Exception:
            return None


def build_pipe_scene_metadata(metal_tube_usd: str) -> dict[str, object]:
    gap_m = float(args_cli.pipe_gap)
    diameter_m = float(args_cli.pipe_diameter)
    section_length_m = float(args_cli.pipe_section_length)
    if gap_m < 0.0:
        raise ValueError("--pipe-gap must be non-negative")
    if diameter_m <= 0.0:
        raise ValueError("--pipe-diameter must be positive")
    if section_length_m <= 0.0:
        raise ValueError("--pipe-section-length must be positive")

    radius_m = 0.5 * diameter_m
    seam_center = np.array(args_cli.pipe_seam_center, dtype=float)
    yaw_rad = math.radians(float(args_cli.pipe_axis_yaw_deg))
    axis = _pipe_axis_from_yaw(yaw_rad)
    pipe_quat = _yaw_quat_wxyz(yaw_rad)
    radial_scale = radius_m / PIPE_ASSET_OUTER_RADIUS_M
    section_scale = section_length_m / PIPE_ASSET_LENGTH_M
    seam_present = args_cli.pipe_scene == "seam"

    metadata: dict[str, object] = {
        "task": "acea_pipe_junction_detection",
        "scene_mode": args_cli.pipe_scene,
        "seam_present": seam_present,
        "asset": {
            "metal_tube_usd": metal_tube_usd,
            "nominal_outer_radius_m": PIPE_ASSET_OUTER_RADIUS_M,
            "nominal_length_m": PIPE_ASSET_LENGTH_M,
        },
        "pipe": {
            "diameter_m": _round_float(diameter_m),
            "radius_m": _round_float(radius_m),
            "section_length_m": _round_float(section_length_m),
            "nominal_gap_m": _round_float(gap_m),
            "rendered_gap_m": _round_float(gap_m if seam_present else 0.0),
            "seam_center_world": _round_list(seam_center),
            "axis_world": _round_list(axis),
            "axis_yaw_deg": _round_float(args_cli.pipe_axis_yaw_deg),
            "orientation_wxyz_world": _round_list(pipe_quat),
        },
        "ground_truth": {
            "seam_present": seam_present,
            "reference_plane": {
                "position_world": _round_list(seam_center),
                "normal_world": _round_list(axis),
                "quaternion_wxyz_world": _round_list(pipe_quat),
            },
        },
    }

    if seam_present:
        center_offset = axis * (0.5 * section_length_m + 0.5 * gap_m)
        left_center = seam_center - center_offset
        right_center = seam_center + center_offset
        left_end = seam_center - axis * (0.5 * gap_m)
        right_end = seam_center + axis * (0.5 * gap_m)
        sections = [
            {
                "name": "negative_axis_section",
                "prim_path": f"{PIPE_SCENE_ROOT}/section_negative_axis",
                "center_world": _round_list(left_center),
                "orientation_wxyz_world": _round_list(pipe_quat),
                "scale_xyz": _round_list((radial_scale, section_scale, radial_scale)),
                "length_m": _round_float(section_length_m),
            },
            {
                "name": "positive_axis_section",
                "prim_path": f"{PIPE_SCENE_ROOT}/section_positive_axis",
                "center_world": _round_list(right_center),
                "orientation_wxyz_world": _round_list(pipe_quat),
                "scale_xyz": _round_list((radial_scale, section_scale, radial_scale)),
                "length_m": _round_float(section_length_m),
            },
        ]
        metadata["pipe"]["sections"] = sections
        metadata["ground_truth"]["seam"] = {
            "position_world": _round_list(seam_center),
            "normal_world": _round_list(axis),
            "quaternion_wxyz_world": _round_list(pipe_quat),
            "gap_width_m": _round_float(gap_m),
            "negative_axis_pipe_end_center_world": _round_list(left_end),
            "positive_axis_pipe_end_center_world": _round_list(right_end),
        }
    else:
        total_length = (2.0 * section_length_m) + gap_m
        total_scale = total_length / PIPE_ASSET_LENGTH_M
        metadata["pipe"]["sections"] = [
            {
                "name": "continuous_no_seam_pipe",
                "prim_path": f"{PIPE_SCENE_ROOT}/continuous_no_seam",
                "center_world": _round_list(seam_center),
                "orientation_wxyz_world": _round_list(pipe_quat),
                "scale_xyz": _round_list((radial_scale, total_scale, radial_scale)),
                "length_m": _round_float(total_length),
            }
        ]
        metadata["ground_truth"]["seam"] = None
        metadata["ground_truth"]["no_seam_reference"] = {
            "position_world": _round_list(seam_center),
            "normal_world": _round_list(axis),
            "quaternion_wxyz_world": _round_list(pipe_quat),
        }

    return metadata


def spawn_pipe_junction_scene(metal_tube_usd: str) -> dict[str, object]:
    metadata = build_pipe_scene_metadata(metal_tube_usd)
    for section in metadata["pipe"]["sections"]:
        spawn_usd_object(
            usd_path=metal_tube_usd,
            prim_path=section["prim_path"],
            position=tuple(section["center_world"]),
            orientation_wxyz=tuple(section["orientation_wxyz_world"]),
            scale=tuple(section["scale_xyz"]),
            static=True,
        )

    pipe = metadata["pipe"]
    print(
        "[ACEA Pipe] Scene "
        f"mode={metadata['scene_mode']} gap={pipe['rendered_gap_m']:.6f} m "
        f"diameter={pipe['diameter_m']:.3f} m seam_center={pipe['seam_center_world']} "
        f"axis={pipe['axis_world']}"
    )
    return metadata


def apply_arm_scan_pose(robot: Articulation) -> torch.Tensor:
    qinit = robot.data.default_joint_pos.clone()
    if args_cli.arm_scan_pose != "retracted":
        return qinit

    missing = []
    for joint_name, value in ARM_RETRACTED_SCAN_POSE.items():
        try:
            joint_idx = robot.joint_names.index(joint_name)
        except ValueError:
            missing.append(joint_name)
            continue
        qinit[0, joint_idx] = float(value)

    if missing:
        print(f"[ACEA Pipe] Arm scan pose skipped missing joints: {missing}")
    else:
        print(f"[ACEA Pipe] Arm scan pose: retracted ({ARM_RETRACTED_SCAN_POSE})")
    return qinit


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
        width=args_cli.rgbd_camera_width,
        height=args_cli.rgbd_camera_height,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=args_cli.rgbd_camera_focal_length,
            focus_distance=400.0,
            horizontal_aperture=args_cli.rgbd_camera_horizontal_aperture,
            clipping_range=(0.1, 20.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=tuple(args_cli.rgbd_camera_pos),
            rot=tuple(args_cli.rgbd_camera_rot_wxyz),
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
    from pxr import Gf, UsdGeom, UsdPhysics, Vt

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

    # Apply translation, orientation and scale via raw xformOp attributes.
    # XformCommonAPI.SetRotate() only accepts Euler angles (GfVec3f), not quaternions,
    # and fails on prims whose xformOp stack was already set by UsdFileCfg.
    # Writing the ops directly works regardless of the existing stack.
    stage = sim_utils.SimulationContext.instance().stage if sim_utils.SimulationContext.instance() else None
    if stage is None:
        import omni.usd
        stage = omni.usd.get_context().get_stage()

    xformable = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    x, y, z = position
    w, qx, qy, qz = orientation_wxyz

    # Clear any existing ops written by the spawner so we start clean.
    xformable.ClearXformOpOrder()

    translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    translate_op.Set(Gf.Vec3d(x, y, z))

    orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    orient_op.Set(Gf.Quatf(w, qx, qy, qz))

    sx, sy, sz = scale
    scale_op = xformable.AddScaleOp(UsdGeom.XformOp.PrecisionFloat)
    scale_op.Set(Gf.Vec3f(sx, sy, sz))

    kind = "static" if static else "dynamic"
    print(f"[Concert] Spawned {kind} object '{prim_path}'  pos={position}  from {usd_path}")


def setup_rgbd_camera(scene: InteractiveScene):
    """Sets up ROS 2 publishers for the RGBD camera defined in the scene config.

    Uses the same Replicator/SDG writer pipeline as the RTX lidar:
      - attach_writer() on the camera's render product rather than building
        an OmniGraph action graph by hand.

    Publishes:
      - /camera/rgb          (sensor_msgs/Image, bgr8)
      - /camera/depth        (sensor_msgs/Image, 32FC1)
      - /camera/camera_info  (sensor_msgs/CameraInfo)

    Writer name construction (mirrors extension.py registration):
      RGB   : "LdrColorSD"            + "ROS2PublishImage"   -> "LdrColorSDROS2PublishImage"
      Depth : "DistanceToImagePlaneSD" + "ROS2PublishImage"  -> "DistanceToImagePlaneSDROS2PublishImage"
      Info  : "ROS2PublishCameraInfo"

    Must be called after sim.reset() so the camera render product exists.
    """
    import omni.replicator.core as rep

    camera: Camera = scene["rgbd_camera"]
    # Use the concrete camera prim path (not the render product path string) so that
    # rep.create.render_product returns — or reuses — exactly the same HydraTexture
    # that IsaacLab created during _initialize_impl(). Passing the already-resolved
    # render product path string can cause Replicator to fall back to the viewport camera.
    cam_prim_path = camera._view.prim_paths[0]
    rp = rep.create.render_product(
        cam_prim_path,
        resolution=(camera.cfg.width, camera.cfg.height),
    )
    render_product_path = rp if isinstance(rp, str) else rp.path

    frame_id = "rgbd_camera"

    # RGB image — writer name matches the registered SDG writer for LdrColorSD
    rgb_writer = rep.WriterRegistry.get("LdrColorSD" + "ROS2PublishImage")
    rgb_writer.initialize(topicName="/camera/rgb", frameId=frame_id)
    rgb_writer.attach([rp])

    # Depth image (distance_to_image_plane, 32FC1)
    depth_writer = rep.WriterRegistry.get("DistanceToImagePlaneSD" + "ROS2PublishImage")
    depth_writer.initialize(topicName="/camera/depth", frameId=frame_id)
    depth_writer.attach([rp])

    # Camera info
    info_writer = rep.WriterRegistry.get("ROS2PublishCameraInfo")
    info_writer.initialize(topicName="/camera/camera_info", frameId=frame_id)
    info_writer.attach([rp])

    print(f"[Concert] RGBD camera prim:         {cam_prim_path}")
    print(f"[Concert] RGBD render product:      {render_product_path}")
    print(f"[Concert] Publishing /camera/rgb, /camera/depth, /camera/camera_info  (frame: {frame_id})")


def _camera_prim_path(camera: Camera) -> str | None:
    try:
        return camera._view.prim_paths[0]
    except Exception:
        return None


def _camera_intrinsics(camera: Camera) -> dict[str, object]:
    data = camera.data
    matrix = None
    source = "computed_from_camera_cfg"
    try:
        matrix = data.intrinsic_matrices[0].detach().cpu().numpy()
        source = "isaaclab_camera_data"
    except Exception:
        width = float(camera.cfg.width)
        height = float(camera.cfg.height)
        focal_length = float(camera.cfg.spawn.focal_length)
        horizontal_aperture = float(camera.cfg.spawn.horizontal_aperture)
        fx = focal_length / horizontal_aperture * width
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float)

    return {
        "source": source,
        "width": int(camera.cfg.width),
        "height": int(camera.cfg.height),
        "frame_id": "rgbd_camera",
        "K": _round_list(matrix.reshape(-1).tolist()),
        "matrix_3x3": [[_round_float(v) for v in row] for row in matrix.tolist()],
        "pinhole_cfg": {
            "focal_length_mm": _round_float(camera.cfg.spawn.focal_length),
            "horizontal_aperture_mm": _round_float(camera.cfg.spawn.horizontal_aperture),
            "clipping_range_m": _round_list(camera.cfg.spawn.clipping_range),
        },
        "offset_cfg": {
            "mounting_prim_path": camera.cfg.prim_path,
            "position_xyz": _round_list(camera.cfg.offset.pos),
            "quaternion_wxyz": _round_list(camera.cfg.offset.rot),
            "convention": str(camera.cfg.offset.convention),
        },
    }


def _camera_pose(camera: Camera) -> dict[str, object]:
    data = camera.data
    pose = {
        "data_pos_w": _tensor_to_list(getattr(data, "pos_w", None)),
        "data_quat_w_world": _tensor_to_list(getattr(data, "quat_w_world", None)),
        "data_quat_w_ros": _tensor_to_list(getattr(data, "quat_w_ros", None)),
    }
    prim_path = _camera_prim_path(camera)
    pose["prim_path"] = prim_path
    if prim_path is not None:
        pose["usd_world_pose"] = _pose_from_usd_prim(prim_path)
    return pose


def _robot_pose(robot: Articulation) -> dict[str, object]:
    return {
        "root_position_world": _tensor_to_list(robot.data.root_pos_w[0]),
        "root_quaternion_wxyz_world": _tensor_to_list(robot.data.root_quat_w[0]),
        "joint_names": list(robot.joint_names),
        "joint_position": _tensor_to_list(robot.data.joint_pos[0]),
        "joint_velocity": _tensor_to_list(robot.data.joint_vel[0]),
        "arm_scan_pose": args_cli.arm_scan_pose,
    }


def _save_rgb_tensor(tensor, path: Path) -> None:
    from PIL import Image

    image = tensor[0].detach().cpu().numpy()
    if image.dtype.kind == "f":
        if float(np.nanmax(image)) <= 1.0 + 1e-3:
            image = image * 255.0
        image = np.clip(image, 0.0, 255.0).astype(np.uint8)
    else:
        image = image.astype(np.uint8)
    Image.fromarray(image[:, :, :3]).save(str(path))


def _save_depth_outputs(tensor, raw_path: Path, vis_path: Path) -> dict[str, object]:
    from PIL import Image

    depth = tensor[0].detach().cpu().numpy()
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[:, :, 0]
    depth = depth.astype(np.float32)
    np.save(str(raw_path), depth)

    valid = np.isfinite(depth) & (depth > 0.0) & (depth < 100.0)
    stats = {
        "shape": list(depth.shape),
        "raw_npy": str(raw_path),
        "visualization_png": str(vis_path),
        "valid_pixel_count": int(valid.sum()),
    }
    if valid.any():
        d_min = float(np.percentile(depth[valid], 1.0))
        d_max = float(np.percentile(depth[valid], 99.0))
        denom = max(d_max - d_min, 1e-6)
        depth_vis = np.clip((depth - d_min) / denom, 0.0, 1.0)
        stats.update({"valid_min_m": _round_float(depth[valid].min()), "valid_max_m": _round_float(depth[valid].max())})
        stats.update({"vis_percentile_min_m": _round_float(d_min), "vis_percentile_max_m": _round_float(d_max)})
    else:
        depth_vis = np.zeros(depth.shape, dtype=np.float32)
    Image.fromarray((depth_vis * 255.0).astype(np.uint8)).save(str(vis_path))
    return stats


class PipeJunctionLogger:
    def __init__(self, scene_metadata: dict[str, object]):
        self.enabled = not args_cli.disable_pipe_logging
        self.scene_metadata = scene_metadata
        self.output_dir = Path(args_cli.pipe_log_dir).expanduser().resolve() / args_cli.pipe_log_run_id
        self.logged_once = False
        self.next_frame = int(args_cli.pipe_log_start_frame)
        self.interval = int(args_cli.pipe_log_interval)
        if not self.enabled:
            print("[ACEA Pipe] Debug logging disabled.")
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = self.output_dir / "manifest.jsonl"
        if manifest_path.exists():
            manifest_path.unlink()
        scene_path = self.output_dir / "scene_metadata.json"
        scene_path.write_text(json.dumps(scene_metadata, indent=2) + "\n", encoding="utf-8")
        print(f"[ACEA Pipe] Debug log directory: {self.output_dir}")
        print(f"[ACEA Pipe] Scene metadata:     {scene_path}")

    def maybe_log(self, frame_index: int, sim_time_s: float, scene: InteractiveScene, robot: Articulation) -> None:
        if not self.enabled:
            return
        if frame_index < self.next_frame:
            return
        if self.logged_once and self.interval <= 0:
            return
        if self.logged_once and self.interval > 0 and frame_index < self.next_frame:
            return

        try:
            camera: Camera = scene["rgbd_camera"]
            rgb_tensor = camera.data.output["rgb"]
            depth_tensor = camera.data.output["distance_to_image_plane"]
        except Exception as exc:
            print(f"[ACEA Pipe] Camera not ready for logging at frame {frame_index}: {exc}")
            self.next_frame = frame_index + 10
            return

        frame_dir = self.output_dir / f"frame_{frame_index:06d}"
        frame_dir.mkdir(parents=True, exist_ok=True)
        rgb_path = frame_dir / "rgb.png"
        depth_raw_path = frame_dir / "depth_m.npy"
        depth_vis_path = frame_dir / "depth_vis.png"
        frame_metadata_path = frame_dir / "metadata.json"

        _save_rgb_tensor(rgb_tensor, rgb_path)
        depth_stats = _save_depth_outputs(depth_tensor, depth_raw_path, depth_vis_path)

        frame_metadata = {
            "frame_index": int(frame_index),
            "sim_time_s": _round_float(sim_time_s),
            "scene": self.scene_metadata,
            "camera": {
                "intrinsics": _camera_intrinsics(camera),
                "pose": _camera_pose(camera),
            },
            "robot_base": _robot_pose(robot),
            "outputs": {
                "rgb_png": str(rgb_path),
                "depth_raw_npy": str(depth_raw_path),
                "depth_vis_png": str(depth_vis_path),
                "depth_stats": depth_stats,
            },
        }
        frame_metadata_path.write_text(json.dumps(frame_metadata, indent=2) + "\n", encoding="utf-8")

        manifest_path = self.output_dir / "manifest.jsonl"
        with manifest_path.open("a", encoding="utf-8") as manifest:
            manifest.write(json.dumps({"frame_index": frame_index, "metadata": str(frame_metadata_path)}) + "\n")

        self.logged_once = True
        self.next_frame = frame_index + self.interval if self.interval > 0 else frame_index + 1
        print(f"[ACEA Pipe] Logged frame {frame_index} to {frame_dir}")


def setup_sensors(sim: SimulationContext, scene: InteractiveScene):
    """Sets up Isaac Sim RTX Lidar sensor and attaches it to the robot base."""

    if LidarRtx is None:
        print("[Concert] RTX Lidar module not available; skipping /lidar/points.")
        print("[Concert] RGB-D camera publishing and ACEA pipe logging remain enabled.")
        return None

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

def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    lidar,
    pipe_logger: PipeJunctionLogger,
):
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
    qinit = apply_arm_scan_pose(robot)
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
        pipe_logger.maybe_log(count, time_sim, scene, robot)

        if 0 > 1:
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

    # Spawn any additional USD objects that are not part of the scene config
    import concert_isaac.assets 
    assets_path = os.path.dirname(concert_isaac.assets.__file__)
    metal_tube_usd = os.path.join(assets_path, "metal_tube_configurable.usda")
    pipe_scene_metadata = spawn_pipe_junction_scene(metal_tube_usd)

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
    if lidar is not None:
        lidar.initialize()

    # Wire up the RGBD camera ROS 2 OmniGraph pipeline.
    # Must be called after sim.reset() so the camera prim is on stage.
    setup_rgbd_camera(scene)
    pipe_logger = PipeJunctionLogger(pipe_scene_metadata)

    print("[Concert] Setup complete. Waiting for xbot2 connection...")

    # Run the simulator
    run_simulator(sim, scene, lidar, pipe_logger)


if __name__ == "__main__":
    main()
    simulation_app.close()
