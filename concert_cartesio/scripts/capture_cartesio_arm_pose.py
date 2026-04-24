#!/usr/bin/env python3
"""Capture a single arm pose from the live Concert CartesIO stack.

This script is meant to run inside the `concert-xbot2` container, after:
  1. Isaac is running and connected to xbot2
  2. xbot2-core is running with `-H isaac`
  3. CartesIO ROS server is running

It publishes a position target for the passive `ee_E` tool point and returns
the arm joint `position_reference` vector chosen by CartesIO.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from xbot_msgs.msg import JointState


JOINT_ORDER = ["J1_E", "J2_E", "J3_E", "J4_E", "J5_E", "J6_E"]


class PoseCaptureNode(Node):
    def __init__(self, topic: str):
        super().__init__("gatefit_cartesio_pose_capture")
        self.latest_joint_state: JointState | None = None
        self.latest_reference: PoseStamped | None = None
        self.publisher = self.create_publisher(PoseStamped, topic, 10)

        joint_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            JointState,
            "/xbotcore/joint_states",
            self._joint_state_cb,
            joint_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/cartesian/tcp/current_reference",
            self._reference_cb,
            10,
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _reference_cb(self, msg: PoseStamped) -> None:
        self.latest_reference = msg


def _extract_arm(msg: JointState, field: str) -> list[float]:
    values = getattr(msg, field)
    return [float(values[msg.name.index(joint)]) for joint in JOINT_ORDER]


def _pose_error(msg: PoseStamped | None, target: np.ndarray) -> float | None:
    if msg is None:
        return None
    position = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=float,
    )
    return float(np.linalg.norm(position - target))


def _pose_position(msg: PoseStamped | None) -> np.ndarray | None:
    if msg is None:
        return None
    return np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=float,
    )


def _arm_snapshot(msg: JointState | None) -> dict[str, list[float]] | None:
    if msg is None:
        return None
    return {
        "link_position": _extract_arm(msg, "link_position"),
        "position_reference": _extract_arm(msg, "position_reference"),
        "link_velocity": _extract_arm(msg, "link_velocity"),
        "velocity_reference": _extract_arm(msg, "velocity_reference"),
    }


def _make_pose(frame: str, position: np.ndarray) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = float(position[0])
    msg.pose.position.y = float(position[1])
    msg.pose.position.z = float(position[2])
    msg.pose.orientation.w = 1.0
    return msg


def _publish_for_duration(
    node: PoseCaptureNode,
    frame: str,
    position: np.ndarray,
    duration_s: float,
    rate_hz: float,
) -> None:
    if duration_s <= 0.0:
        return
    period_s = 1.0 / max(rate_hz, 1.0)
    deadline = time.time() + duration_s
    msg = _make_pose(frame, position)
    while time.time() < deadline:
        node.publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=min(period_s, 0.05))
        time.sleep(period_s)


def _publish_segment(
    node: PoseCaptureNode,
    frame: str,
    start_pos: np.ndarray,
    end_pos: np.ndarray,
    duration_s: float,
    rate_hz: float,
) -> None:
    if duration_s <= 0.0:
        _publish_for_duration(node, frame, end_pos, 0.1, rate_hz)
        return
    steps = max(2, int(round(duration_s * max(rate_hz, 1.0))))
    period_s = duration_s / steps
    for index in range(1, steps + 1):
        alpha = index / steps
        pos = (1.0 - alpha) * start_pos + alpha * end_pos
        node.publisher.publish(_make_pose(frame, pos))
        rclpy.spin_once(node, timeout_sec=min(period_s, 0.05))
        time.sleep(period_s)


def _publish_waypoints(
    node: PoseCaptureNode,
    frame: str,
    waypoints: list[np.ndarray],
    duration_s: float,
    rate_hz: float,
) -> None:
    if len(waypoints) < 2:
        return

    segment_lengths = [
        float(np.linalg.norm(waypoints[idx + 1] - waypoints[idx]))
        for idx in range(len(waypoints) - 1)
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-9:
        _publish_for_duration(node, frame, waypoints[-1], duration_s, rate_hz)
        return

    remaining_duration = max(0.0, duration_s)
    for idx, segment_length in enumerate(segment_lengths):
        start_pos = waypoints[idx]
        end_pos = waypoints[idx + 1]
        if idx == len(segment_lengths) - 1:
            segment_duration = remaining_duration
        else:
            segment_duration = duration_s * (segment_length / total_length)
            remaining_duration = max(0.0, remaining_duration - segment_duration)
        _publish_segment(node, frame, start_pos, end_pos, segment_duration, rate_hz)


def _build_waypoints(
    start_pos: np.ndarray,
    target: np.ndarray,
    pregrasp_z_offset: float,
    safe_z_offset: float,
    approach_mode: str,
) -> tuple[list[np.ndarray], np.ndarray, str]:
    pregrasp = target + np.array([0.0, 0.0, max(0.0, pregrasp_z_offset)], dtype=float)
    resolved_mode = approach_mode
    if approach_mode == "auto":
        resolved_mode = "staged_lateral" if abs(float(target[1])) >= 0.20 else "vertical"

    if resolved_mode == "vertical":
        waypoints = [start_pos, pregrasp, target]
    elif resolved_mode == "staged_lateral":
        safe_z = max(float(start_pos[2]), float(pregrasp[2]), float(target[2])) + max(0.0, safe_z_offset)
        lift = np.array([float(start_pos[0]), float(start_pos[1]), safe_z], dtype=float)
        over_target = np.array([float(target[0]), float(target[1]), safe_z], dtype=float)
        waypoints = [start_pos, lift, over_target, pregrasp, target]
    else:
        raise ValueError(f"unsupported approach mode '{approach_mode}'")

    filtered = [waypoints[0]]
    for point in waypoints[1:]:
        if np.linalg.norm(point - filtered[-1]) > 1e-6:
            filtered.append(point)
    return filtered, pregrasp, resolved_mode


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture a CartesIO-derived arm pose.")
    parser.add_argument("--x", type=float, required=True, help="Target x in the chosen frame.")
    parser.add_argument("--y", type=float, required=True, help="Target y in the chosen frame.")
    parser.add_argument("--z", type=float, required=True, help="Target z in the chosen frame.")
    parser.add_argument("--frame", type=str, default="base_link", help="PoseStamped frame_id.")
    parser.add_argument(
        "--publish-duration",
        type=float,
        default=3.0,
        help="How long to command the full approach trajectory, in seconds.",
    )
    parser.add_argument(
        "--settle-duration",
        type=float,
        default=4.0,
        help="Extra observation time after reaching the final target, in seconds.",
    )
    parser.add_argument(
        "--pregrasp-z-offset",
        type=float,
        default=0.12,
        help="Vertical pre-grasp offset above the final target, in meters.",
    )
    parser.add_argument(
        "--approach-fraction",
        type=float,
        default=0.7,
        help="Fraction of publish-duration spent moving toward the pre-grasp waypoint.",
    )
    parser.add_argument(
        "--approach-mode",
        type=str,
        default="auto",
        choices=("auto", "vertical", "staged_lateral"),
        help="Trajectory staging mode for the live approach.",
    )
    parser.add_argument(
        "--safe-z-offset",
        type=float,
        default=0.18,
        help="Extra vertical lift used by staged_lateral approaches.",
    )
    parser.add_argument(
        "--publish-rate-hz",
        type=float,
        default=25.0,
        help="Waypoint publish rate for the commanded approach trajectory.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Overall timeout waiting for reference + joint-state data.",
    )
    parser.add_argument(
        "--observe-duration",
        type=float,
        default=1.0,
        help="Extra no-command observation window after settle, in seconds.",
    )
    parser.add_argument(
        "--reference-topic",
        type=str,
        default="/cartesian/tcp/reference",
        help="Reference topic to publish.",
    )
    args = parser.parse_args()

    target = np.array([args.x, args.y, args.z], dtype=float)

    rclpy.init()
    node = PoseCaptureNode(args.reference_topic)

    start = time.time()
    while time.time() - start < min(args.timeout, 1.0):
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)

    start_pos = _pose_position(node.latest_reference)
    if start_pos is None:
        start_pos = target + np.array([0.0, 0.0, max(0.0, args.pregrasp_z_offset)], dtype=float)

    waypoints, pregrasp, resolved_approach_mode = _build_waypoints(
        start_pos=start_pos,
        target=target,
        pregrasp_z_offset=args.pregrasp_z_offset,
        safe_z_offset=args.safe_z_offset,
        approach_mode=args.approach_mode,
    )

    approach_fraction = max(0.0, min(args.approach_fraction, 1.0))
    if resolved_approach_mode == "vertical":
        pregrasp_duration = args.publish_duration * approach_fraction
        final_duration = max(0.0, args.publish_duration - pregrasp_duration)
        if np.linalg.norm(pregrasp - start_pos) > 1e-5:
            _publish_segment(node, args.frame, start_pos, pregrasp, pregrasp_duration, args.publish_rate_hz)
        else:
            _publish_for_duration(node, args.frame, pregrasp, pregrasp_duration, args.publish_rate_hz)

        if np.linalg.norm(target - pregrasp) > 1e-5:
            _publish_segment(node, args.frame, pregrasp, target, final_duration, args.publish_rate_hz)
        else:
            _publish_for_duration(node, args.frame, target, final_duration, args.publish_rate_hz)
    else:
        _publish_waypoints(node, args.frame, waypoints, args.publish_duration, args.publish_rate_hz)

    _publish_for_duration(node, args.frame, target, args.settle_duration, args.publish_rate_hz)

    snapshot_a = _arm_snapshot(node.latest_joint_state)
    observe_deadline = time.time() + max(0.0, args.observe_duration)
    while time.time() < observe_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)
    snapshot_b = _arm_snapshot(node.latest_joint_state)

    joint_state = node.latest_joint_state
    if joint_state is None:
        raise RuntimeError("No /xbotcore/joint_states received.")
    if snapshot_a is None or snapshot_b is None:
        raise RuntimeError("Could not capture post-settle arm snapshots.")

    link_position_delta = (
        np.array(snapshot_b["link_position"], dtype=float)
        - np.array(snapshot_a["link_position"], dtype=float)
    )
    position_reference_delta = (
        np.array(snapshot_b["position_reference"], dtype=float)
        - np.array(snapshot_a["position_reference"], dtype=float)
    )

    result = {
        "joint_order": JOINT_ORDER,
        "frame_id": args.frame,
        "target_position": [float(v) for v in target],
        "pregrasp_position": [float(v) for v in pregrasp],
        "current_reference_error_m": _pose_error(node.latest_reference, target),
        "link_position": snapshot_b["link_position"],
        "position_reference": snapshot_b["position_reference"],
        "link_velocity": snapshot_b["link_velocity"],
        "velocity_reference": snapshot_b["velocity_reference"],
        "link_position_stability_delta_rad": [float(v) for v in link_position_delta],
        "position_reference_stability_delta_rad": [float(v) for v in position_reference_delta],
        "link_position_stability_max_abs_rad": float(np.max(np.abs(link_position_delta))),
        "position_reference_stability_max_abs_rad": float(np.max(np.abs(position_reference_delta))),
        "capture_profile": {
            "publish_duration_s": float(args.publish_duration),
            "settle_duration_s": float(args.settle_duration),
            "observe_duration_s": float(args.observe_duration),
            "pregrasp_z_offset_m": float(args.pregrasp_z_offset),
            "safe_z_offset_m": float(args.safe_z_offset),
            "approach_fraction": float(approach_fraction),
            "publish_rate_hz": float(args.publish_rate_hz),
            "approach_mode_requested": args.approach_mode,
            "approach_mode_used": resolved_approach_mode,
            "waypoints": [[float(v) for v in point] for point in waypoints],
        },
    }
    if node.latest_reference is not None:
        result["current_reference_position"] = [
            float(node.latest_reference.pose.position.x),
            float(node.latest_reference.pose.position.y),
            float(node.latest_reference.pose.position.z),
        ]

    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
