#!/usr/bin/env python3
"""Coarse 3D seam localization for ACEA pipe-junction RGB-D logs.

This is Phase 5 of the ACEA Module 2 perception demo. It consumes the
Phase 4 temporal-confirmation result, localizes the confirmed seam in the
confirmed RGB-D frame, and writes a deterministic coarse 3D handoff estimate.

The estimate is intentionally classical and debug-oriented. It uses the
pipe-aligned strip candidate, the depth-supported pipe mask, camera
intrinsics, and the Phase 2 pipe-axis estimate. It does not use ROS, CartesIO,
lidar, or machine learning.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

import pipe_depth_tracker
import pipe_strip_seam_detector
import pipe_temporal_confirmation


DEFAULT_RUN = "seam_3mm_sequence"


def _repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _round(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def _round_list(values, digits: int = 6) -> list[float]:
    return [_round(v, digits) for v in values]


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return vector.astype(np.float64)
    return vector.astype(np.float64) / norm


def _quat_wxyz_to_rot(quat: list[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in quat]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        raise ValueError("Invalid zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _backproject(depth: np.ndarray, xs: np.ndarray, ys: np.ndarray, intrinsics: list[list[float]]) -> np.ndarray:
    k = np.asarray(intrinsics, dtype=np.float64)
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    z = depth[ys, xs].astype(np.float64)
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def _project(points_camera: np.ndarray, intrinsics: list[list[float]]) -> np.ndarray:
    k = np.asarray(intrinsics, dtype=np.float64)
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    z = np.maximum(points_camera[:, 2], 1e-9)
    u = fx * points_camera[:, 0] / z + cx
    v = fy * points_camera[:, 1] / z + cy
    return np.stack([u, v], axis=1)


def _depth_visual(depth: np.ndarray) -> Image.Image:
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < 100.0)
    if valid.any():
        lo, hi = np.percentile(depth[valid], [1.0, 99.0])
        image = np.clip((depth - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        image = np.zeros_like(depth, dtype=np.float32)
    return Image.fromarray((image * 255.0).astype(np.uint8)).convert("RGBA")


def _default_temporal_args() -> argparse.Namespace:
    return argparse.Namespace(
        output_dir=None,
        min_confidence=0.35,
        min_confirm_frames=2,
        max_axis_angle_delta_deg=3.0,
        max_stand_off_delta_m=0.15,
        max_yaw_delta_deg=5.0,
        max_candidate_jump_px=120,
        reset_geometry_on_reject=False,
    )


def _ensure_temporal_summary(run_dir: Path) -> dict[str, Any]:
    summary_path = run_dir / "temporal_confirmation" / "temporal_summary.json"
    if summary_path.exists():
        return _load_json(summary_path)
    return pipe_temporal_confirmation.process_run(run_dir, None, _default_temporal_args())


def _ensure_seam_summary(frame_dir: Path) -> dict[str, Any]:
    summary_path = frame_dir / "pipe_strip" / "seam_detector_summary.json"
    if summary_path.exists():
        return _load_json(summary_path)
    return pipe_strip_seam_detector.process_frame(frame_dir, None, pipe_temporal_confirmation._default_detector_args())


def _ensure_tracker_summary(frame_dir: Path) -> dict[str, Any]:
    summary_path = frame_dir / "pipe_tracker" / "pipe_tracker_summary.json"
    if summary_path.exists():
        return _load_json(summary_path)
    tracker_args = argparse.Namespace(
        min_depth_m=0.05,
        max_depth_m=20.0,
        sample_stride=4,
        max_pca_points=60000,
        min_pipe_pixels=1000,
        keep_largest_component=False,
    )
    return pipe_depth_tracker.process_frame(frame_dir, None, tracker_args)


def _resolve_input_run(input_path: Path) -> Path:
    path = input_path.expanduser().resolve()
    if (path / "metadata.json").exists():
        return path.parent
    if path.is_dir():
        return path
    raise FileNotFoundError(f"Could not find run directory: {input_path}")


def _confirmed_frame_dir(temporal_summary: dict[str, Any]) -> Path:
    state = temporal_summary["state_machine"]
    if not state.get("confirmed"):
        raise RuntimeError("Temporal confirmation did not confirm a seam; cannot localize.")

    confirmed_frame_dir = state.get("confirmed_frame_dir")
    if confirmed_frame_dir:
        return Path(confirmed_frame_dir)

    for row in temporal_summary.get("per_frame", []):
        if row.get("state") in ("CONFIRMED", "STOP_AND_LOCALIZE"):
            return Path(row["frame_dir"])
    raise RuntimeError("Temporal summary is confirmed but does not include a confirmed frame directory.")


def _inverse_rotate_uv(points_uv: np.ndarray, image_size_wh: tuple[int, int], angle_deg: float) -> np.ndarray:
    width, height = image_size_wh
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    theta = math.radians(angle_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    shifted = points_uv.astype(np.float64) - np.array([[cx, cy]], dtype=np.float64)
    x = shifted[:, 0]
    y = shifted[:, 1]
    original_x = cos_t * x + sin_t * y + cx
    original_y = -sin_t * x + cos_t * y + cy
    return np.stack([original_x, original_y], axis=1)


def _candidate_line_points_original(
    candidate_x_rot: int,
    crop_xyxy: list[int],
    image_size_wh: tuple[int, int],
    rotation_deg: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    y0, y1 = float(crop_xyxy[1]), float(crop_xyxy[3])
    points_rot = np.array([[float(candidate_x_rot), y0], [float(candidate_x_rot), y1]], dtype=np.float64)
    points_original = _inverse_rotate_uv(points_rot, image_size_wh, rotation_deg)
    return (float(points_original[0, 0]), float(points_original[0, 1])), (
        float(points_original[1, 0]),
        float(points_original[1, 1]),
    )


def _surface_support_pixels(
    frame_dir: Path,
    seam_summary: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    rgb = Image.open(frame_dir / "rgb.png").convert("RGB")
    depth = np.load(frame_dir / "depth_m.npy")
    pipe_mask = np.asarray(Image.open(frame_dir / "pipe_tracker" / "pipe_mask.png").convert("L")) > 0

    rotation_deg = float(seam_summary["normalization"]["rotation_deg"])
    crop_xyxy = [int(v) for v in seam_summary["normalization"]["crop_xyxy"]]
    det = seam_summary["seam_detection"]
    candidate_x_strip = int(det["candidate_x_strip_px"])
    candidate_x_rot = crop_xyxy[0] + candidate_x_strip

    _, rotated_mask = pipe_strip_seam_detector._rotate_image_and_mask(rgb, pipe_mask, rotation_deg)
    height, width = depth.shape
    x0 = max(0, candidate_x_rot - args.surface_band_half_width_px)
    x1 = min(width - 1, candidate_x_rot + args.surface_band_half_width_px)
    y0 = max(0, crop_xyxy[1])
    y1 = min(height - 1, crop_xyxy[3])

    band_mask = np.zeros(rotated_mask.shape, dtype=bool)
    band_mask[y0:y1 + 1, x0:x1 + 1] = rotated_mask[y0:y1 + 1, x0:x1 + 1]
    ys_rot, xs_rot = np.nonzero(band_mask)
    if xs_rot.size == 0:
        raise RuntimeError("No rotated pipe-mask pixels found around the confirmed seam candidate.")

    rotated_uv = np.stack([xs_rot.astype(np.float64), ys_rot.astype(np.float64)], axis=1)
    original_uv = _inverse_rotate_uv(rotated_uv, rgb.size, rotation_deg)
    xs = np.rint(original_uv[:, 0]).astype(np.int64)
    ys = np.rint(original_uv[:, 1]).astype(np.int64)
    in_bounds = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
    xs = xs[in_bounds]
    ys = ys[in_bounds]

    valid = pipe_mask[ys, xs] & np.isfinite(depth[ys, xs]) & (depth[ys, xs] > 0.0) & (depth[ys, xs] < 100.0)
    xs = xs[valid]
    ys = ys[valid]
    if xs.size < args.min_surface_points:
        raise RuntimeError(
            f"Only {xs.size} depth-supported seam pixels found; expected at least {args.min_surface_points}."
        )

    unique_uv = np.unique(np.stack([xs, ys], axis=1), axis=0)
    xs = unique_uv[:, 0]
    ys = unique_uv[:, 1]

    return {
        "rgb": rgb,
        "depth": depth,
        "xs": xs,
        "ys": ys,
        "candidate_x_strip_px": candidate_x_strip,
        "candidate_x_rotated_px": int(candidate_x_rot),
        "candidate_line_original_uv": _candidate_line_points_original(
            candidate_x_rot,
            crop_xyxy,
            rgb.size,
            rotation_deg,
        ),
        "surface_band_rotated_xyxy": [int(x0), int(y0), int(x1), int(y1)],
    }


def _camera_usd_pose(metadata: dict[str, Any]) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        pose = metadata["camera"]["pose"]["usd_world_pose"]
        position = np.asarray(pose["position_world"], dtype=np.float64)
        rotation = _quat_wxyz_to_rot(pose["quaternion_wxyz_world"])
        return position, rotation
    except Exception:
        return None


def _optical_point_to_world(point_camera: np.ndarray, metadata: dict[str, Any]) -> list[float] | None:
    pose = _camera_usd_pose(metadata)
    if pose is None:
        return None
    position_world, rot_world_from_usd_camera = pose
    point_usd_camera = np.array([point_camera[0], point_camera[1], -point_camera[2]], dtype=np.float64)
    return _round_list(rot_world_from_usd_camera @ point_usd_camera + position_world)


def _optical_vector_to_world(vector_camera: np.ndarray, metadata: dict[str, Any]) -> list[float] | None:
    pose = _camera_usd_pose(metadata)
    if pose is None:
        return None
    _, rot_world_from_usd_camera = pose
    vector_usd_camera = np.array([vector_camera[0], vector_camera[1], -vector_camera[2]], dtype=np.float64)
    return _round_list(_normalize(rot_world_from_usd_camera @ vector_usd_camera))


def _world_point_to_optical(point_world: list[float], metadata: dict[str, Any]) -> list[float] | None:
    pose = _camera_usd_pose(metadata)
    if pose is None:
        return None
    position_world, rot_world_from_usd_camera = pose
    point_usd_camera = rot_world_from_usd_camera.T @ (np.asarray(point_world, dtype=np.float64) - position_world)
    point_optical = np.array([point_usd_camera[0], point_usd_camera[1], -point_usd_camera[2]], dtype=np.float64)
    return _round_list(point_optical)


def _world_vector_to_optical(vector_world: list[float], metadata: dict[str, Any]) -> list[float] | None:
    pose = _camera_usd_pose(metadata)
    if pose is None:
        return None
    _, rot_world_from_usd_camera = pose
    vector_usd_camera = rot_world_from_usd_camera.T @ np.asarray(vector_world, dtype=np.float64)
    vector_optical = np.array([vector_usd_camera[0], vector_usd_camera[1], -vector_usd_camera[2]], dtype=np.float64)
    return _round_list(_normalize(vector_optical))


def _angle_between_axes_deg(a: np.ndarray, b: np.ndarray) -> float:
    a = _normalize(a)
    b = _normalize(b)
    dot = float(np.clip(abs(np.dot(a, b)), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _draw_overlay(
    base: Image.Image,
    selected_uv: np.ndarray,
    line_uv: tuple[tuple[float, float], tuple[float, float]],
    surface_uv: list[float],
    center_uv: list[float] | None,
    output_path: Path,
) -> None:
    image = base.convert("RGBA")
    overlay = np.zeros((image.height, image.width, 4), dtype=np.uint8)

    if selected_uv.size:
        step = max(1, selected_uv.shape[0] // 6000)
        for x, y in selected_uv[::step]:
            xi = int(x)
            yi = int(y)
            if 0 <= xi < image.width and 0 <= yi < image.height:
                overlay[max(0, yi - 1):min(image.height, yi + 2), max(0, xi - 1):min(image.width, xi + 2)] = (
                    0,
                    220,
                    160,
                    120,
                )

    image = Image.alpha_composite(image, Image.fromarray(overlay, mode="RGBA"))
    draw = ImageDraw.Draw(image)
    draw.line(
        (line_uv[0][0], line_uv[0][1], line_uv[1][0], line_uv[1][1]),
        fill=(0, 210, 255, 255),
        width=3,
    )

    def marker(uv: list[float], color: tuple[int, int, int, int], radius: int) -> None:
        u, v = uv
        draw.ellipse((u - radius, v - radius, u + radius, v + radius), outline=color, width=3)
        draw.line((u - radius - 3, v, u + radius + 3, v), fill=color, width=2)
        draw.line((u, v - radius - 3, u, v + radius + 3), fill=color, width=2)

    marker(surface_uv, (255, 80, 50, 255), 6)
    if center_uv is not None:
        marker(center_uv, (255, 210, 0, 255), 9)

    image.convert("RGB").save(output_path)


def localize_run(input_path: Path, output_dir: Path | None, args: argparse.Namespace) -> dict[str, Any]:
    run_dir = _resolve_input_run(input_path)
    temporal_summary = _ensure_temporal_summary(run_dir)
    confirmed_frame_dir = _confirmed_frame_dir(temporal_summary)
    if output_dir is None:
        output_dir = run_dir / "seam_localization"
    output_dir.mkdir(parents=True, exist_ok=True)

    seam_summary = _ensure_seam_summary(confirmed_frame_dir)
    tracker_summary = _ensure_tracker_summary(confirmed_frame_dir)
    metadata = _load_json(confirmed_frame_dir / "metadata.json")
    support = _surface_support_pixels(confirmed_frame_dir, seam_summary, args)

    depth = support["depth"]
    xs = support["xs"]
    ys = support["ys"]
    intrinsics = metadata["camera"]["intrinsics"]["matrix_3x3"]
    surface_points_camera = _backproject(depth, xs, ys, intrinsics)

    surface_center_camera = np.median(surface_points_camera, axis=0)
    surface_center_uv = _project(surface_center_camera.reshape(1, 3), intrinsics)[0]
    pipe_axis_camera = _normalize(np.asarray(tracker_summary["camera_frame_estimate"]["pipe_axis_xyz"], dtype=np.float64))

    pipe_radius_m = float(metadata["scene"]["pipe"]["radius_m"])
    view_direction = _normalize(surface_center_camera)
    radial_direction = view_direction - float(np.dot(view_direction, pipe_axis_camera)) * pipe_axis_camera
    radial_direction = _normalize(radial_direction)
    pipe_center_camera = surface_center_camera + pipe_radius_m * radial_direction
    pipe_center_uv = _project(pipe_center_camera.reshape(1, 3), intrinsics)[0]

    stand_off_m = float(tracker_summary["camera_frame_estimate"]["stand_off_m"])
    lateral_offset_m = float(tracker_summary["camera_frame_estimate"]["lateral_offset_m"])
    yaw_error_deg = float(tracker_summary["camera_frame_estimate"]["yaw_error_deg"])

    gt = metadata["scene"]["ground_truth"].get("seam")
    ground_truth: dict[str, Any] = {"available": gt is not None}
    if gt is not None:
        gt_center_camera = _world_point_to_optical(gt["position_world"], metadata)
        gt_normal_camera = _world_vector_to_optical(gt["normal_world"], metadata)
        if gt_center_camera is not None and gt_normal_camera is not None:
            gt_center = np.asarray(gt_center_camera, dtype=np.float64)
            gt_normal = _normalize(np.asarray(gt_normal_camera, dtype=np.float64))
            surface_error = float(np.linalg.norm(surface_center_camera - gt_center))
            center_error = float(np.linalg.norm(pipe_center_camera - gt_center))
            plane_error = float(np.dot(pipe_center_camera - gt_center, gt_normal))
            normal_error = _angle_between_axes_deg(pipe_axis_camera, gt_normal)
            ground_truth.update(
                {
                    "seam_center_world_xyz_m": gt["position_world"],
                    "seam_normal_world_xyz": gt["normal_world"],
                    "seam_center_camera_xyz_m": gt_center_camera,
                    "seam_normal_camera_xyz": gt_normal_camera,
                    "gap_width_m": _round(float(gt.get("gap_width_m", 0.0))),
                    "visible_surface_to_gt_center_distance_m": _round(surface_error),
                    "pipe_center_to_gt_center_distance_m": _round(center_error),
                    "signed_distance_along_gt_axis_m": _round(plane_error),
                    "plane_normal_angle_error_deg": _round(normal_error),
                }
            )

    selected_uv = np.stack([xs, ys], axis=1)
    rgb_output = output_dir / "rgb_seam_localization_overlay.png"
    depth_output = output_dir / "depth_seam_localization_overlay.png"
    _draw_overlay(
        support["rgb"],
        selected_uv,
        support["candidate_line_original_uv"],
        _round_list(surface_center_uv),
        _round_list(pipe_center_uv),
        rgb_output,
    )
    _draw_overlay(
        _depth_visual(depth),
        selected_uv,
        support["candidate_line_original_uv"],
        _round_list(surface_center_uv),
        _round_list(pipe_center_uv),
        depth_output,
    )

    summary = {
        "input": {
            "run_dir": str(run_dir),
            "temporal_summary_json": str(run_dir / "temporal_confirmation" / "temporal_summary.json"),
            "confirmed_frame_dir": str(confirmed_frame_dir),
            "confirmed_frame_index": int(metadata["frame_index"]),
            "seam_detector_summary_json": str(confirmed_frame_dir / "pipe_strip" / "seam_detector_summary.json"),
            "pipe_tracker_summary_json": str(confirmed_frame_dir / "pipe_tracker" / "pipe_tracker_summary.json"),
        },
        "outputs": {
            "output_dir": str(output_dir),
            "summary_json": str(output_dir / "seam_localization_summary.json"),
            "rgb_seam_localization_overlay_png": str(rgb_output),
            "depth_seam_localization_overlay_png": str(depth_output),
        },
        "confirmed_detection": {
            "candidate_x_strip_px": int(support["candidate_x_strip_px"]),
            "candidate_x_rotated_px": int(support["candidate_x_rotated_px"]),
            "candidate_line_original_uv": [
                _round_list(support["candidate_line_original_uv"][0]),
                _round_list(support["candidate_line_original_uv"][1]),
            ],
            "confidence": _round(float(seam_summary["seam_detection"]["confidence"])),
            "candidate_contrast": _round(float(seam_summary["seam_detection"]["candidate_contrast"])),
            "support_pixel_count": int(xs.size),
            "surface_band_rotated_xyxy": support["surface_band_rotated_xyxy"],
        },
        "camera_frame_seam_pose": {
            "visible_surface_center_xyz_m": _round_list(surface_center_camera),
            "visible_surface_center_uv_px": _round_list(surface_center_uv),
            "pipe_center_estimate_xyz_m": _round_list(pipe_center_camera),
            "pipe_center_estimate_uv_px": _round_list(pipe_center_uv),
            "seam_plane_normal_xyz": _round_list(pipe_axis_camera),
            "pipe_axis_xyz": _round_list(pipe_axis_camera),
            "radial_correction_direction_xyz": _round_list(radial_direction),
            "radius_correction_m": _round(pipe_radius_m),
        },
        "world_frame_seam_pose": {
            "visible_surface_center_world_xyz_m": _optical_point_to_world(surface_center_camera, metadata),
            "pipe_center_estimate_world_xyz_m": _optical_point_to_world(pipe_center_camera, metadata),
            "seam_plane_normal_world_xyz": _optical_vector_to_world(pipe_axis_camera, metadata),
            "camera_pose_source": "metadata.camera.pose.usd_world_pose with optical +Z mapped to USD camera -Z",
        },
        "tracking_estimates": {
            "stand_off_m": _round(stand_off_m),
            "lateral_offset_m": _round(lateral_offset_m),
            "yaw_error_deg": _round(yaw_error_deg),
            "tracker_pipe_centroid_xyz_m": tracker_summary["camera_frame_estimate"]["centroid_xyz_m"],
        },
        "ground_truth_comparison": ground_truth,
        "assumptions": [
            "The confirmed seam candidate is treated as a vertical line in the pipe-aligned strip.",
            "Depth-supported pipe pixels in a narrow band around that line provide the visible seam-surface estimate.",
            "The pipe center estimate is coarse: it offsets the visible surface by the known pipe radius along the camera-view radial direction perpendicular to the pipe axis.",
            "metadata.camera.pose.usd_world_pose uses the USD camera convention where camera -Z is optical forward; the script flips optical Z when transforming to world.",
            "The seam plane normal is approximated by the Phase 2 pipe-axis PCA direction. Its sign is physically ambiguous for this coarse handoff.",
            "This is suitable for the first simulation handoff, not final 3 mm weld metrology.",
        ],
    }

    _write_json(output_dir / "seam_localization_summary.json", summary)
    return summary


def _default_input_paths() -> list[Path]:
    root = _repo_root_from_script() / "logs" / "acea_pipe_junction"
    return [root / DEFAULT_RUN]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Run directory or frame directory. If omitted, seam_3mm_sequence is processed.",
    )
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory for a single input.")
    parser.add_argument(
        "--surface-band-half-width-px",
        type=int,
        default=4,
        help="Half-width of the strip-space seam band used to collect depth-supported pipe pixels.",
    )
    parser.add_argument("--min-surface-points", type=int, default=50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.inputs if args.inputs else _default_input_paths()
    if args.output_dir is not None and len(inputs) != 1:
        raise SystemExit("--output-dir can only be used with a single input")

    for input_path in inputs:
        summary = localize_run(input_path, args.output_dir, args)
        pose = summary["camera_frame_seam_pose"]
        gt = summary["ground_truth_comparison"]
        print(f"[pipe_seam_localizer] {summary['input']['run_dir']}")
        print(
            "  frame={frame} x={x} support={support} center_camera={center}".format(
                frame=summary["input"]["confirmed_frame_index"],
                x=summary["confirmed_detection"]["candidate_x_strip_px"],
                support=summary["confirmed_detection"]["support_pixel_count"],
                center=pose["pipe_center_estimate_xyz_m"],
            )
        )
        if gt.get("available") and "pipe_center_to_gt_center_distance_m" in gt:
            print(
                "  gt_center_error={err:.3f} m signed_axis_error={axis:.3f} m".format(
                    err=gt["pipe_center_to_gt_center_distance_m"],
                    axis=gt["signed_distance_along_gt_axis_m"],
                )
            )
        print(f"  wrote {summary['outputs']['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
