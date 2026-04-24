#!/usr/bin/env python3
"""Offline depth-based pipe tracker for ACEA pipe-junction RGB-D logs.

This is Phase 2 of the ACEA Module 2 perception demo. It is intentionally
classical and deterministic: foreground depth selection, backprojection, and
PCA. It does not depend on ROS, CartesIO, or machine learning.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw


DEFAULT_RUNS = (
    "seam_3mm_smoke",
    "seam_3mm_centered",
    "no_seam_smoke",
)


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
        return vector.astype(float)
    return vector.astype(float) / norm


def _resolve_frame_dir(input_path: Path) -> Path:
    path = input_path.expanduser().resolve()
    if (path / "metadata.json").exists():
        return path

    manifest = path / "manifest.jsonl"
    if manifest.exists():
        for line in manifest.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            metadata_path = Path(record["metadata"])
            if not metadata_path.exists() and str(metadata_path).startswith("/workspace/iit-concert-ros-pkg/"):
                rel = Path(str(metadata_path).replace("/workspace/iit-concert-ros-pkg/", "", 1))
                metadata_path = _repo_root_from_script() / rel
            if metadata_path.exists():
                return metadata_path.parent

    frame_dirs = sorted(path.glob("frame_*"))
    for frame_dir in frame_dirs:
        if (frame_dir / "metadata.json").exists():
            return frame_dir

    raise FileNotFoundError(f"Could not find a logged frame under {input_path}")


def _depth_threshold_kmeans(depth: np.ndarray, valid: np.ndarray, sample_stride: int) -> dict[str, Any]:
    sample = depth[::sample_stride, ::sample_stride]
    sample_valid = valid[::sample_stride, ::sample_stride]
    values = sample[sample_valid].astype(np.float64)
    if values.size < 32:
        raise ValueError("Not enough valid depth pixels for foreground selection")

    centers = np.percentile(values, [10.0, 90.0]).astype(np.float64)
    for _ in range(32):
        threshold = float(0.5 * (centers[0] + centers[1]))
        low = values[values <= threshold]
        high = values[values > threshold]
        if low.size == 0 or high.size == 0:
            break
        new_centers = np.array([low.mean(), high.mean()], dtype=np.float64)
        if np.linalg.norm(new_centers - centers) < 1e-9:
            centers = new_centers
            break
        centers = new_centers

    centers = np.sort(centers)
    threshold = float(0.5 * (centers[0] + centers[1]))
    return {
        "near_center_m": _round(centers[0]),
        "far_center_m": _round(centers[1]),
        "threshold_m": _round(threshold),
        "sample_count": int(values.size),
    }


def _largest_connected_component(mask: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    visited = np.zeros(mask.shape, dtype=bool)
    best_component: list[tuple[int, int]] = []

    for start_y, start_x in np.argwhere(mask):
        start = (int(start_y), int(start_x))
        if visited[start]:
            continue

        stack = [start]
        visited[start] = True
        component: list[tuple[int, int]] = []

        while stack:
            y, x = stack.pop()
            component.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if ny < 0 or ny >= height or nx < 0 or nx >= width:
                    continue
                if visited[ny, nx] or not mask[ny, nx]:
                    continue
                visited[ny, nx] = True
                stack.append((ny, nx))

        if len(component) > len(best_component):
            best_component = component

    clean = np.zeros(mask.shape, dtype=bool)
    if best_component:
        ys, xs = zip(*best_component)
        clean[np.array(ys), np.array(xs)] = True
    return clean


def _pca(points: np.ndarray) -> dict[str, Any]:
    if points.shape[0] < 3:
        raise ValueError("Need at least three points for PCA")
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    direction = _normalize(eigenvectors[:, 0])
    return {
        "centroid": centroid,
        "eigenvalues": eigenvalues,
        "eigenvectors": eigenvectors,
        "direction": direction,
    }


def _line_box_segment(center_uv: np.ndarray, direction_uv: np.ndarray, width: int, height: int):
    cx, cy = center_uv
    dx, dy = direction_uv
    points: list[tuple[float, float]] = []

    if abs(dx) > 1e-9:
        for x in (0.0, float(width - 1)):
            t = (x - cx) / dx
            y = cy + t * dy
            if 0.0 <= y <= height - 1:
                points.append((x, y))

    if abs(dy) > 1e-9:
        for y in (0.0, float(height - 1)):
            t = (y - cy) / dy
            x = cx + t * dx
            if 0.0 <= x <= width - 1:
                points.append((x, y))

    if len(points) < 2:
        return None

    unique: list[tuple[float, float]] = []
    for point in points:
        if not any(np.linalg.norm(np.array(point) - np.array(existing)) < 1.0 for existing in unique):
            unique.append(point)

    if len(unique) < 2:
        return None

    best_pair = (unique[0], unique[1])
    best_dist = -1.0
    for i, p0 in enumerate(unique):
        for p1 in unique[i + 1:]:
            dist = float(np.linalg.norm(np.array(p0) - np.array(p1)))
            if dist > best_dist:
                best_dist = dist
                best_pair = (p0, p1)
    return best_pair


def _backproject(depth: np.ndarray, xs: np.ndarray, ys: np.ndarray, intrinsics: list[list[float]]) -> np.ndarray:
    k = np.asarray(intrinsics, dtype=np.float64)
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    z = depth[ys, xs].astype(np.float64)
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    return np.stack([x, y, z], axis=1)


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


def _gt_axis_camera(metadata: dict[str, Any]) -> list[float] | None:
    try:
        axis_world = np.asarray(metadata["scene"]["pipe"]["axis_world"], dtype=np.float64)
        quat = metadata["camera"]["pose"]["usd_world_pose"]["quaternion_wxyz_world"]
        rot_world_from_camera = _quat_wxyz_to_rot(quat)
        axis_camera = rot_world_from_camera.T @ axis_world
        return _round_list(_normalize(axis_camera))
    except Exception:
        return None


def _axis_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    a = _normalize(a)
    b = _normalize(b)
    dot = float(np.clip(abs(np.dot(a, b)), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _depth_visual(depth: np.ndarray) -> Image.Image:
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < 100.0)
    if valid.any():
        lo, hi = np.percentile(depth[valid], [1.0, 99.0])
        image = np.clip((depth - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        image = np.zeros_like(depth, dtype=np.float32)
    return Image.fromarray((image * 255.0).astype(np.uint8)).convert("RGBA")


def _draw_overlay(base: Image.Image, mask: np.ndarray, line_segment, centroid_uv, bbox, output_path: Path) -> None:
    image = base.convert("RGBA")
    overlay_array = np.zeros((image.height, image.width, 4), dtype=np.uint8)
    overlay_array[mask] = (20, 190, 90, 70)
    overlay = Image.fromarray(overlay_array, mode="RGBA")
    image = Image.alpha_composite(image, overlay)

    draw = ImageDraw.Draw(image)
    if bbox is not None:
        x0, y0, x1, y1 = bbox
        draw.rectangle((x0, y0, x1, y1), outline=(255, 210, 0, 255), width=2)
    if line_segment is not None:
        p0, p1 = line_segment
        draw.line((p0[0], p0[1], p1[0], p1[1]), fill=(255, 40, 40, 255), width=3)
    if centroid_uv is not None:
        u, v = centroid_uv
        r = 5
        draw.ellipse((u - r, v - r, u + r, v + r), outline=(40, 120, 255, 255), width=3)

    image.convert("RGB").save(output_path)


def process_frame(frame_dir: Path, output_dir: Path | None, args: argparse.Namespace) -> dict[str, Any]:
    frame_dir = _resolve_frame_dir(frame_dir)
    metadata_path = frame_dir / "metadata.json"
    rgb_path = frame_dir / "rgb.png"
    depth_path = frame_dir / "depth_m.npy"
    if not metadata_path.exists() or not rgb_path.exists() or not depth_path.exists():
        raise FileNotFoundError(f"Missing rgb/depth/metadata files in {frame_dir}")

    metadata = _load_json(metadata_path)
    rgb = Image.open(rgb_path).convert("RGB")
    depth = np.load(depth_path)
    height, width = depth.shape

    valid = np.isfinite(depth) & (depth > args.min_depth_m) & (depth < args.max_depth_m)
    threshold_info = _depth_threshold_kmeans(depth, valid, args.sample_stride)
    pipe_mask = valid & (depth <= float(threshold_info["threshold_m"]))
    if args.keep_largest_component:
        pipe_mask = _largest_connected_component(pipe_mask)

    ys, xs = np.nonzero(pipe_mask)
    if xs.size < args.min_pipe_pixels:
        raise ValueError(f"Only {xs.size} pipe pixels selected; check depth thresholding")

    if xs.size > args.max_pca_points:
        indices = np.linspace(0, xs.size - 1, args.max_pca_points, dtype=np.int64)
        xs_pca = xs[indices]
        ys_pca = ys[indices]
    else:
        xs_pca = xs
        ys_pca = ys

    intrinsics = metadata["camera"]["intrinsics"]["matrix_3x3"]
    points_camera = _backproject(depth, xs_pca, ys_pca, intrinsics)

    uv_points = np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1)
    uv_pca = _pca(uv_points)
    xyz_pca = _pca(points_camera)

    image_direction = _normalize(uv_pca["direction"])
    if image_direction[0] < 0.0:
        image_direction *= -1.0
    image_centroid = uv_pca["centroid"]
    line_segment = _line_box_segment(image_centroid, image_direction, width, height)
    image_axis_angle_deg = math.degrees(math.atan2(float(image_direction[1]), float(image_direction[0])))

    axis_camera = _normalize(xyz_pca["direction"])
    if axis_camera[0] < 0.0:
        axis_camera *= -1.0
    centroid_camera = xyz_pca["centroid"]
    yaw_error_deg = math.degrees(math.atan2(float(axis_camera[2]), float(axis_camera[0])))

    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    bbox = (x0, y0, x1, y1)

    if output_dir is None:
        output_dir = frame_dir / "pipe_tracker"
    output_dir.mkdir(parents=True, exist_ok=True)

    _draw_overlay(rgb.convert("RGBA"), pipe_mask, line_segment, image_centroid, bbox, output_dir / "rgb_pipe_overlay.png")
    _draw_overlay(_depth_visual(depth), pipe_mask, line_segment, image_centroid, bbox, output_dir / "depth_pipe_overlay.png")
    Image.fromarray((pipe_mask.astype(np.uint8) * 255), mode="L").save(output_dir / "pipe_mask.png")

    gt_axis_cam = _gt_axis_camera(metadata)
    axis_angle_error = None
    if gt_axis_cam is not None:
        axis_angle_error = _axis_angle_deg(axis_camera, np.asarray(gt_axis_cam, dtype=np.float64))

    pipe_depths = depth[pipe_mask]
    summary = {
        "input": {
            "frame_dir": str(frame_dir),
            "rgb_png": str(rgb_path),
            "depth_npy": str(depth_path),
            "metadata_json": str(metadata_path),
            "scene_mode": metadata["scene"]["scene_mode"],
            "seam_present": bool(metadata["scene"]["seam_present"]),
        },
        "outputs": {
            "output_dir": str(output_dir),
            "rgb_pipe_overlay_png": str(output_dir / "rgb_pipe_overlay.png"),
            "depth_pipe_overlay_png": str(output_dir / "depth_pipe_overlay.png"),
            "pipe_mask_png": str(output_dir / "pipe_mask.png"),
            "summary_json": str(output_dir / "pipe_tracker_summary.json"),
        },
        "depth_selection": {
            "valid_pixels": int(valid.sum()),
            "pipe_pixels": int(pipe_mask.sum()),
            "pipe_fraction_of_image": _round(float(pipe_mask.mean())),
            "bbox_uv": [int(x0), int(y0), int(x1), int(y1)],
            "kmeans": threshold_info,
            "used_largest_connected_component": bool(args.keep_largest_component),
        },
        "image_pipe_axis": {
            "centroid_uv": _round_list(image_centroid),
            "direction_uv": _round_list(image_direction),
            "angle_deg": _round(image_axis_angle_deg),
            "line_segment_uv": None
            if line_segment is None
            else [[_round(v) for v in line_segment[0]], [_round(v) for v in line_segment[1]]],
        },
        "camera_frame_estimate": {
            "point_count_used_for_pca": int(points_camera.shape[0]),
            "centroid_xyz_m": _round_list(centroid_camera),
            "pipe_axis_xyz": _round_list(axis_camera),
            "pca_eigenvalues": _round_list(xyz_pca["eigenvalues"]),
            "nearest_surface_depth_m": _round(float(np.percentile(pipe_depths, 1.0))),
            "median_surface_depth_m": _round(float(np.median(pipe_depths))),
            "centroid_range_m": _round(float(np.linalg.norm(centroid_camera))),
            "stand_off_m": _round(float(np.median(pipe_depths))),
            "lateral_offset_m": _round(float(centroid_camera[0])),
            "vertical_offset_m": _round(float(centroid_camera[1])),
            "yaw_error_deg": _round(yaw_error_deg),
        },
        "ground_truth_reference": {
            "pipe_axis_world": metadata["scene"]["pipe"]["axis_world"],
            "pipe_axis_camera_from_usd_pose": gt_axis_cam,
            "axis_angle_error_deg": None if axis_angle_error is None else _round(axis_angle_error),
            "seam": metadata["scene"]["ground_truth"].get("seam"),
        },
        "assumptions": [
            "Depth is treated as distance along the camera optical axis and backprojected with the logged pinhole intrinsics.",
            "The pipe is assumed to be the nearest dominant foreground surface in the RGB-D frame.",
            "The 3D pipe axis is the first PCA component of selected foreground depth points.",
            "lateral_offset_m is the selected pipe centroid X coordinate in the camera frame, not yet a base-frame control error.",
            "yaw_error_deg is the pipe-axis projection angle relative to the camera horizontal X axis; TF/odometry fusion comes later.",
        ],
    }
    _write_json(output_dir / "pipe_tracker_summary.json", summary)
    return summary


def _default_input_paths() -> list[Path]:
    root = _repo_root_from_script() / "logs" / "acea_pipe_junction"
    return [root / name for name in DEFAULT_RUNS]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Run directory or frame directory. If omitted, the three smoke runs are processed.",
    )
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory for a single input.")
    parser.add_argument("--min-depth-m", type=float, default=0.05)
    parser.add_argument("--max-depth-m", type=float, default=20.0)
    parser.add_argument("--sample-stride", type=int, default=4, help="Spatial stride for deterministic depth k-means.")
    parser.add_argument("--max-pca-points", type=int, default=60000)
    parser.add_argument("--min-pipe-pixels", type=int, default=1000)
    parser.add_argument(
        "--keep-largest-component",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Keep only the largest connected foreground component after depth thresholding.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.inputs if args.inputs else _default_input_paths()
    if args.output_dir is not None and len(inputs) != 1:
        raise SystemExit("--output-dir can only be used with a single input")

    for input_path in inputs:
        summary = process_frame(input_path, args.output_dir, args)
        estimate = summary["camera_frame_estimate"]
        image_axis = summary["image_pipe_axis"]
        print(f"[pipe_depth_tracker] {summary['input']['frame_dir']}")
        print(
            "  pipe_pixels={pipe_pixels} image_axis={angle:.2f} deg "
            "stand_off={stand:.3f} m lateral={lat:.3f} m yaw={yaw:.2f} deg".format(
                pipe_pixels=summary["depth_selection"]["pipe_pixels"],
                angle=image_axis["angle_deg"],
                stand=estimate["stand_off_m"],
                lat=estimate["lateral_offset_m"],
                yaw=estimate["yaw_error_deg"],
            )
        )
        print(f"  wrote {summary['outputs']['output_dir']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
