#!/usr/bin/env python3
"""ROS2 RGB-D pipe-junction detector for the ACEA Module 2 demo.

This node is the first online version of the offline ACEA RGB-D pipeline. It
subscribes to RGB, depth, and camera info, then runs a deterministic classical
pipeline:

RGB-D pair -> depth pipe tracker -> pipe-aligned strip -> seam score ->
temporal confirmation -> coarse camera-frame seam estimate.

It publishes JSON on std_msgs/String and RGB debug overlays on sensor_msgs/Image.
It does not use CartesIO, machine learning, custom ROS messages, or robot
commands.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def _stamp_sec(msg: Image | CameraInfo) -> float:
    return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)


def _normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return vector.astype(np.float64)
    return vector.astype(np.float64) / norm


def _round(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def _round_list(values: np.ndarray | list[float] | None, digits: int = 6) -> list[float] | None:
    if values is None:
        return None
    return [_round(float(v), digits) for v in values]


def _pca(points: np.ndarray) -> dict[str, np.ndarray]:
    if points.shape[0] < 3:
        raise ValueError("Need at least three points for PCA")
    centroid = points.mean(axis=0)
    centered = points - centroid
    cov = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    return {
        "centroid": centroid,
        "eigenvalues": eigenvalues,
        "direction": _normalize(eigenvectors[:, 0]),
    }


def _depth_threshold_kmeans(depth: np.ndarray, valid: np.ndarray, sample_stride: int) -> dict[str, float | int]:
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
        "near_center_m": float(centers[0]),
        "far_center_m": float(centers[1]),
        "threshold_m": threshold,
        "sample_count": int(values.size),
    }


def _backproject(depth: np.ndarray, xs: np.ndarray, ys: np.ndarray, k: np.ndarray) -> np.ndarray:
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    z = depth[ys, xs].astype(np.float64)
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def _project(points_camera: np.ndarray, k: np.ndarray) -> np.ndarray:
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    z = np.maximum(points_camera[:, 2], 1e-9)
    u = fx * points_camera[:, 0] / z + cx
    v = fy * points_camera[:, 1] / z + cy
    return np.stack([u, v], axis=1)


def _line_box_segment(
    center_uv: np.ndarray,
    direction_uv: np.ndarray,
    width: int,
    height: int,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
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

    best_pair = (points[0], points[1])
    best_dist = -1.0
    for i, p0 in enumerate(points):
        for p1 in points[i + 1:]:
            dist = float(np.linalg.norm(np.array(p0) - np.array(p1)))
            if dist > best_dist:
                best_dist = dist
                best_pair = (p0, p1)
    return best_pair


def _median_filter_1d(values: np.ndarray, window: int) -> np.ndarray:
    window = max(3, int(window) | 1)
    half = window // 2
    output = np.empty_like(values, dtype=np.float64)
    for i in range(values.size):
        output[i] = float(np.median(values[max(0, i - half):min(values.size, i + half + 1)]))
    return output


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


def _depth_visual(depth: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < 100.0)
    if valid.any():
        lo, hi = np.percentile(depth[valid], [1.0, 99.0])
        image = np.clip((depth - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    else:
        image = np.zeros_like(depth, dtype=np.float32)
    gray = (image * 255.0).astype(np.uint8)
    return np.repeat(gray[:, :, None], 3, axis=2)


@dataclass
class TrackerResult:
    pipe_mask: np.ndarray
    pipe_pixels: int
    pipe_fraction: float
    bbox_uv: list[int]
    image_centroid_uv: np.ndarray
    image_direction_uv: np.ndarray
    image_axis_angle_deg: float
    image_line_segment_uv: tuple[tuple[float, float], tuple[float, float]] | None
    centroid_xyz_m: np.ndarray
    pipe_axis_xyz: np.ndarray
    stand_off_m: float
    lateral_offset_m: float
    vertical_offset_m: float
    yaw_error_deg: float
    threshold_info: dict[str, float | int]


@dataclass
class SeamResult:
    candidate_x_strip_px: int
    candidate_x_rotated_px: int
    candidate_contrast: float
    candidate_z_score: float
    confidence: float
    accepted: bool
    edge_margin_px: int
    crop_xyxy: list[int]
    strip_size_wh: list[int]
    strip_mask: np.ndarray
    rotated_mask: np.ndarray
    rotation_deg: float


@dataclass
class LocalizationResult:
    visible_surface_center_xyz_m: np.ndarray | None
    pipe_center_estimate_xyz_m: np.ndarray | None
    support_pixel_count: int


class OnlinePipeJunctionDetector:
    """Classical RGB-D detector with temporal confirmation state."""

    def __init__(self, params: dict[str, Any]):
        self.params = params
        self.state = "SCAN"
        self.candidate_streak = 0
        self.processed_frame_count = 0
        self.confirmed_frame_count: int | None = None
        self.previous_geometry: dict[str, float | None] | None = None
        self.previous_candidate_x: int | None = None

    def process(self, rgb: np.ndarray, depth: np.ndarray, k: np.ndarray) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
        self.processed_frame_count += 1
        tracker = self._track_pipe(depth, k)
        seam = self._detect_seam(rgb, tracker)
        state_info = self._update_state(tracker, seam)
        localization = None
        if self.state in ("CONFIRMED", "STOP_AND_LOCALIZE"):
            localization = self._localize_confirmed_seam(depth, k, tracker, seam)

        rgb_overlay = self._draw_overlay(rgb, tracker, seam, localization)
        depth_overlay = self._draw_overlay(_depth_visual(depth), tracker, seam, localization)
        status = self._status_dict(tracker, seam, localization, state_info)
        return status, rgb_overlay, depth_overlay

    def _track_pipe(self, depth: np.ndarray, k: np.ndarray) -> TrackerResult:
        min_depth = float(self.params["min_depth_m"])
        max_depth = float(self.params["max_depth_m"])
        valid = np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)
        threshold_info = _depth_threshold_kmeans(depth, valid, int(self.params["sample_stride"]))
        pipe_mask = valid & (depth <= float(threshold_info["threshold_m"]))
        ys, xs = np.nonzero(pipe_mask)
        if xs.size < int(self.params["min_pipe_pixels"]):
            raise ValueError(f"Only {xs.size} pipe pixels selected")

        max_pca_points = int(self.params["max_pca_points"])
        if xs.size > max_pca_points:
            indices = np.linspace(0, xs.size - 1, max_pca_points, dtype=np.int64)
            xs_pca = xs[indices]
            ys_pca = ys[indices]
        else:
            xs_pca = xs
            ys_pca = ys

        points_camera = _backproject(depth, xs_pca, ys_pca, k)
        uv_pca = _pca(np.stack([xs.astype(np.float64), ys.astype(np.float64)], axis=1))
        xyz_pca = _pca(points_camera)

        image_direction = _normalize(uv_pca["direction"])
        if image_direction[0] < 0.0:
            image_direction *= -1.0
        image_axis_angle_deg = math.degrees(math.atan2(float(image_direction[1]), float(image_direction[0])))

        axis_camera = _normalize(xyz_pca["direction"])
        if axis_camera[0] < 0.0:
            axis_camera *= -1.0
        pipe_depths = depth[pipe_mask]
        yaw_error_deg = math.degrees(math.atan2(float(axis_camera[2]), float(axis_camera[0])))
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())

        return TrackerResult(
            pipe_mask=pipe_mask,
            pipe_pixels=int(pipe_mask.sum()),
            pipe_fraction=float(pipe_mask.mean()),
            bbox_uv=[x0, y0, x1, y1],
            image_centroid_uv=uv_pca["centroid"],
            image_direction_uv=image_direction,
            image_axis_angle_deg=image_axis_angle_deg,
            image_line_segment_uv=_line_box_segment(uv_pca["centroid"], image_direction, depth.shape[1], depth.shape[0]),
            centroid_xyz_m=xyz_pca["centroid"],
            pipe_axis_xyz=axis_camera,
            stand_off_m=float(np.median(pipe_depths)),
            lateral_offset_m=float(xyz_pca["centroid"][0]),
            vertical_offset_m=float(xyz_pca["centroid"][1]),
            yaw_error_deg=yaw_error_deg,
            threshold_info=threshold_info,
        )

    def _detect_seam(self, rgb: np.ndarray, tracker: TrackerResult) -> SeamResult:
        rgb_image = PilImage.fromarray(rgb, mode="RGB")
        mask_image = PilImage.fromarray((tracker.pipe_mask.astype(np.uint8) * 255), mode="L")
        angle_deg = tracker.image_axis_angle_deg
        rotated_rgb = rgb_image.rotate(angle_deg, resample=PilImage.Resampling.BICUBIC, expand=False, fillcolor=(0, 0, 0))
        rotated_mask_img = mask_image.rotate(angle_deg, resample=PilImage.Resampling.NEAREST, expand=False, fillcolor=0)
        rotated_mask = np.asarray(rotated_mask_img) > 0

        ys, _ = np.nonzero(rotated_mask)
        if ys.size == 0:
            raise ValueError("Rotated pipe mask is empty")

        width, height = rotated_rgb.size
        vertical_margin = int(self.params["strip_vertical_margin_px"])
        x0, x1 = 0, width - 1
        y0 = max(0, int(ys.min()) - vertical_margin)
        y1 = min(height - 1, int(ys.max()) + vertical_margin)
        strip = rotated_rgb.crop((x0, y0, x1 + 1, y1 + 1))
        strip_mask = rotated_mask[y0:y1 + 1, x0:x1 + 1]

        gray = np.asarray(strip.convert("L"), dtype=np.float64) / 255.0
        profile, counts = self._column_profile(gray, strip_mask)
        background = _median_filter_1d(profile, int(self.params["background_window_px"]))
        residual = background - profile

        strip_width = profile.size
        edge_margin = max(
            int(self.params["edge_margin_px"]),
            int(round(strip_width * float(self.params["edge_margin_fraction"]))),
        )
        min_count = max(8, int(float(self.params["min_valid_column_fraction"]) * strip.height))
        valid = np.isfinite(profile) & (counts >= min_count)
        columns = np.arange(strip_width)
        valid &= columns >= edge_margin
        valid &= columns < strip_width - edge_margin
        if not valid.any():
            raise ValueError("No valid interior strip columns for seam scoring")

        candidate_x = int(np.argmax(np.where(valid, residual, -np.inf)))
        contrast = float(max(0.0, residual[candidate_x]))
        min_dark = float(self.params["min_dark_contrast"])
        strong_dark = float(self.params["strong_dark_contrast"])
        confidence = float(np.clip((contrast - min_dark) / max(strong_dark - min_dark, 1e-6), 0.0, 1.0))
        accepted = confidence >= float(self.params["accept_confidence"])

        interior_residual = residual[valid]
        residual_median = float(np.median(interior_residual))
        residual_mad = float(np.median(np.abs(interior_residual - residual_median)))
        robust_sigma = max(1.4826 * residual_mad, 1.0 / 255.0)
        z_score = (float(residual[candidate_x]) - residual_median) / robust_sigma

        return SeamResult(
            candidate_x_strip_px=candidate_x,
            candidate_x_rotated_px=int(x0 + candidate_x),
            candidate_contrast=contrast,
            candidate_z_score=float(z_score),
            confidence=confidence,
            accepted=accepted,
            edge_margin_px=edge_margin,
            crop_xyxy=[x0, y0, x1, y1],
            strip_size_wh=[strip.width, strip.height],
            strip_mask=strip_mask,
            rotated_mask=rotated_mask,
            rotation_deg=angle_deg,
        )

    def _column_profile(self, gray: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = gray.shape
        profile = np.full(width, np.nan, dtype=np.float64)
        counts = mask.sum(axis=0)
        min_count = max(8, int(float(self.params["min_valid_column_fraction"]) * height))
        for x in range(width):
            if counts[x] >= min_count:
                profile[x] = float(np.median(gray[mask[:, x], x]))

        good = np.isfinite(profile)
        if not good.any():
            raise ValueError("No valid strip columns for seam scoring")
        if not good.all():
            xi = np.arange(width)
            profile[~good] = np.interp(xi[~good], xi[good], profile[good])
        return profile, counts

    def _update_state(self, tracker: TrackerResult, seam: SeamResult) -> dict[str, Any]:
        current_geometry = {
            "pipe_axis_angle_deg": tracker.image_axis_angle_deg,
            "stand_off_m": tracker.stand_off_m,
            "yaw_error_deg": tracker.yaw_error_deg,
        }
        geometry_ok, geometry_failures = self._geometry_consistent(current_geometry)
        candidate_not_border = self._candidate_not_border(seam)
        confidence_ok = seam.confidence >= float(self.params["min_confidence"])
        jump_ok = True
        jump_reason = ""
        if self.previous_candidate_x is not None:
            jump = abs(seam.candidate_x_strip_px - self.previous_candidate_x)
            if jump > int(self.params["max_candidate_jump_px"]):
                jump_ok = False
                jump_reason = f"candidate_jump={jump}px"

        eligible = seam.accepted and confidence_ok and candidate_not_border and geometry_ok and jump_ok

        if self.state in ("CONFIRMED", "STOP_AND_LOCALIZE"):
            self.state = "STOP_AND_LOCALIZE"
        elif eligible:
            self.candidate_streak += 1
            if self.candidate_streak >= int(self.params["min_confirm_frames"]):
                self.state = "CONFIRMED"
                if self.confirmed_frame_count is None:
                    self.confirmed_frame_count = self.processed_frame_count
            else:
                self.state = "CANDIDATE"
        else:
            self.candidate_streak = 0
            self.state = "SCAN"

        if eligible:
            self.previous_geometry = current_geometry
            self.previous_candidate_x = seam.candidate_x_strip_px
        elif bool(self.params["reset_geometry_on_reject"]):
            self.previous_geometry = None
            self.previous_candidate_x = None

        reason_parts = []
        if not seam.accepted:
            reason_parts.append("detector_rejected")
        if not confidence_ok:
            reason_parts.append("low_confidence")
        if not candidate_not_border:
            reason_parts.append("candidate_near_border")
        if not geometry_ok:
            reason_parts.extend(geometry_failures)
        if not jump_ok:
            reason_parts.append(jump_reason)
        if eligible:
            reason_parts.append("eligible")
        if self.state == "STOP_AND_LOCALIZE":
            reason_parts.append("stop_and_localize")

        return {
            "eligible": eligible,
            "candidate_not_border": candidate_not_border,
            "geometry_consistent": geometry_ok,
            "reason": ";".join(reason_parts),
        }

    def _geometry_consistent(self, current: dict[str, float | None]) -> tuple[bool, list[str]]:
        if self.previous_geometry is None:
            return True, []

        failures: list[str] = []
        axis_delta = abs(float(current["pipe_axis_angle_deg"]) - float(self.previous_geometry["pipe_axis_angle_deg"]))
        if axis_delta > float(self.params["max_axis_angle_delta_deg"]):
            failures.append(f"axis_delta={axis_delta:.3f}deg")

        stand_delta = abs(float(current["stand_off_m"]) - float(self.previous_geometry["stand_off_m"]))
        if stand_delta > float(self.params["max_stand_off_delta_m"]):
            failures.append(f"stand_off_delta={stand_delta:.3f}m")

        yaw_delta = abs(float(current["yaw_error_deg"]) - float(self.previous_geometry["yaw_error_deg"]))
        if yaw_delta > float(self.params["max_yaw_delta_deg"]):
            failures.append(f"yaw_delta={yaw_delta:.3f}deg")

        return not failures, failures

    def _candidate_not_border(self, seam: SeamResult) -> bool:
        width = int(seam.strip_size_wh[0])
        x = int(seam.candidate_x_strip_px)
        return seam.edge_margin_px <= x < width - seam.edge_margin_px

    def _localize_confirmed_seam(
        self,
        depth: np.ndarray,
        k: np.ndarray,
        tracker: TrackerResult,
        seam: SeamResult,
    ) -> LocalizationResult:
        x_center = int(seam.candidate_x_rotated_px)
        y0, y1 = int(seam.crop_xyxy[1]), int(seam.crop_xyxy[3])
        half_width = int(self.params["surface_band_half_width_px"])
        x0 = max(0, x_center - half_width)
        x1 = min(depth.shape[1] - 1, x_center + half_width)
        y0 = max(0, y0)
        y1 = min(depth.shape[0] - 1, y1)

        band_mask = np.zeros(seam.rotated_mask.shape, dtype=bool)
        band_mask[y0:y1 + 1, x0:x1 + 1] = seam.rotated_mask[y0:y1 + 1, x0:x1 + 1]
        ys_rot, xs_rot = np.nonzero(band_mask)
        if xs_rot.size == 0:
            return LocalizationResult(None, None, 0)

        rotated_uv = np.stack([xs_rot.astype(np.float64), ys_rot.astype(np.float64)], axis=1)
        original_uv = _inverse_rotate_uv(rotated_uv, (depth.shape[1], depth.shape[0]), seam.rotation_deg)
        xs = np.rint(original_uv[:, 0]).astype(np.int64)
        ys = np.rint(original_uv[:, 1]).astype(np.int64)
        in_bounds = (xs >= 0) & (xs < depth.shape[1]) & (ys >= 0) & (ys < depth.shape[0])
        xs = xs[in_bounds]
        ys = ys[in_bounds]
        valid = tracker.pipe_mask[ys, xs] & np.isfinite(depth[ys, xs]) & (depth[ys, xs] > 0.0)
        xs = xs[valid]
        ys = ys[valid]
        if xs.size < int(self.params["min_surface_points"]):
            return LocalizationResult(None, None, int(xs.size))

        unique_uv = np.unique(np.stack([xs, ys], axis=1), axis=0)
        points_camera = _backproject(depth, unique_uv[:, 0], unique_uv[:, 1], k)
        surface_center = np.median(points_camera, axis=0)

        view_direction = _normalize(surface_center)
        axis = _normalize(tracker.pipe_axis_xyz)
        radial_direction = view_direction - float(np.dot(view_direction, axis)) * axis
        radial_direction = _normalize(radial_direction)
        pipe_center = surface_center + float(self.params["pipe_radius_m"]) * radial_direction
        return LocalizationResult(surface_center, pipe_center, int(unique_uv.shape[0]))

    def _status_dict(
        self,
        tracker: TrackerResult,
        seam: SeamResult,
        localization: LocalizationResult | None,
        state_info: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "state": self.state,
            "processed_frame_count": self.processed_frame_count,
            "confirmed_frame_count": self.confirmed_frame_count,
            "candidate_streak": self.candidate_streak,
            "confidence": _round(seam.confidence),
            "candidate_x_strip_px": int(seam.candidate_x_strip_px),
            "candidate_contrast": _round(seam.candidate_contrast),
            "candidate_z_score": _round(seam.candidate_z_score),
            "detector_accepted": bool(seam.accepted),
            "eligible": bool(state_info["eligible"]),
            "candidate_not_border": bool(state_info["candidate_not_border"]),
            "geometry_consistent": bool(state_info["geometry_consistent"]),
            "reason": state_info["reason"],
            "stand_off_m": _round(tracker.stand_off_m),
            "lateral_offset_m": _round(tracker.lateral_offset_m),
            "vertical_offset_m": _round(tracker.vertical_offset_m),
            "yaw_error_deg": _round(tracker.yaw_error_deg),
            "image_pipe_axis_angle_deg": _round(tracker.image_axis_angle_deg),
            "pipe_pixels": int(tracker.pipe_pixels),
            "pipe_fraction_of_image": _round(tracker.pipe_fraction),
            "pipe_axis_camera_xyz": _round_list(tracker.pipe_axis_xyz),
            "pipe_centroid_camera_xyz_m": _round_list(tracker.centroid_xyz_m),
            "coarse_seam_visible_surface_camera_xyz_m": None
            if localization is None
            else _round_list(localization.visible_surface_center_xyz_m),
            "coarse_seam_center_camera_xyz_m": None
            if localization is None
            else _round_list(localization.pipe_center_estimate_xyz_m),
            "coarse_seam_support_pixel_count": None if localization is None else localization.support_pixel_count,
        }

    def _draw_overlay(
        self,
        rgb: np.ndarray,
        tracker: TrackerResult,
        seam: SeamResult,
        localization: LocalizationResult | None,
    ) -> np.ndarray:
        image = PilImage.fromarray(rgb, mode="RGB").convert("RGBA")
        overlay = np.zeros((image.height, image.width, 4), dtype=np.uint8)
        overlay[tracker.pipe_mask] = (20, 190, 90, 70)
        image = PilImage.alpha_composite(image, PilImage.fromarray(overlay, mode="RGBA"))
        draw = ImageDraw.Draw(image)

        if tracker.image_line_segment_uv is not None:
            p0, p1 = tracker.image_line_segment_uv
            draw.line((p0[0], p0[1], p1[0], p1[1]), fill=(255, 40, 40, 255), width=3)

        u, v = tracker.image_centroid_uv
        draw.ellipse((u - 5, v - 5, u + 5, v + 5), outline=(40, 120, 255, 255), width=3)

        line_uv = self._candidate_line_original_uv(seam)
        color = (0, 220, 255, 255) if seam.accepted else (255, 180, 0, 255)
        draw.line((line_uv[0][0], line_uv[0][1], line_uv[1][0], line_uv[1][1]), fill=color, width=2)

        if localization is not None and localization.pipe_center_estimate_xyz_m is not None:
            k = np.array(
                [
                    [float(self.params["last_fx"]), 0.0, float(self.params["last_cx"])],
                    [0.0, float(self.params["last_fy"]), float(self.params["last_cy"])],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            uv = _project(localization.pipe_center_estimate_xyz_m.reshape(1, 3), k)[0]
            draw.ellipse((uv[0] - 8, uv[1] - 8, uv[0] + 8, uv[1] + 8), outline=(255, 210, 0, 255), width=3)

        return np.asarray(image.convert("RGB"))

    def _candidate_line_original_uv(self, seam: SeamResult) -> tuple[tuple[float, float], tuple[float, float]]:
        y0, y1 = float(seam.crop_xyxy[1]), float(seam.crop_xyxy[3])
        x = float(seam.candidate_x_rotated_px)
        points_rot = np.array([[x, y0], [x, y1]], dtype=np.float64)
        original = _inverse_rotate_uv(points_rot, (seam.rotated_mask.shape[1], seam.rotated_mask.shape[0]), seam.rotation_deg)
        return (float(original[0, 0]), float(original[0, 1])), (float(original[1, 0]), float(original[1, 1]))


class AceaPipeJunctionNode(Node):
    def __init__(self) -> None:
        super().__init__("acea_pipe_junction_detector")
        self.params = self._declare_params()
        self.detector = OnlinePipeJunctionDetector(self.params)
        self.rgb_queue: deque[tuple[float, Image]] = deque(maxlen=int(self.params["queue_size"]))
        self.depth_queue: deque[tuple[float, Image]] = deque(maxlen=int(self.params["queue_size"]))
        self.info_queue: deque[tuple[float, CameraInfo]] = deque(maxlen=int(self.params["queue_size"]))
        self.received_count = 0
        self.last_processed_rgb_time: float | None = None
        self.last_status_publish_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.params["queue_size"]),
        )
        self.create_subscription(Image, str(self.params["rgb_topic"]), self._rgb_cb, qos)
        self.create_subscription(Image, str(self.params["depth_topic"]), self._depth_cb, qos)
        self.create_subscription(CameraInfo, str(self.params["camera_info_topic"]), self._info_cb, qos)

        self.detection_pub = self.create_publisher(String, str(self.params["detection_topic"]), 10)
        self.rgb_overlay_pub = self.create_publisher(Image, str(self.params["rgb_overlay_topic"]), 10)
        self.depth_overlay_pub = self.create_publisher(Image, str(self.params["depth_overlay_topic"]), 10)
        if bool(self.params["publish_waiting_status"]):
            self.create_timer(float(self.params["waiting_status_period_s"]), self._publish_waiting_status)
        self.get_logger().info(
            "ACEA pipe-junction detector listening to "
            f"{self.params['rgb_topic']}, {self.params['depth_topic']}, {self.params['camera_info_topic']}"
        )

    def _declare_params(self) -> dict[str, Any]:
        declarations = {
            "rgb_topic": "/camera/rgb",
            "depth_topic": "/camera/depth",
            "camera_info_topic": "/camera/camera_info",
            "detection_topic": "/acea/pipe_junction/detection",
            "rgb_overlay_topic": "/acea/pipe_junction/debug/rgb_overlay",
            "depth_overlay_topic": "/acea/pipe_junction/debug/depth_overlay",
            "sync_slop_s": 0.08,
            "allow_stale_camera_info": True,
            "queue_size": 10,
            "publish_waiting_status": True,
            "waiting_status_period_s": 1.0,
            "process_every_n": 1,
            "min_depth_m": 0.05,
            "max_depth_m": 20.0,
            "sample_stride": 4,
            "max_pca_points": 60000,
            "min_pipe_pixels": 1000,
            "strip_vertical_margin_px": 6,
            "min_valid_column_fraction": 0.25,
            "background_window_px": 41,
            "edge_margin_fraction": 0.06,
            "edge_margin_px": 20,
            "min_dark_contrast": 0.015,
            "strong_dark_contrast": 0.06,
            "accept_confidence": 0.35,
            "min_confidence": 0.35,
            "min_confirm_frames": 2,
            "max_axis_angle_delta_deg": 3.0,
            "max_stand_off_delta_m": 0.15,
            "max_yaw_delta_deg": 5.0,
            "max_candidate_jump_px": 120,
            "reset_geometry_on_reject": False,
            "pipe_radius_m": 0.45,
            "surface_band_half_width_px": 4,
            "min_surface_points": 50,
            "last_fx": 1.0,
            "last_fy": 1.0,
            "last_cx": 0.0,
            "last_cy": 0.0,
        }
        values: dict[str, Any] = {}
        for name, default in declarations.items():
            self.declare_parameter(name, default)
            values[name] = self.get_parameter(name).value
        return values

    def _rgb_cb(self, msg: Image) -> None:
        self.rgb_queue.append((self._message_time(msg), msg))
        self._try_process()

    def _depth_cb(self, msg: Image) -> None:
        self.depth_queue.append((self._message_time(msg), msg))
        self._try_process()

    def _info_cb(self, msg: CameraInfo) -> None:
        self.info_queue.append((self._message_time(msg), msg))
        self._try_process()

    def _message_time(self, msg: Image | CameraInfo) -> float:
        stamp = _stamp_sec(msg)
        if stamp > 0.0:
            return stamp
        return 1e-9 * float(self.get_clock().now().nanoseconds)

    def _try_process(self) -> None:
        if not self.rgb_queue or not self.depth_queue or not self.info_queue:
            return

        rgb_time, rgb_msg = self.rgb_queue[-1]
        depth_pair = self._closest(self.depth_queue, rgb_time)
        info_pair = self.info_queue[-1] if bool(self.params["allow_stale_camera_info"]) else self._closest(self.info_queue, rgb_time)
        if depth_pair is None or info_pair is None:
            return

        depth_time, depth_msg = depth_pair
        info_time, info_msg = info_pair
        slop = float(self.params["sync_slop_s"])
        if abs(depth_time - rgb_time) > slop:
            return
        if not bool(self.params["allow_stale_camera_info"]) and abs(info_time - rgb_time) > slop:
            return
        if self.last_processed_rgb_time is not None and abs(rgb_time - self.last_processed_rgb_time) < 1e-12:
            return
        self.last_processed_rgb_time = rgb_time

        self.received_count += 1
        every_n = max(1, int(self.params["process_every_n"]))
        if self.received_count % every_n != 0:
            return

        try:
            rgb = self._image_to_rgb_array(rgb_msg)
            depth = self._image_to_depth_m(depth_msg)
            k = np.asarray(info_msg.k, dtype=np.float64).reshape(3, 3)
            self.params["last_fx"] = float(k[0, 0])
            self.params["last_fy"] = float(k[1, 1])
            self.params["last_cx"] = float(k[0, 2])
            self.params["last_cy"] = float(k[1, 2])
            status, rgb_overlay, depth_overlay = self.detector.process(rgb, depth, k)
            status.update(
                {
                    "stamp": {"sec": int(rgb_msg.header.stamp.sec), "nanosec": int(rgb_msg.header.stamp.nanosec)},
                    "frame_id": rgb_msg.header.frame_id,
                    "rgb_depth_dt_s": _round(depth_time - rgb_time),
                    "rgb_info_dt_s": _round(info_time - rgb_time),
                }
            )
            self._publish(status, rgb_overlay, depth_overlay, rgb_msg.header)
        except Exception as exc:
            status = {
                "state": "ERROR",
                "error": f"{type(exc).__name__}: {exc}",
                "processed_frame_count": self.detector.processed_frame_count,
            }
            self.detection_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            self.last_status_publish_time = self._now_sec()
            self.get_logger().warn(status["error"], throttle_duration_sec=2.0)

    @staticmethod
    def _closest(queue: deque[tuple[float, Any]], target: float) -> tuple[float, Any] | None:
        if not queue:
            return None
        return min(queue, key=lambda pair: abs(pair[0] - target))

    def _publish(self, status: dict[str, Any], rgb_overlay: np.ndarray, depth_overlay: np.ndarray, header: Any) -> None:
        self.detection_pub.publish(String(data=json.dumps(status, sort_keys=True, allow_nan=False)))
        self.last_status_publish_time = self._now_sec()
        self.rgb_overlay_pub.publish(self._rgb_array_to_msg(rgb_overlay, header))
        self.depth_overlay_pub.publish(self._rgb_array_to_msg(depth_overlay, header))

    def _publish_waiting_status(self) -> None:
        now = self._now_sec()
        if now - self.last_status_publish_time < 0.8 * float(self.params["waiting_status_period_s"]):
            return

        rgb_time = self.rgb_queue[-1][0] if self.rgb_queue else None
        depth_pair = self._closest(self.depth_queue, rgb_time) if rgb_time is not None else None
        info_pair = self.info_queue[-1] if self.info_queue else None
        depth_dt = None if depth_pair is None or rgb_time is None else depth_pair[0] - rgb_time
        info_dt = None if info_pair is None or rgb_time is None else info_pair[0] - rgb_time

        if not self.rgb_queue:
            reason = "waiting_for_rgb"
        elif not self.depth_queue:
            reason = "waiting_for_depth"
        elif not self.info_queue:
            reason = "waiting_for_camera_info"
        elif depth_dt is not None and abs(depth_dt) > float(self.params["sync_slop_s"]):
            reason = "waiting_for_rgb_depth_sync"
        else:
            reason = "waiting_for_next_frame"

        status = {
            "state": "WAITING_FOR_SYNC",
            "reason": reason,
            "rgb_queue": len(self.rgb_queue),
            "depth_queue": len(self.depth_queue),
            "camera_info_queue": len(self.info_queue),
            "processed_frame_count": self.detector.processed_frame_count,
            "received_synced_candidate_count": self.received_count,
            "sync_slop_s": _round(float(self.params["sync_slop_s"])),
            "latest_rgb_depth_dt_s": _round(depth_dt),
            "latest_rgb_info_dt_s": _round(info_dt),
            "allow_stale_camera_info": bool(self.params["allow_stale_camera_info"]),
        }
        self.detection_pub.publish(String(data=json.dumps(status, sort_keys=True, allow_nan=False)))
        self.last_status_publish_time = now

    def _now_sec(self) -> float:
        return 1e-9 * float(self.get_clock().now().nanoseconds)

    @staticmethod
    def _image_to_rgb_array(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        height, width = int(msg.height), int(msg.width)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ("rgb8", "bgr8"):
            channels = 3
            rows = raw.reshape(height, int(msg.step))
            array = rows[:, :width * channels].reshape(height, width, channels).copy()
            if encoding == "bgr8":
                array = array[:, :, ::-1].copy()
            return array
        if encoding in ("rgba8", "bgra8"):
            channels = 4
            rows = raw.reshape(height, int(msg.step))
            array = rows[:, :width * channels].reshape(height, width, channels).copy()
            if encoding == "bgra8":
                array = array[:, :, [2, 1, 0, 3]]
            return array[:, :, :3].copy()
        if encoding in ("mono8", "8uc1"):
            rows = raw.reshape(height, int(msg.step))
            gray = rows[:, :width].copy()
            return np.repeat(gray[:, :, None], 3, axis=2)
        raise ValueError(f"Unsupported RGB image encoding: {msg.encoding}")

    @staticmethod
    def _image_to_depth_m(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        height, width = int(msg.height), int(msg.width)
        if encoding in ("32fc1", "passthrough"):
            raw = np.frombuffer(msg.data, dtype=np.float32)
            row_floats = int(msg.step) // np.dtype(np.float32).itemsize
            return raw.reshape(height, row_floats)[:, :width].astype(np.float32).copy()
        if encoding == "16uc1":
            raw = np.frombuffer(msg.data, dtype=np.uint16)
            row_values = int(msg.step) // np.dtype(np.uint16).itemsize
            return (raw.reshape(height, row_values)[:, :width].astype(np.float32) / 1000.0).copy()
        raise ValueError(f"Unsupported depth image encoding: {msg.encoding}")

    @staticmethod
    def _rgb_array_to_msg(array: np.ndarray, header: Any) -> Image:
        output = Image()
        output.header = header
        output.height = int(array.shape[0])
        output.width = int(array.shape[1])
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = int(array.shape[1] * 3)
        output.data = np.ascontiguousarray(array.astype(np.uint8)).tobytes()
        return output


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AceaPipeJunctionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
