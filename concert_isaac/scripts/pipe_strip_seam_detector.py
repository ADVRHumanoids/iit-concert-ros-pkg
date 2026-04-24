#!/usr/bin/env python3
"""Pipe-aligned strip extraction and classical seam scoring for ACEA logs.

This is Phase 3 of the ACEA Module 2 perception demo. It consumes logged RGB-D
frames plus the Phase 2 depth-tracker output, normalizes the visible pipe into a
horizontal strip, and searches for a thin dark vertical seam with a classical
column score. It does not use ROS, CartesIO, lidar, or machine learning.
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


def _ensure_tracker_summary(frame_dir: Path) -> dict[str, Any]:
    tracker_dir = frame_dir / "pipe_tracker"
    summary_path = tracker_dir / "pipe_tracker_summary.json"
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


def _resolve_frame_dir(input_path: Path) -> Path:
    return pipe_depth_tracker._resolve_frame_dir(input_path)


def _default_input_paths() -> list[Path]:
    root = _repo_root_from_script() / "logs" / "acea_pipe_junction"
    return [root / run for run in DEFAULT_RUNS]


def _rotate_image_and_mask(rgb: Image.Image, mask: np.ndarray, angle_deg: float) -> tuple[Image.Image, np.ndarray]:
    # Positive PIL angles are counter-clockwise. The tracker angle is already
    # small in the current camera setup; using it directly keeps the line level
    # for the logged smoke cases.
    rotated_rgb = rgb.rotate(angle_deg, resample=Image.Resampling.BICUBIC, expand=False, fillcolor=(0, 0, 0))
    mask_img = Image.fromarray((mask.astype(np.uint8) * 255), mode="L")
    rotated_mask_img = mask_img.rotate(angle_deg, resample=Image.Resampling.NEAREST, expand=False, fillcolor=0)
    return rotated_rgb, np.asarray(rotated_mask_img) > 0


def _crop_pipe_strip(
    rotated_rgb: Image.Image,
    rotated_mask: np.ndarray,
    vertical_margin_px: int,
) -> tuple[Image.Image, np.ndarray, dict[str, Any]]:
    ys, xs = np.nonzero(rotated_mask)
    if xs.size == 0:
        raise ValueError("Rotated pipe mask is empty")

    width, height = rotated_rgb.size
    x0, x1 = 0, width - 1
    y0 = max(0, int(ys.min()) - vertical_margin_px)
    y1 = min(height - 1, int(ys.max()) + vertical_margin_px)
    strip = rotated_rgb.crop((x0, y0, x1 + 1, y1 + 1))
    strip_mask = rotated_mask[y0:y1 + 1, x0:x1 + 1]
    crop = {
        "x0": int(x0),
        "y0": int(y0),
        "x1": int(x1),
        "y1": int(y1),
        "width": int(strip.width),
        "height": int(strip.height),
    }
    return strip, strip_mask, crop


def _column_profile(gray: np.ndarray, mask: np.ndarray, min_valid_fraction: float) -> tuple[np.ndarray, np.ndarray]:
    height, width = gray.shape
    profile = np.full(width, np.nan, dtype=np.float64)
    counts = mask.sum(axis=0)
    min_count = max(8, int(min_valid_fraction * height))
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


def _median_filter_1d(values: np.ndarray, window: int) -> np.ndarray:
    window = max(3, int(window) | 1)
    half = window // 2
    output = np.empty_like(values, dtype=np.float64)
    for i in range(values.size):
        output[i] = float(np.median(values[max(0, i - half):min(values.size, i + half + 1)]))
    return output


def _score_columns(
    strip: Image.Image,
    strip_mask: np.ndarray,
    args: argparse.Namespace,
) -> dict[str, Any]:
    gray = np.asarray(strip.convert("L"), dtype=np.float64) / 255.0
    profile, counts = _column_profile(gray, strip_mask, args.min_valid_column_fraction)
    background = _median_filter_1d(profile, args.background_window_px)
    residual = background - profile

    width = profile.size
    edge_margin_px = max(args.edge_margin_px, int(round(width * args.edge_margin_fraction)))
    min_count = max(8, int(args.min_valid_column_fraction * strip.height))
    valid = np.isfinite(profile) & (counts >= min_count)
    interior = np.arange(width)
    valid &= interior >= edge_margin_px
    valid &= interior < width - edge_margin_px

    if not valid.any():
        raise ValueError("No valid interior columns for seam scoring")

    interior_residual = residual[valid]
    candidate_x = int(np.argmax(np.where(valid, residual, -np.inf)))
    contrast = float(max(0.0, residual[candidate_x]))
    confidence = np.clip(
        (contrast - args.min_dark_contrast) / max(args.strong_dark_contrast - args.min_dark_contrast, 1e-6),
        0.0,
        1.0,
    )
    accepted = bool(confidence >= args.accept_confidence)

    residual_median = float(np.median(interior_residual))
    residual_mad = float(np.median(np.abs(interior_residual - residual_median)))
    robust_sigma = max(1.4826 * residual_mad, 1.0 / 255.0)
    z_score = (float(residual[candidate_x]) - residual_median) / robust_sigma

    return {
        "profile": profile,
        "background": background,
        "residual": residual,
        "counts": counts,
        "valid_columns": valid,
        "candidate_x": candidate_x,
        "candidate_contrast": contrast,
        "candidate_z_score": float(z_score),
        "confidence": float(confidence),
        "accepted": accepted,
        "edge_margin_px": int(edge_margin_px),
    }


def _draw_rgb_axis_overlay(
    rgb: Image.Image,
    tracker_summary: dict[str, Any],
    seam_x: int | None,
    output_path: Path,
) -> None:
    image = rgb.convert("RGBA")
    draw = ImageDraw.Draw(image)

    line = tracker_summary["image_pipe_axis"].get("line_segment_uv")
    if line is not None:
        p0, p1 = line
        draw.line((p0[0], p0[1], p1[0], p1[1]), fill=(255, 40, 40, 255), width=3)

    centroid = tracker_summary["image_pipe_axis"].get("centroid_uv")
    if centroid is not None:
        u, v = centroid
        r = 5
        draw.ellipse((u - r, v - r, u + r, v + r), outline=(40, 120, 255, 255), width=3)

    if seam_x is not None:
        draw.line((seam_x, 0, seam_x, image.height - 1), fill=(0, 210, 255, 255), width=2)

    image.convert("RGB").save(output_path)


def _draw_strip_debug(strip: Image.Image, strip_mask: np.ndarray, candidate_x: int, accepted: bool, output_path: Path) -> None:
    image = strip.convert("RGBA")
    overlay = np.zeros((strip.height, strip.width, 4), dtype=np.uint8)
    overlay[strip_mask] = (30, 190, 90, 45)
    image = Image.alpha_composite(image, Image.fromarray(overlay, mode="RGBA"))
    draw = ImageDraw.Draw(image)
    color = (0, 220, 255, 255) if accepted else (255, 180, 0, 255)
    draw.line((candidate_x, 0, candidate_x, strip.height - 1), fill=color, width=2)
    image.convert("RGB").save(output_path)


def _draw_score_plot(score: dict[str, Any], args: argparse.Namespace, output_path: Path) -> None:
    width = 900
    height = 320
    pad_l, pad_r, pad_t, pad_b = 50, 20, 25, 45
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    profile = score["profile"]
    residual = score["residual"]
    candidate_x = score["candidate_x"]
    xs = np.linspace(pad_l, pad_l + plot_w, profile.size)

    image = Image.new("RGB", (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(image)
    draw.rectangle((pad_l, pad_t, pad_l + plot_w, pad_t + plot_h), outline=(80, 80, 80), width=1)

    def y_from_profile(v: float) -> float:
        return pad_t + (1.0 - np.clip(v, 0.0, 1.0)) * plot_h

    res_max = max(args.strong_dark_contrast, float(np.nanmax(np.maximum(residual, 0.0))), 1e-6)

    def y_from_residual(v: float) -> float:
        return pad_t + (1.0 - np.clip(v / res_max, 0.0, 1.0)) * plot_h

    prof_points = [(float(xs[i]), y_from_profile(float(profile[i]))) for i in range(profile.size)]
    res_points = [(float(xs[i]), y_from_residual(float(max(0.0, residual[i])))) for i in range(residual.size)]
    draw.line(prof_points, fill=(60, 100, 220), width=2)
    draw.line(res_points, fill=(220, 70, 70), width=2)

    threshold_y = y_from_residual(args.min_dark_contrast)
    draw.line((pad_l, threshold_y, pad_l + plot_w, threshold_y), fill=(120, 120, 120), width=1)
    cand_x_plot = float(xs[candidate_x])
    draw.line((cand_x_plot, pad_t, cand_x_plot, pad_t + plot_h), fill=(0, 160, 180), width=2)

    draw.text((pad_l, height - 32), "blue: strip brightness profile   red: dark-column seam score", fill=(30, 30, 30))
    draw.text((pad_l + 410, height - 32), f"candidate x={candidate_x}", fill=(30, 30, 30))
    image.save(output_path)


def process_frame(input_path: Path, output_dir: Path | None, args: argparse.Namespace) -> dict[str, Any]:
    frame_dir = _resolve_frame_dir(input_path)
    tracker_summary = _ensure_tracker_summary(frame_dir)
    metadata = _load_json(frame_dir / "metadata.json")
    rgb = Image.open(frame_dir / "rgb.png").convert("RGB")
    mask = np.asarray(Image.open(frame_dir / "pipe_tracker" / "pipe_mask.png").convert("L")) > 0

    axis_angle_deg = float(tracker_summary["image_pipe_axis"]["angle_deg"])
    rotated_rgb, rotated_mask = _rotate_image_and_mask(rgb, mask, axis_angle_deg)
    strip, strip_mask, crop = _crop_pipe_strip(rotated_rgb, rotated_mask, args.strip_vertical_margin_px)
    score = _score_columns(strip, strip_mask, args)

    if output_dir is None:
        output_dir = frame_dir / "pipe_strip"
    output_dir.mkdir(parents=True, exist_ok=True)

    candidate_x = int(score["candidate_x"])
    candidate_x_raw_approx = int(candidate_x + crop["x0"])
    strip.save(output_dir / "pipe_aligned_strip.png")
    _draw_strip_debug(strip, strip_mask, candidate_x, bool(score["accepted"]), output_dir / "pipe_aligned_strip_debug.png")
    _draw_rgb_axis_overlay(
        rgb,
        tracker_summary,
        candidate_x_raw_approx if score["accepted"] else None,
        output_dir / "rgb_axis_overlay.png",
    )
    _draw_score_plot(score, args, output_dir / "seam_score_plot.png")

    summary = {
        "input": {
            "frame_dir": str(frame_dir),
            "scene_mode": metadata["scene"]["scene_mode"],
            "seam_present_ground_truth": bool(metadata["scene"]["seam_present"]),
            "tracker_summary_json": str(frame_dir / "pipe_tracker" / "pipe_tracker_summary.json"),
        },
        "outputs": {
            "output_dir": str(output_dir),
            "rgb_axis_overlay_png": str(output_dir / "rgb_axis_overlay.png"),
            "pipe_aligned_strip_png": str(output_dir / "pipe_aligned_strip.png"),
            "pipe_aligned_strip_debug_png": str(output_dir / "pipe_aligned_strip_debug.png"),
            "seam_score_plot_png": str(output_dir / "seam_score_plot.png"),
            "summary_json": str(output_dir / "seam_detector_summary.json"),
        },
        "normalization": {
            "rotation_deg": _round(axis_angle_deg),
            "crop_xyxy": [crop["x0"], crop["y0"], crop["x1"], crop["y1"]],
            "strip_size_wh": [crop["width"], crop["height"]],
            "pipe_mask_pixels_in_strip": int(strip_mask.sum()),
        },
        "seam_detection": {
            "candidate_x_strip_px": int(candidate_x),
            "candidate_x_raw_px_approx": int(candidate_x_raw_approx),
            "candidate_contrast": _round(score["candidate_contrast"]),
            "candidate_z_score": _round(score["candidate_z_score"]),
            "confidence": _round(score["confidence"]),
            "accepted": bool(score["accepted"]),
            "thresholds": {
                "min_dark_contrast": _round(args.min_dark_contrast),
                "strong_dark_contrast": _round(args.strong_dark_contrast),
                "accept_confidence": _round(args.accept_confidence),
                "edge_margin_px": int(score["edge_margin_px"]),
                "background_window_px": int(args.background_window_px),
            },
        },
        "quick_result": {
            "declared_seam": bool(score["accepted"]),
            "expected_ground_truth_seam": bool(metadata["scene"]["seam_present"]),
            "matches_ground_truth_label": bool(score["accepted"]) == bool(metadata["scene"]["seam_present"]),
        },
        "assumptions": [
            "The depth tracker has already selected visible pipe pixels and estimated the image pipe axis.",
            "The strip is only an affine image normalization, not a full cylindrical unwrap yet.",
            "The first seam score looks for a thin dark vertical column inside the pipe strip.",
            "Pipe ends are ignored by an edge margin to reduce false positives on no-seam negative samples.",
            "Partially visible seam-at-edge frames may be rejected until temporal confirmation is added.",
        ],
    }
    _write_json(output_dir / "seam_detector_summary.json", summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Run directory or frame directory. If omitted, the three smoke runs are processed.",
    )
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory for a single input.")
    parser.add_argument("--strip-vertical-margin-px", type=int, default=6)
    parser.add_argument("--min-valid-column-fraction", type=float, default=0.25)
    parser.add_argument("--background-window-px", type=int, default=41)
    parser.add_argument("--edge-margin-fraction", type=float, default=0.06)
    parser.add_argument("--edge-margin-px", type=int, default=20)
    parser.add_argument("--min-dark-contrast", type=float, default=0.015)
    parser.add_argument("--strong-dark-contrast", type=float, default=0.06)
    parser.add_argument("--accept-confidence", type=float, default=0.35)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.inputs if args.inputs else _default_input_paths()
    if args.output_dir is not None and len(inputs) != 1:
        raise SystemExit("--output-dir can only be used with a single input")

    for input_path in inputs:
        summary = process_frame(input_path, args.output_dir, args)
        det = summary["seam_detection"]
        print(f"[pipe_strip_seam_detector] {summary['input']['frame_dir']}")
        print(
            "  seam={accepted} confidence={confidence:.2f} x={x} "
            "contrast={contrast:.3f} gt={gt}".format(
                accepted=det["accepted"],
                confidence=det["confidence"],
                x=det["candidate_x_strip_px"],
                contrast=det["candidate_contrast"],
                gt=summary["input"]["seam_present_ground_truth"],
            )
        )
        print(f"  wrote {summary['outputs']['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
