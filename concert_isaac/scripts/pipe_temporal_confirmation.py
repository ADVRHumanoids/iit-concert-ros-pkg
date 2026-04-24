#!/usr/bin/env python3
"""Temporal confirmation for ACEA pipe-junction seam detections.

This is Phase 4 of the ACEA Module 2 perception demo. It consumes a sequence of
logged RGB-D frames from one scan run, runs or reads the Phase 3 seam detector
summary for each frame, and applies a deterministic state machine:

SCAN -> CANDIDATE -> CONFIRMED -> STOP_AND_LOCALIZE

It does not use ROS, CartesIO, lidar, or machine learning.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

import pipe_strip_seam_detector


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


def _default_input_paths() -> list[Path]:
    root = _repo_root_from_script() / "logs" / "acea_pipe_junction"
    return [root / run for run in DEFAULT_RUNS]


def _frame_index(frame_dir: Path) -> int:
    try:
        return int(frame_dir.name.split("_")[-1])
    except Exception:
        metadata_path = frame_dir / "metadata.json"
        if metadata_path.exists():
            return int(_load_json(metadata_path).get("frame_index", 0))
    return 0


def _resolve_run_frames(input_path: Path) -> tuple[Path, list[Path]]:
    path = input_path.expanduser().resolve()
    if (path / "metadata.json").exists():
        return path.parent, [path]

    if (path / "manifest.jsonl").exists() or path.is_dir():
        frames = sorted(
            [p for p in path.glob("frame_*") if (p / "metadata.json").exists()],
            key=_frame_index,
        )
        if frames:
            return path, frames

    raise FileNotFoundError(f"Could not find logged frame directories under {input_path}")


def _default_detector_args() -> argparse.Namespace:
    return argparse.Namespace(
        output_dir=None,
        strip_vertical_margin_px=6,
        min_valid_column_fraction=0.25,
        background_window_px=41,
        edge_margin_fraction=0.06,
        edge_margin_px=20,
        min_dark_contrast=0.015,
        strong_dark_contrast=0.06,
        accept_confidence=0.35,
    )


def _ensure_seam_summary(frame_dir: Path) -> dict[str, Any]:
    summary_path = frame_dir / "pipe_strip" / "seam_detector_summary.json"
    if summary_path.exists():
        return _load_json(summary_path)
    return pipe_strip_seam_detector.process_frame(frame_dir, None, _default_detector_args())


def _load_tracker_summary(frame_dir: Path) -> dict[str, Any] | None:
    path = frame_dir / "pipe_tracker" / "pipe_tracker_summary.json"
    if not path.exists():
        return None
    return _load_json(path)


def _geometry_values(frame_dir: Path, seam_summary: dict[str, Any]) -> dict[str, float | None]:
    tracker = _load_tracker_summary(frame_dir)
    if tracker is None:
        return {
            "pipe_axis_angle_deg": seam_summary["normalization"]["rotation_deg"],
            "stand_off_m": None,
            "yaw_error_deg": None,
        }
    return {
        "pipe_axis_angle_deg": float(tracker["image_pipe_axis"]["angle_deg"]),
        "stand_off_m": float(tracker["camera_frame_estimate"]["stand_off_m"]),
        "yaw_error_deg": float(tracker["camera_frame_estimate"]["yaw_error_deg"]),
    }


def _geometry_consistent(
    current: dict[str, float | None],
    previous: dict[str, float | None] | None,
    args: argparse.Namespace,
) -> tuple[bool, list[str]]:
    if previous is None:
        return True, []

    failures: list[str] = []
    axis_delta = abs(float(current["pipe_axis_angle_deg"]) - float(previous["pipe_axis_angle_deg"]))
    if axis_delta > args.max_axis_angle_delta_deg:
        failures.append(f"axis_delta={axis_delta:.3f}deg")

    if current["stand_off_m"] is not None and previous["stand_off_m"] is not None:
        stand_delta = abs(float(current["stand_off_m"]) - float(previous["stand_off_m"]))
        if stand_delta > args.max_stand_off_delta_m:
            failures.append(f"stand_off_delta={stand_delta:.3f}m")

    if current["yaw_error_deg"] is not None and previous["yaw_error_deg"] is not None:
        yaw_delta = abs(float(current["yaw_error_deg"]) - float(previous["yaw_error_deg"]))
        if yaw_delta > args.max_yaw_delta_deg:
            failures.append(f"yaw_delta={yaw_delta:.3f}deg")

    return not failures, failures


def _candidate_not_border(det: dict[str, Any], norm: dict[str, Any]) -> bool:
    x = int(det["candidate_x_strip_px"])
    width = int(norm["strip_size_wh"][0])
    edge_margin = int(det["thresholds"]["edge_margin_px"])
    return edge_margin <= x < width - edge_margin


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "frame_index",
        "state",
        "confidence",
        "candidate_x_strip_px",
        "candidate_not_border",
        "geometry_consistent",
        "eligible",
        "candidate_streak",
        "stand_off_m",
        "yaw_error_deg",
        "pipe_axis_angle_deg",
        "reason",
        "frame_dir",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def _draw_confidence_plot(rows: list[dict[str, Any]], args: argparse.Namespace, output_path: Path) -> None:
    width = max(900, 80 + 70 * max(1, len(rows)))
    height = 330
    pad_l, pad_r, pad_t, pad_b = 60, 30, 30, 55
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    image = Image.new("RGB", (width, height), (250, 250, 250))
    draw = ImageDraw.Draw(image)
    draw.rectangle((pad_l, pad_t, pad_l + plot_w, pad_t + plot_h), outline=(80, 80, 80), width=1)

    if len(rows) == 1:
        xs = [pad_l + plot_w / 2.0]
    else:
        xs = [pad_l + i * plot_w / (len(rows) - 1) for i in range(len(rows))]

    def y_conf(conf: float) -> float:
        return pad_t + (1.0 - max(0.0, min(1.0, conf))) * plot_h

    threshold_y = y_conf(args.min_confidence)
    draw.line((pad_l, threshold_y, pad_l + plot_w, threshold_y), fill=(120, 120, 120), width=1)
    draw.text((pad_l + 5, threshold_y - 18), f"confidence threshold {args.min_confidence:.2f}", fill=(80, 80, 80))

    points = [(xs[i], y_conf(float(row["confidence"]))) for i, row in enumerate(rows)]
    if len(points) > 1:
        draw.line(points, fill=(70, 100, 220), width=2)

    state_colors = {
        "SCAN": (140, 140, 140),
        "CANDIDATE": (255, 160, 0),
        "CONFIRMED": (20, 150, 70),
        "STOP_AND_LOCALIZE": (0, 120, 190),
    }
    for x, row in zip(xs, rows):
        y = y_conf(float(row["confidence"]))
        color = state_colors.get(str(row["state"]), (50, 50, 50))
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)
        draw.text((x - 16, height - 38), str(row["frame_index"]), fill=(40, 40, 40))

    draw.text((pad_l, height - 22), "Frame index; dot color is temporal state", fill=(30, 30, 30))
    image.save(output_path)


def process_run(input_path: Path, output_dir: Path | None, args: argparse.Namespace) -> dict[str, Any]:
    run_dir, frames = _resolve_run_frames(input_path)
    if output_dir is None:
        output_dir = run_dir / "temporal_confirmation"
    output_dir.mkdir(parents=True, exist_ok=True)

    state = "SCAN"
    candidate_streak = 0
    previous_geometry: dict[str, float | None] | None = None
    previous_candidate_x: int | None = None
    confirmed_frame_index: int | None = None
    confirmed_frame_dir: str | None = None
    rows: list[dict[str, Any]] = []

    for frame_dir in frames:
        seam_summary = _ensure_seam_summary(frame_dir)
        det = seam_summary["seam_detection"]
        norm = seam_summary["normalization"]
        geometry = _geometry_values(frame_dir, seam_summary)

        frame_index = _frame_index(frame_dir)
        confidence = float(det["confidence"])
        candidate_x = int(det["candidate_x_strip_px"])
        candidate_not_border = _candidate_not_border(det, norm)
        confidence_ok = confidence >= args.min_confidence
        detector_accepts = bool(det["accepted"])
        geometry_ok, geometry_failures = _geometry_consistent(geometry, previous_geometry, args)
        jump_ok = True
        jump_reason = ""
        if previous_candidate_x is not None:
            jump = abs(candidate_x - previous_candidate_x)
            if jump > args.max_candidate_jump_px:
                jump_ok = False
                jump_reason = f"candidate_jump={jump}px"

        eligible = detector_accepts and confidence_ok and candidate_not_border and geometry_ok and jump_ok

        if state in ("CONFIRMED", "STOP_AND_LOCALIZE"):
            state = "STOP_AND_LOCALIZE"
        elif eligible:
            candidate_streak += 1
            if candidate_streak >= args.min_confirm_frames:
                state = "CONFIRMED"
                if confirmed_frame_index is None:
                    confirmed_frame_index = frame_index
                    confirmed_frame_dir = str(frame_dir)
            else:
                state = "CANDIDATE"
        else:
            candidate_streak = 0
            state = "SCAN"

        reason_parts = []
        if not detector_accepts:
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
        if state == "CONFIRMED":
            reason_parts.append("confirmed")
        if state == "STOP_AND_LOCALIZE":
            reason_parts.append("already_confirmed")

        row = {
            "frame_index": frame_index,
            "state": state,
            "confidence": _round(confidence),
            "candidate_x_strip_px": candidate_x,
            "candidate_not_border": candidate_not_border,
            "geometry_consistent": geometry_ok,
            "eligible": eligible,
            "candidate_streak": candidate_streak,
            "stand_off_m": "" if geometry["stand_off_m"] is None else _round(float(geometry["stand_off_m"])),
            "yaw_error_deg": "" if geometry["yaw_error_deg"] is None else _round(float(geometry["yaw_error_deg"])),
            "pipe_axis_angle_deg": _round(float(geometry["pipe_axis_angle_deg"])),
            "reason": ";".join(reason_parts),
            "frame_dir": str(frame_dir),
        }
        rows.append(row)

        if eligible:
            previous_geometry = geometry
            previous_candidate_x = candidate_x
        elif args.reset_geometry_on_reject:
            previous_geometry = None
            previous_candidate_x = None

    final_state = rows[-1]["state"] if rows else "SCAN"
    confirmed = confirmed_frame_index is not None
    summary = {
        "input": {
            "run_dir": str(run_dir),
            "frame_count": len(frames),
            "frames": [str(frame) for frame in frames],
        },
        "outputs": {
            "output_dir": str(output_dir),
            "temporal_summary_json": str(output_dir / "temporal_summary.json"),
            "temporal_confidence_plot_png": str(output_dir / "temporal_confidence_plot.png"),
            "per_frame_states_csv": str(output_dir / "per_frame_states.csv"),
        },
        "state_machine": {
            "states": ["SCAN", "CANDIDATE", "CONFIRMED", "STOP_AND_LOCALIZE"],
            "final_state": final_state,
            "confirmed": confirmed,
            "confirmed_frame_index": confirmed_frame_index,
            "confirmed_frame_dir": confirmed_frame_dir,
            "min_confirm_frames": int(args.min_confirm_frames),
        },
        "thresholds": {
            "min_confidence": _round(args.min_confidence),
            "max_axis_angle_delta_deg": _round(args.max_axis_angle_delta_deg),
            "max_stand_off_delta_m": _round(args.max_stand_off_delta_m),
            "max_yaw_delta_deg": _round(args.max_yaw_delta_deg),
            "max_candidate_jump_px": int(args.max_candidate_jump_px),
        },
        "per_frame": rows,
        "assumptions": [
            "The script reads or runs the Phase 3 seam detector for each logged frame.",
            "A single frame can become CANDIDATE but cannot be CONFIRMED unless min_confirm_frames is 1.",
            "Geometry consistency is checked using the Phase 2 camera-frame pipe axis, stand-off, and yaw estimates.",
            "Candidate x-position must stay away from the strip border to avoid pipe-end/image-edge false positives.",
            "STOP_AND_LOCALIZE means the offline detector would recommend stopping the base and handing off to coarse 3D localization.",
        ],
    }

    _write_csv(output_dir / "per_frame_states.csv", rows)
    _draw_confidence_plot(rows, args, output_dir / "temporal_confidence_plot.png")
    _write_json(output_dir / "temporal_summary.json", summary)
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
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--min-confirm-frames", type=int, default=2)
    parser.add_argument("--max-axis-angle-delta-deg", type=float, default=3.0)
    parser.add_argument("--max-stand-off-delta-m", type=float, default=0.15)
    parser.add_argument("--max-yaw-delta-deg", type=float, default=5.0)
    parser.add_argument("--max-candidate-jump-px", type=int, default=120)
    parser.add_argument("--reset-geometry-on-reject", action="store_true", default=False)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.inputs if args.inputs else _default_input_paths()
    if args.output_dir is not None and len(inputs) != 1:
        raise SystemExit("--output-dir can only be used with a single input")

    for input_path in inputs:
        summary = process_run(input_path, args.output_dir, args)
        state = summary["state_machine"]
        print(f"[pipe_temporal_confirmation] {summary['input']['run_dir']}")
        print(
            "  frames={frames} final_state={final_state} confirmed={confirmed} confirmed_frame={frame}".format(
                frames=summary["input"]["frame_count"],
                final_state=state["final_state"],
                confirmed=state["confirmed"],
                frame=state["confirmed_frame_index"],
            )
        )
        print(f"  wrote {summary['outputs']['output_dir']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
