#!/usr/bin/env python3
"""Offline evaluation report for ACEA Module 2 pipe-junction logs.

This is Phase 6 of the ACEA Module 2 RGB-D demo. It discovers or accepts
logged seam/no-seam runs, runs or reads the existing Phase 2-5 outputs, and
writes a compact deterministic report.

It does not use ROS, CartesIO, lidar, or machine learning.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import pipe_seam_localizer
import pipe_temporal_confirmation


def _repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _round(value: float | None, digits: int = 6) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


def _frame_index(frame_dir: Path) -> int:
    try:
        return int(frame_dir.name.split("_")[-1])
    except Exception:
        metadata_path = frame_dir / "metadata.json"
        if metadata_path.exists():
            return int(_load_json(metadata_path).get("frame_index", 0))
    return 0


def _frame_dirs(run_dir: Path) -> list[Path]:
    return sorted(
        [path for path in run_dir.glob("frame_*") if (path / "metadata.json").exists()],
        key=_frame_index,
    )


def _first_metadata(run_dir: Path) -> dict[str, Any] | None:
    frames = _frame_dirs(run_dir)
    if frames:
        return _load_json(frames[0] / "metadata.json")
    scene_metadata = run_dir / "scene_metadata.json"
    if scene_metadata.exists():
        return _load_json(scene_metadata)
    return None


def _is_pipe_run(run_dir: Path) -> bool:
    metadata = _first_metadata(run_dir)
    if metadata is None:
        return False
    scene = metadata.get("scene", metadata)
    return scene.get("scene_mode") in ("seam", "no_seam") and bool(_frame_dirs(run_dir))


def _resolve_run(input_path: Path) -> Path:
    path = input_path.expanduser().resolve()
    if (path / "metadata.json").exists():
        return path.parent
    if path.is_dir() and _is_pipe_run(path):
        return path
    raise FileNotFoundError(f"Could not resolve logged ACEA pipe run: {input_path}")


def _discover_runs(log_root: Path) -> list[Path]:
    if not log_root.exists():
        return []
    runs = [path for path in log_root.iterdir() if path.is_dir() and _is_pipe_run(path)]
    return sorted(runs, key=lambda path: path.name)


def _temporal_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        output_dir=None,
        min_confidence=args.min_confidence,
        min_confirm_frames=args.min_confirm_frames,
        max_axis_angle_delta_deg=args.max_axis_angle_delta_deg,
        max_stand_off_delta_m=args.max_stand_off_delta_m,
        max_yaw_delta_deg=args.max_yaw_delta_deg,
        max_candidate_jump_px=args.max_candidate_jump_px,
        reset_geometry_on_reject=args.reset_geometry_on_reject,
    )


def _localizer_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        output_dir=None,
        surface_band_half_width_px=args.surface_band_half_width_px,
        min_surface_points=args.min_surface_points,
    )


def _classification(ground_truth_seam: bool | None, confirmed: bool | None) -> str:
    if ground_truth_seam is None or confirmed is None:
        return "unknown"
    if ground_truth_seam and confirmed:
        return "true_positive"
    if ground_truth_seam and not confirmed:
        return "false_negative"
    if not ground_truth_seam and confirmed:
        return "false_positive"
    return "true_negative"


def _mean_confidence(rows: list[dict[str, Any]]) -> float | None:
    values = [float(row["confidence"]) for row in rows if row.get("confidence") not in ("", None)]
    if not values:
        return None
    return sum(values) / len(values)


def _max_confidence(rows: list[dict[str, Any]]) -> float | None:
    values = [float(row["confidence"]) for row in rows if row.get("confidence") not in ("", None)]
    if not values:
        return None
    return max(values)


def _run_note(frame_count: int, ground_truth_seam: bool | None, confirmed: bool, args: argparse.Namespace) -> str:
    notes: list[str] = []
    if ground_truth_seam and frame_count < args.min_confirm_frames:
        notes.append(f"frame_count<{args.min_confirm_frames}_cannot_temporally_confirm")
    if ground_truth_seam and not confirmed:
        notes.append("expected_seam_not_confirmed")
    if ground_truth_seam is False and confirmed:
        notes.append("no_seam_false_positive")
    return ";".join(notes)


def _process_run(run_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    frames = _frame_dirs(run_dir)
    metadata = _first_metadata(run_dir)
    if metadata is None:
        raise RuntimeError("Run has no metadata")

    scene = metadata.get("scene", metadata)
    scene_mode = scene.get("scene_mode")
    ground_truth_seam = scene.get("seam_present")
    ground_truth_seam = None if ground_truth_seam is None else bool(ground_truth_seam)
    gap_width_m = scene.get("pipe", {}).get("nominal_gap_m")

    temporal_summary = pipe_temporal_confirmation.process_run(run_dir, None, _temporal_args(args))
    state = temporal_summary["state_machine"]
    confirmed = bool(state["confirmed"])
    localization_summary: dict[str, Any] | None = None
    localization_error: str | None = None

    if confirmed:
        try:
            localization_summary = pipe_seam_localizer.localize_run(run_dir, None, _localizer_args(args))
        except Exception as exc:
            localization_error = f"{type(exc).__name__}: {exc}"

    localization_gt = (localization_summary or {}).get("ground_truth_comparison", {})
    localization_outputs = (localization_summary or {}).get("outputs", {})
    temporal_rows = temporal_summary.get("per_frame", [])

    row = {
        "run_name": run_dir.name,
        "run_dir": str(run_dir),
        "scene_mode": scene_mode,
        "seam_present_ground_truth": ground_truth_seam,
        "gap_width_m": _round(gap_width_m),
        "frame_count": len(frames),
        "first_frame_index": _frame_index(frames[0]) if frames else None,
        "last_frame_index": _frame_index(frames[-1]) if frames else None,
        "confirmed": confirmed,
        "confirmed_frame": state.get("confirmed_frame_index"),
        "final_state": state.get("final_state"),
        "classification": _classification(ground_truth_seam, confirmed),
        "max_confidence": _round(_max_confidence(temporal_rows)),
        "mean_confidence": _round(_mean_confidence(temporal_rows)),
        "localization_ran": localization_summary is not None,
        "localization_error": localization_error,
        "pipe_center_to_gt_center_distance_m": _round(
            localization_gt.get("pipe_center_to_gt_center_distance_m")
        ),
        "signed_distance_along_gt_axis_m": _round(localization_gt.get("signed_distance_along_gt_axis_m")),
        "plane_normal_angle_error_deg": _round(localization_gt.get("plane_normal_angle_error_deg")),
        "temporal_summary_json": temporal_summary["outputs"]["temporal_summary_json"],
        "per_frame_states_csv": temporal_summary["outputs"]["per_frame_states_csv"],
        "localization_summary_json": localization_outputs.get("summary_json"),
        "note": _run_note(len(frames), ground_truth_seam, confirmed, args),
    }
    return row


def _csv_value(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "run_name",
        "scene_mode",
        "seam_present_ground_truth",
        "confirmed",
        "confirmed_frame",
        "classification",
        "frame_count",
        "max_confidence",
        "mean_confidence",
        "pipe_center_to_gt_center_distance_m",
        "signed_distance_along_gt_axis_m",
        "plane_normal_angle_error_deg",
        "localization_ran",
        "localization_error",
        "note",
        "run_dir",
        "temporal_summary_json",
        "localization_summary_json",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: _csv_value(row.get(name)) for name in fieldnames})


def _format_md_value(value: Any) -> str:
    if value is None or value == "":
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def _write_markdown(path: Path, rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    columns = [
        ("run_name", "Run"),
        ("scene_mode", "Scene"),
        ("seam_present_ground_truth", "GT Seam"),
        ("confirmed", "Confirmed"),
        ("confirmed_frame", "Frame"),
        ("classification", "Class"),
        ("pipe_center_to_gt_center_distance_m", "3D Err m"),
        ("signed_distance_along_gt_axis_m", "Axis Err m"),
        ("note", "Note"),
    ]

    lines = [
        "# ACEA Module 2 Evaluation Report",
        "",
        "Offline RGB-D pipe-junction evaluation over logged seam/no-seam runs.",
        "",
        "## Totals",
        "",
        f"- runs: {summary['totals']['run_count']}",
        f"- true positives: {summary['totals']['true_positive']}",
        f"- true negatives: {summary['totals']['true_negative']}",
        f"- false positives: {summary['totals']['false_positive']}",
        f"- false negatives: {summary['totals']['false_negative']}",
        "",
        "## Runs",
        "",
        "| " + " | ".join(label for _, label in columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]

    for row in rows:
        lines.append("| " + " | ".join(_format_md_value(row.get(key)) for key, _ in columns) + " |")

    lines.extend(
        [
            "",
            "## Assumptions",
            "",
            "- The evaluator re-runs the Phase 4 temporal state machine for repeatable thresholds.",
            "- Phase 2 and Phase 3 frame-level outputs are reused when already present, and generated when missing.",
            "- Phase 5 localization is run only for temporally confirmed runs.",
            "- Single-frame seam examples are expected to remain unconfirmed with the default two-frame persistence rule.",
            "- The report is for the first RGB-D simulation demo, not final weld metrology.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def evaluate(inputs: list[Path], output_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    for run_dir in inputs:
        try:
            row = _process_run(run_dir, args)
        except Exception as exc:
            row = {
                "run_name": run_dir.name,
                "run_dir": str(run_dir),
                "scene_mode": None,
                "seam_present_ground_truth": None,
                "confirmed": None,
                "confirmed_frame": None,
                "classification": "error",
                "frame_count": len(_frame_dirs(run_dir)) if run_dir.exists() else 0,
                "localization_ran": False,
                "localization_error": f"{type(exc).__name__}: {exc}",
                "note": "evaluation_failed",
            }
        rows.append(row)

    totals = {
        "run_count": len(rows),
        "true_positive": sum(1 for row in rows if row["classification"] == "true_positive"),
        "true_negative": sum(1 for row in rows if row["classification"] == "true_negative"),
        "false_positive": sum(1 for row in rows if row["classification"] == "false_positive"),
        "false_negative": sum(1 for row in rows if row["classification"] == "false_negative"),
        "error": sum(1 for row in rows if row["classification"] == "error"),
    }
    summary = {
        "input": {
            "runs": [str(path) for path in inputs],
            "log_root": str(args.log_root),
        },
        "outputs": {
            "output_dir": str(output_dir),
            "summary_json": str(output_dir / "acea_module2_eval_summary.json"),
            "report_csv": str(output_dir / "acea_module2_eval_report.csv"),
            "report_md": str(output_dir / "acea_module2_eval_report.md"),
        },
        "thresholds": {
            "min_confidence": _round(args.min_confidence),
            "min_confirm_frames": int(args.min_confirm_frames),
            "max_axis_angle_delta_deg": _round(args.max_axis_angle_delta_deg),
            "max_stand_off_delta_m": _round(args.max_stand_off_delta_m),
            "max_yaw_delta_deg": _round(args.max_yaw_delta_deg),
            "max_candidate_jump_px": int(args.max_candidate_jump_px),
            "surface_band_half_width_px": int(args.surface_band_half_width_px),
            "min_surface_points": int(args.min_surface_points),
        },
        "totals": totals,
        "runs": rows,
        "assumptions": [
            "Only directories with frame_*/metadata.json and scene_mode seam/no_seam are evaluated.",
            "The evaluator re-runs temporal confirmation using the configured thresholds.",
            "Localization is run only after temporal confirmation reaches STOP_AND_LOCALIZE/confirmed.",
            "False negative on a one-frame seam run usually means temporal persistence could not be established.",
            "Generated logs and reports remain under concert_isaac/logs, which is ignored by git.",
        ],
    }

    _write_json(output_dir / "acea_module2_eval_summary.json", summary)
    _write_csv(output_dir / "acea_module2_eval_report.csv", rows)
    _write_markdown(output_dir / "acea_module2_eval_report.md", rows, summary)
    return summary


def parse_args() -> argparse.Namespace:
    default_log_root = _repo_root_from_script() / "logs" / "acea_pipe_junction"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Run directories or frame directories. If omitted, all seam/no_seam runs under --log-root are evaluated.",
    )
    parser.add_argument("--log-root", type=Path, default=default_log_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for acea_module2_eval_summary/report files.",
    )
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--min-confirm-frames", type=int, default=2)
    parser.add_argument("--max-axis-angle-delta-deg", type=float, default=3.0)
    parser.add_argument("--max-stand-off-delta-m", type=float, default=0.15)
    parser.add_argument("--max-yaw-delta-deg", type=float, default=5.0)
    parser.add_argument("--max-candidate-jump-px", type=int, default=120)
    parser.add_argument("--reset-geometry-on-reject", action="store_true", default=False)
    parser.add_argument("--surface-band-half-width-px", type=int, default=4)
    parser.add_argument("--min-surface-points", type=int, default=50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.log_root = args.log_root.expanduser().resolve()
    if args.inputs:
        runs = [_resolve_run(path) for path in args.inputs]
    else:
        runs = _discover_runs(args.log_root)

    if not runs:
        raise SystemExit(f"No seam/no_seam ACEA pipe runs found under {args.log_root}")

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.log_root / "module2_evaluation"
    output_dir = output_dir.expanduser().resolve()

    summary = evaluate(runs, output_dir, args)
    totals = summary["totals"]
    print(f"[pipe_module2_evaluator] wrote {summary['outputs']['output_dir']}")
    print(
        "  runs={runs} TP={tp} TN={tn} FP={fp} FN={fn} errors={err}".format(
            runs=totals["run_count"],
            tp=totals["true_positive"],
            tn=totals["true_negative"],
            fp=totals["false_positive"],
            fn=totals["false_negative"],
            err=totals["error"],
        )
    )
    for row in summary["runs"]:
        print(
            "  {run}: class={klass} confirmed={confirmed} frame={frame} err={err}".format(
                run=row["run_name"],
                klass=row["classification"],
                confirmed=row.get("confirmed"),
                frame=row.get("confirmed_frame"),
                err=row.get("pipe_center_to_gt_center_distance_m"),
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
