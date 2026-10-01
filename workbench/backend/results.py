"""Expose original tool artifacts without inventing unavailable measurements."""

import csv
import math
import os
from pathlib import Path
import re
from urllib.parse import quote

import yaml

from .store import inside, read_json


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def artifact_file(job, relative):
    root = Path(job["directory"]).resolve()
    path = (root / relative).resolve()
    if not inside(path, root) or not path.is_file():
        raise KeyError("Artifact is not a file within this task")
    return path


def artifact_url(job, relative):
    return "/api/artifacts/" + job["id"] + "/" + quote(relative, safe="/")


def artifacts(job):
    root = Path(job["directory"])
    items = []
    paths = []
    for directory, folders, names in os.walk(root):
        # Avoid traversing every object file or the full isolated source checkout.
        if Path(directory) == root / "acceptance":
            folders[:] = [name for name in folders if name not in {"source", "build"}]
        paths.extend(Path(directory) / name for name in names)
    for path in sorted(paths):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        # A source copy/build tree is evidence, but not an artifact preview inventory.
        if relative.endswith(".tmp"):
            continue
        if not inside(path.resolve(), root.resolve()):
            continue
        kind = ("image" if path.suffix.lower() in {".png", ".jpg", ".jpeg"} else
                "yaml" if path.suffix.lower() in {".yaml", ".yml"} else
                "table" if path.suffix.lower() in {".tsv", ".csv"} else
                "json" if path.suffix.lower() == ".json" else "text")
        items.append({"path": relative, "name": path.name, "kind": kind,
                      "url": artifact_url(job, relative)})
    return items


def read_commands(path):
    if not path.is_file():
        return []
    rows = []
    with path.open(encoding="utf-8", newline="") as stream:
        for record in csv.DictReader(stream, delimiter="\t"):
            row = {}
            for key, value in record.items():
                if value in {"NA", "", None}:
                    row[key] = None
                elif key == "space":
                    row[key] = value
                else:
                    try:
                        row[key] = int(value) if key in {
                            "write_ns", "frame_id", "generation", "exposure_ns", "control", "shoot"
                        } else float(value)
                    except ValueError:
                        row[key] = value
            rows.append(json_safe(row))
    return rows


def enemy(run):
    return run.get("effective_configuration", {}).get("detector", {}).get("enemy")


def comparison(job, logical, timing, groups=None):
    rows, reasons, conditions = [], [], []
    snapshot_ids = {str(Path(path).resolve()): identifier
                    for identifier, path in job.get("snapshots", {}).items()}
    build_ids = list(dict.fromkeys(run["build_id"] for run in job["options"].get("runs", [])))
    bound_groups = ([(index, report, timing[index] if index < len(timing) else None)
                     for index, report in enumerate(logical)] if groups is None else groups)
    for index, report, costs in bound_groups:
        if not isinstance(report, dict):
            reasons.append("Logical report was not produced for benchmark_" + str(index))
            continue
        if groups is not None and costs is None:
            reasons.append("Timing report was not produced for benchmark_" + str(index))
        measured = costs.get("runs", []) if isinstance(costs, dict) else []
        timing_by_path = {}
        for measured_run in measured:
            timing_by_path.setdefault(measured_run.get("configuration_path"), []).append(measured_run)
        for run in report.get("runs", []):
            path = run.get("configuration_path", "")
            profile_id = snapshot_ids.get(str(Path(path).resolve())) if path else None
            metadata = read_json(Path(path).parent / "profile.json", {}) if profile_id else {}
            matching_costs = timing_by_path.get(path, [])
            rows.append({"build_id": build_ids[index] if index < len(build_ids) else None,
                         "profile_id": profile_id, "name": metadata.get("name", profile_id),
                         "detector": run.get("detector"), "summary": run.get("summary", {}),
                         "timing": matching_costs.pop(0) if matching_costs else None, "logic": run})
            conditions.append({"dataset_fingerprint": report.get("dataset_fingerprint"),
                               "enemy": enemy(run), "minimum_iou": report.get("minimum_iou"),
                               "pose_reference": report.get("pose_reference"),
                               "pose_metrics": report.get("pose_metrics"),
                               "position_limit": job["options"].get("position_limit"),
                               "rotation_limit": job["options"].get("rotation_limit")})
    if not rows:
        reasons.append("No batch evaluation reports were produced")
    for key in ("dataset_fingerprint", "enemy", "minimum_iou", "pose_metrics"):
        if conditions and any(item[key] is None for item in conditions):
            reasons.append("Missing comparison condition: " + key)
    if conditions:
        first = conditions[0]
        for key in first:
            if any(item[key] != first[key] for item in conditions[1:]):
                reasons.append("Comparison conditions differ: " + key)
    expected = len(job["options"].get("runs", []))
    if expected and len(rows) != expected:
        reasons.append("Some requested runs did not produce a report")
    return json_safe({"compatible": bool(rows) and not reasons, "reasons": reasons, "rows": rows})


def results(job):
    root = Path(job["directory"])
    reports, logic, timings, frames = [], [], [], []
    grouped = {}
    for item in artifacts(job):
        relative, name = item["path"], item["name"]
        path = artifact_file(job, relative)
        if name in {"report.yaml", "timing.yaml", "check_config.yaml"}:
            try:
                data = yaml.safe_load(path.read_text(encoding="utf-8"))
            except yaml.YAMLError:
                data = {"unparsed_output": path.read_text(encoding="utf-8")}
            kind = {"report.yaml": "logic", "timing.yaml": "timing", "check_config.yaml": "configuration"}[name]
            reports.append({"name": relative, "kind": kind, "data": json_safe(data)})
            if name == "report.yaml" and isinstance(data, dict):
                logic.append(data)
            elif name == "timing.yaml" and isinstance(data, dict):
                timings.append(data)
            group = re.fullmatch(r"benchmark_(\d+)", path.parent.name)
            if group and name in {"report.yaml", "timing.yaml"}:
                grouped.setdefault(int(group[1]), {})[name] = data if isinstance(data, dict) else None
        elif name in {"pipeline_metrics.txt", "single_image.txt"}:
            text = path.read_text(encoding="utf-8")
            data = text
            if name == "single_image.txt":
                data = {"scope": "local_detector_only", "text": text}
                for key, value in re.findall(r"([a-z0-9_]+)=([^\s]+)", text):
                    try:
                        data[key] = float(value)
                    except ValueError:
                        data[key] = None if value == "NA" else value
            reports.append({"name": relative, "kind": "timing" if name == "single_image.txt" else "text",
                            "data": data})
        elif relative.startswith("visualized/"):
            match = re.fullmatch(r"(\d+)_(\d+)\.png", name)
            if match:
                frame = {"generation": int(match[1]), "frame_id": int(match[2]),
                         "url": item["url"], "source": "independent_replay"}
                original = "recording/frame_" + str(frame["frame_id"]) + ".png"
                if (root / original).is_file() and inside((root / original).resolve(), root.resolve()):
                    frame["original_url"] = artifact_url(job, original)
                frames.append(frame)
    frames.sort(key=lambda frame: (frame["generation"], frame["frame_id"]))
    bound_groups = None
    if job["kind"] == "benchmark":
        expected = len(set(run["build_id"] for run in job["options"].get("runs", [])))
        indices = sorted(set(range(expected)) | set(grouped))
        bound_groups = [(index, grouped.get(index, {}).get("report.yaml"),
                         grouped.get(index, {}).get("timing.yaml")) for index in indices]
    return {"reports": reports, "frames": frames, "commands": read_commands(root / "commands.tsv"),
            "comparison": comparison(job, logic, timings, groups=bound_groups),
            "preview_source": "independent replay_visualizer run; not batch evaluation frame records"}
