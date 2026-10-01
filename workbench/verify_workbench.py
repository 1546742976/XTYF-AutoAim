"""Exercise the running local workbench with actual C++ programs, never fake tools.

Run on the same Linux/WSL host as the server. Creates new profiles and experiments
in its workspace. Defaults to building and testing the no-model baseline; pass an
existing successful --build-id to reuse it. --with-combination also builds I1+I2.
Optional --yolov5-model / --yolo11-model use existing XML/BIN pairs for real CPU
inference. Synthetic images verify execution only, never real-world accuracy.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import yaml


FLAGS = (
    "AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM", "AUTOAIM_I3_LINEAR_CA",
    "AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC", "AUTOAIM_USE_ESO",
)


class Client:
    def __init__(self, url, timeout):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.jobs = []

    def request(self, path, data=None):
        request = Request(self.url + path, headers={"Content-Type": "application/json"},
                          data=json.dumps(data).encode() if data is not None else None)
        try:
            with urlopen(request, timeout=30) as response:
                return json.load(response)
        except HTTPError as error:
            raise RuntimeError(error.read().decode()) from error

    def job(self, kind, **options):
        job = self.request("/api/jobs", {"kind": kind, "options": options})
        index = len(self.jobs)
        self.jobs.append(job)
        deadline = time.monotonic() + self.timeout
        previous = None
        while job["status"] in {"queued", "running"}:
            if previous != job["status"]:
                print(f"{kind} {job['id']}: {job['status']}", flush=True)
                previous = job["status"]
            if time.monotonic() >= deadline:
                self.jobs[index] = self.request(f"/api/jobs/{job['id']}/cancel", {})
                raise TimeoutError(f"Timed out and requested cancellation: {job['id']}")
            time.sleep(0.5)
            job = self.request(f"/api/jobs/{job['id']}")
            self.jobs[index] = job
        if job["status"] != "succeeded":
            log = self.request(f"/api/jobs/{job['id']}/log")
            raise RuntimeError(f"{kind}: {job['status']}\n{log['text']}")
        print(f"{kind} {job['id']}: succeeded", flush=True)
        return job

    def image(self, path):
        with urlopen(self.url + path, timeout=30) as response:
            content = response.read()
        if not content.startswith(b"\x89PNG\r\n\x1a\n"):
            raise AssertionError("Preview URL did not return a PNG: " + path)
        return content

    def artifacts(self, job):
        return self.request(f"/api/jobs/{job['id']}/artifacts")

    def artifact_path(self, job, name):
        matches = [item for item in self.artifacts(job) if Path(item["path"]).name == name]
        if len(matches) != 1:
            raise AssertionError(f"Expected one {name}, found {len(matches)}")
        return Path(job["directory"]) / matches[0]["path"]


def logical_report(report):
    result = copy.deepcopy(report)
    # These two invocation fields necessarily differ for a new output directory.
    result.pop("command_line", None)
    result.pop("working_directory", None)
    return result


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def config_inventory(directory):
    return {path.relative_to(directory).as_posix(): sha256(path)
            for path in sorted(directory.rglob("*")) if path.is_file()}


def supplied_models(args):
    models = {}
    for kind, supplied in (("yolov5", args.yolov5_model), ("yolo11", args.yolo11_model)):
        if supplied is None:
            continue
        xml = Path(supplied).expanduser().resolve()
        binary = xml.with_suffix(".bin")
        if xml.suffix.lower() != ".xml" or not xml.is_file() or not binary.is_file():
            raise ValueError("Model needs an existing XML and adjacent BIN: " + str(xml))
        models[kind] = {"xml": str(xml), "bin": str(binary),
                        "xml_sha256": sha256(xml), "bin_sha256": sha256(binary)}
    if args.openvino_dir:
        directory = Path(args.openvino_dir).expanduser().resolve()
        if not models or not directory.is_dir():
            raise ValueError("--openvino-dir requires a supplied model and an existing directory")
    return models


def check_configuration(client, build_id, profile_id, detector):
    job = client.job("check_config", build_id=build_id, profile_id=profile_id)
    path = client.artifact_path(job, "check_config.yaml")
    checked = yaml.safe_load(path.read_text(encoding="utf-8"))
    if checked.get("check_schema_version") != 1 or checked.get("valid") is not True:
        raise AssertionError("Configuration check did not produce the valid schema")
    if checked["effective_configuration"]["detector"]["kind"] != detector:
        raise AssertionError("Configuration selected a different detector")
    return job


def check_replay(client, build_id, profile_id, dataset):
    job = client.job("replay", build_id=build_id, profile_id=profile_id, dataset=dataset)
    view = client.request(f"/api/jobs/{job['id']}/results")
    if len(view["frames"]) != 6 or not view["commands"]:
        raise AssertionError("Replay did not expose six frames and command records")
    for frame in view["frames"]:
        if frame.get("source") != "independent_replay" or not frame.get("original_url"):
            raise AssertionError("Replay preview lacks its recording provenance/original image")
        client.image(frame["url"])
        client.image(frame["original_url"])
    for name in ("commands.tsv", "uart.hex", "pipeline_metrics.txt"):
        client.artifact_path(job, name)
    if not (Path(job["directory"]) / "recording/events.yaml").is_file():
        raise AssertionError("Replay did not preserve a recording session")
    return job


def check_batch(client, evidence, directory, dataset, runs):
    job = client.job("benchmark", dataset=dataset, iou=0.5, runs=runs)
    result = client.request(f"/api/jobs/{job['id']}/results")
    if not result["comparison"]["compatible"] or len(result["comparison"]["rows"]) != len(runs):
        raise AssertionError("Comparison conditions or requested run count differ")
    # Group artifacts by the exact persisted --output of each C++ process.
    commands = [command for command in job["commands"]
                if Path(command["argv"][0]).name == "bench_detector"]
    reports = []
    for index, command in enumerate(commands):
        argv = list(command["argv"])
        api_output = Path(argv[argv.index("--output") + 1])
        report_path, timing_path = api_output / "report.yaml", api_output / "timing.yaml"
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        timing = yaml.safe_load(timing_path.read_text(encoding="utf-8"))
        if len(report["runs"]) != len(timing["runs"]):
            raise AssertionError("Logical and timing report run counts differ")
        output = directory / ("direct_" + job["id"] + "_" + str(index))
        argv[argv.index("--output") + 1] = str(output)
        completed = subprocess.run(argv, cwd=command["cwd"], capture_output=True, text=True)
        direct_command = {"argv": argv, "cwd": command["cwd"], "exit_code": completed.returncode,
                          "report": str(output / "report.yaml"), "timing": str(output / "timing.yaml")}
        evidence.setdefault("direct_commands", []).append(direct_command)
        (directory / (output.name + ".log")).write_text(completed.stdout + completed.stderr, encoding="utf-8")
        if completed.returncode:
            raise AssertionError("Direct benchmark CLI failed: " + completed.stderr)
        direct = yaml.safe_load((output / "report.yaml").read_text(encoding="utf-8"))
        if logical_report(direct) != logical_report(report):
            raise AssertionError("API and direct CLI logical reports differ")
        reports.append(report)
    if len(commands) != len({run["build_id"] for run in runs}):
        raise AssertionError("Batch did not run exactly one process per build")
    evidence.setdefault("cli_equivalence", []).append({"job": job["id"], "passed": True,
                                                        "groups": len(commands)})
    return job, reports


def workflow(args, client, context, models, evidence, directory):
    for model in models.values():
        client.request("/api/roots", {"path": str(Path(model["xml"]).parent)})
    if args.openvino_dir:
        client.request("/api/roots", {"path": str(Path(args.openvino_dir).expanduser().resolve())})
    template = client.request("/api/config/template")
    values = copy.deepcopy(template["values"])
    values["active_detector"] = "traditional"
    name = "API acceptance " + time.strftime("%Y%m%d-%H%M%S")
    first = client.request("/api/profiles", {"name": name, "values": values})
    modified = copy.deepcopy(values)
    modified["common"]["refinement"]["brightness"] = 101
    second = client.request("/api/profiles", {"name": name + " variant", "values": modified})
    if first["id"] == second["id"]:
        raise AssertionError("Saving a new profile reused the previous ID")
    evidence["profiles"] += [{"id": first["id"], "detector": "traditional"},
                              {"id": second["id"], "detector": "traditional"}]

    def build(label, enabled, use_models=False):
        options = {"name": label, "flags": {key: key in enabled for key in FLAGS},
                   "openvino": use_models, "build_type": "Release", "jobs": 2}
        if use_models:
            options.update({kind + "_model": model["xml"] for kind, model in models.items()})
            if args.openvino_dir:
                options["openvino_dir"] = str(Path(args.openvino_dir).expanduser().resolve())
        job = client.job("build", **options)
        builds = client.request("/api/builds")
        found = [item for item in builds if item["id"] == job["id"]]
        if not found:
            raise AssertionError("Completed build is missing from build registry")
        return found[0]["id"]

    build_id = args.build_id or build(name + " baseline", set(), bool(models))
    registered = [item for item in client.request("/api/builds") if item["id"] == build_id]
    if not registered or registered[0]["status"] != "succeeded":
        raise AssertionError("The reused baseline must be a successful registered build")
    reused = client.request("/api/jobs/" + build_id)
    if not any(job["id"] == build_id for job in client.jobs):
        client.jobs.append(reused)
    evidence["baseline"] = build_id
    if models:
        if not registered[0]["openvino"]:
            raise AssertionError("Supplied models require an OpenVINO build")
        for kind, model in models.items():
            accepted = registered[0]["models"].get("YOLOV5" if kind == "yolov5" else "YOLO11", {})
            if any(accepted.get(key) != model[key] for key in ("xml_sha256", "bin_sha256")):
                raise AssertionError("Build model fingerprints differ from supplied " + kind)
        if args.build_id and args.openvino_dir:
            cache = (Path(registered[0]["build_dir"]) / "CMakeCache.txt").read_text(encoding="utf-8")
            configured = [line.split("=", 1)[1] for line in cache.splitlines()
                          if line.startswith("OpenVINO_DIR:")]
            requested = Path(args.openvino_dir).expanduser().resolve()
            if len(configured) != 1 or Path(configured[0]).resolve() != requested:
                raise AssertionError("Reused build OpenVINO_DIR differs from --openvino-dir")
    check_configuration(client, build_id, first["id"], "traditional")
    synthetic = client.job("synthetic", build_id=build_id, profile_id=first["id"], frames=6)
    dataset = str(client.artifact_path(synthetic, "events.yaml"))
    replay = check_replay(client, build_id, first["id"], dataset)
    evidence.update(synthetic=synthetic["id"], replay=replay["id"])
    compared, _ = check_batch(client, evidence, directory, dataset, [
        {"build_id": build_id, "profile_id": first["id"]},
        {"build_id": build_id, "profile_id": second["id"]},
    ])
    evidence["comparison"] = compared["id"]

    image = str(client.artifact_path(synthetic, "frame_000001.png"))
    for kind, model in models.items():
        selected = copy.deepcopy(template["values"])
        selected["active_detector"] = kind
        selected["detectors"][kind]["model_path"] = model["xml"]
        selected["detectors"][kind]["device"] = "CPU"
        profile = client.request("/api/profiles", {"name": name + " " + kind, "values": selected})
        evidence["profiles"].append({"id": profile["id"], "detector": kind, "model": model["xml"]})
        checked = check_configuration(client, build_id, profile["id"], kind)
        single = client.job("single_image", build_id=build_id, profile_id=profile["id"], image=image, iterations=3)
        single_results = client.request(f"/api/jobs/{single['id']}/results")
        measurements = [report["data"] for report in single_results["reports"] if report["kind"] == "timing"]
        if len(measurements) != 1 or any(not isinstance(measurements[0].get(key), (float, int)) or
                                       not math.isfinite(measurements[0][key]) or measurements[0][key] < 0
                                       for key in ("model_load_ms", "p50_ms", "p95_ms")):
            raise AssertionError("Single-image inference did not produce finite timings")
        replayed = check_replay(client, build_id, profile["id"], dataset)
        batch, reports = check_batch(client, evidence, directory, dataset,
                                     [{"build_id": build_id, "profile_id": profile["id"]}])
        if len(reports) != 1 or len(reports[0]["runs"]) != 1:
            raise AssertionError("Model batch did not produce one expected run")
        report, run = reports[0], reports[0]["runs"][0]
        if run.get("detector") != kind or Path(run["model_file"]["path"]).resolve() != Path(model["xml"]):
            raise AssertionError("Batch did not run the explicitly selected model")
        if run["model_file"]["bytes"] <= 0 or run["model_weights"]["bytes"] <= 0 or run["summary"]["frames"] != 6:
            raise AssertionError("Batch lacks model provenance or six-frame accounting")
        if report["pose_metrics"] != "not_produced" or any(run["summary"][key] is not None
            for key in ("mean_position_error_m", "mean_rotation_error_rad")):
            raise AssertionError("Unqualified synthetic pose metrics should remain unavailable")
        costs = yaml.safe_load(client.artifact_path(batch, "timing.yaml").read_text(encoding="utf-8"))
        inference = costs["runs"][0]["inference_wall"]
        if inference["samples"] <= 0 or inference["mean_ms"] is None:
            raise AssertionError("Batch did not record actual inference samples")
        evidence.setdefault("model_runs", {})[kind] = {"profile": profile["id"], "check_config": checked["id"],
            "single_image": single["id"], "replay": replayed["id"], "benchmark": batch["id"],
            "summary": run["summary"], "inference_wall": inference,
            "scope": "real model inference on synthetic inputs; no real-world accuracy conclusion"}

    combination = None
    if args.with_combination:
        combination = build(name + " I1+I2", {FLAGS[0], FLAGS[1]})
        across, _ = check_batch(client, evidence, directory, dataset, [
            {"build_id": build_id, "profile_id": first["id"]},
            {"build_id": combination, "profile_id": first["id"]},
        ])
        evidence["cross_comparison"] = across["id"]
    evidence["combination"] = combination


def verify(args):
    client = Client(args.url, args.timeout)
    context = client.request("/api/context")
    config = Path(context["source_root"]) / "config"
    before = config_inventory(config)
    models = supplied_models(args)
    directory = Path(context["workspace_root"]) / "acceptance_checks" / (
        time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    evidence = {"schema_version": 1, "result": "running", "url": args.url,
                "scope": "offline API and actual CLI execution; synthetic inputs do not establish model accuracy",
                "profiles": [], "config_before": before, "models_before": models,
                "evidence_file": str(directory / "evidence.json")}
    failure = None
    try:
        workflow(args, client, context, models, evidence, directory)
        evidence["result"] = "passed"
    except Exception as error:
        failure = error
        evidence.update(result="failed", error=str(error))
    finally:
        after = config_inventory(config)
        evidence["config_after"] = after
        evidence["source_configuration_unchanged"] = before == after
        models_after = {kind: dict(model, xml_sha256=sha256(Path(model["xml"])),
                                   bin_sha256=sha256(Path(model["bin"]))) for kind, model in models.items()}
        evidence["models_after"] = models_after
        evidence["models_unchanged"] = models_after == models
        evidence["tasks"] = client.jobs
        if before != after or models_after != models:
            mutation = "Workbench or direct CLI changed source configuration/model inputs"
            evidence.update(result="failed", readonly_error=mutation)
            if failure is None:
                failure = AssertionError(mutation)
        (directory / "evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
        summary = {key: evidence.get(key) for key in (
            "result", "baseline", "combination", "synthetic", "replay", "comparison",
            "source_configuration_unchanged", "models_unchanged", "evidence_file")}
        summary["model_runs"] = {kind: {key: run[key] for key in (
            "profile", "check_config", "single_image", "replay", "benchmark")}
            for kind, run in evidence.get("model_runs", {}).items()}
        if failure:
            summary["error"] = str(failure)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if failure:
        raise failure


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--build-id")
    parser.add_argument("--with-combination", action="store_true")
    parser.add_argument("--yolov5-model", type=Path)
    parser.add_argument("--yolo11-model", type=Path)
    parser.add_argument("--openvino-dir", type=Path)
    parser.add_argument("--timeout", type=float, default=3600,
                        help="Maximum seconds per job; cancels a job on timeout")
    verify(parser.parse_args())
