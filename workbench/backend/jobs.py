"""One durable, serial queue of argument-array subprocesses."""

import copy
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
import math

from .store import FLAGS, now, read_json, read_yaml, write_json
from .nuc import validate_target


KINDS = {"build", "check_config", "synthetic", "replay", "benchmark", "single_image", "ctest", "nuc_probe", "nuc_run"}
TERMINAL = {"succeeded", "failed", "cancelled", "interrupted"}


def integer(value, name, minimum=1, maximum=None):
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise ValueError(name + " must be an integer in the supported range")
    return value


class JobQueue:
    def __init__(self, store):
        self.store = store
        self.lock = store.lock
        self.jobs = {}
        self.pending = queue.Queue()
        self.stopping = threading.Event()
        self.thread = None
        self.process = None
        self.current = None
        self.remote_stops = {}
        for path in store.jobs_dir.glob("*/job.json"):
            job = read_json(path)
            if job["status"] in {"queued", "running"}:
                job.update(status="interrupted", finished_at=now(),
                           error="Server stopped before this task completed; not automatically rerun")
                write_json(path, job)
                with (path.parent / "log.txt").open("a", encoding="utf-8") as output:
                    output.write("\n[workbench] Previous unfinished task marked interrupted.\n")
            self.jobs[job["id"]] = job

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stopping.clear()
        self.thread = threading.Thread(target=self._worker, name="workbench-queue", daemon=True)
        self.thread.start()

    def stop(self):
        with self.lock:
            active = self.jobs.get(self.current)
        if active and active["kind"] == "nuc_run" and active["status"] == "running":
            try:
                self.cancel(active["id"])
            except ValueError as error:
                with (Path(active["directory"]) / "log.txt").open("a", encoding="utf-8") as log:
                    log.write("\n[workbench] Shutdown: " + str(error) + "\n")
        self.stopping.set()
        with self.lock:
            process = self.process
        if process:
            self._terminate(process)
        if self.thread:
            self.thread.join(timeout=6)
        with self.lock:
            for job in self.jobs.values():
                if job["status"] in {"queued", "running"}:
                    job.update(status="interrupted", finished_at=now(), error="Server shut down")
                    self._save(job)

    def _save(self, job):
        write_json(Path(job["directory"]) / "job.json", job)
        write_json(Path(job["directory"]) / "commands.json", job["commands"])

    def list(self):
        with self.lock:
            return copy.deepcopy(sorted(self.jobs.values(), key=lambda job: job["created_at"], reverse=True))

    def get(self, identifier):
        with self.lock:
            if identifier not in self.jobs:
                raise KeyError("Unknown job: " + identifier)
            return copy.deepcopy(self.jobs[identifier])

    def build(self, identifier, require_ready=True):
        job = self.get(identifier)
        if job["kind"] != "build" or (require_ready and job["status"] != "succeeded"):
            raise ValueError("Selected build must be a successful build task")
        return job

    def builds(self):
        builds = []
        for job in self.list():
            if job["kind"] != "build":
                continue
            root = Path(job["directory"]) / "acceptance"
            summary = read_json(root / "summary.json", {})
            builds.append({"id": job["id"], "name": job["options"]["name"],
                           "flags": job["options"]["flags"], "openvino": job["options"]["openvino"],
                           "build_type": job["options"]["build_type"], "status": job["status"],
                           "build_dir": str(root / "build"),
                           "information": read_json(root / "build_information.json", summary),
                           "models": summary.get("models", {}),
                           "model_acceptance": summary.get("model_acceptance", "not_verified")})
        return builds

    def _options(self, kind, options):
        if kind not in KINDS or not isinstance(options, dict):
            raise ValueError("Unsupported job kind or options")
        options = copy.deepcopy(options)
        if kind in {"nuc_probe", "nuc_run"}:
            options["target"] = validate_target(read_json(self.store.root / "nuc_target.json", {}),
                                                require_paths=kind == "nuc_run")
            if kind == "nuc_run":
                self.store.profile(options.get("profile_id", ""))
                flags = options.get("flags")
                if not isinstance(flags, dict) or set(flags) != set(FLAGS) or any(type(v) is not bool for v in flags.values()):
                    raise ValueError("NUC flags must explicitly specify all six boolean CMake names")
                if type(options.get("openvino")) is not bool:
                    raise ValueError("NUC openvino must be boolean")
                if options.get("build_type") not in {"Debug", "Release"}:
                    raise ValueError("NUC build_type must be Debug or Release")
                if flags["AUTOAIM_I3_LINEAR_CA"] and flags["AUTOAIM_USE_ESO"]:
                    raise ValueError("I3_LINEAR_CA and USE_ESO are mutually exclusive")
                if flags["AUTOAIM_I9_THROUGHPUT"] and not options["openvino"]:
                    raise ValueError("I9_THROUGHPUT requires OpenVINO")
        elif kind == "build":
            flags = options.get("flags", {flag: False for flag in FLAGS})
            if not isinstance(flags, dict) or set(flags) != set(FLAGS) or any(type(value) is not bool for value in flags.values()):
                raise ValueError("flags must explicitly specify all six boolean CMake names")
            options["flags"] = flags
            options.setdefault("openvino", False)
            if type(options["openvino"]) is not bool:
                raise ValueError("openvino must be boolean")
            if flags["AUTOAIM_I3_LINEAR_CA"] and flags["AUTOAIM_USE_ESO"]:
                raise ValueError("I3_LINEAR_CA and USE_ESO are mutually exclusive")
            if flags["AUTOAIM_I9_THROUGHPUT"] and not options["openvino"]:
                raise ValueError("I9_THROUGHPUT requires OpenVINO")
            options.setdefault("name", "Baseline")
            if not isinstance(options["name"], str) or not options["name"].strip():
                raise ValueError("Build name is required")
            options.setdefault("build_type", "Release")
            if options["build_type"] not in {"Debug", "Release"}:
                raise ValueError("build_type must be Debug or Release")
            options["jobs"] = integer(options.get("jobs", 2), "jobs")
            for key in ("yolov5_model", "yolo11_model"):
                if options.get(key):
                    model = self.store.allowed_path(options[key])
                    if model.suffix.lower() != ".xml":
                        raise ValueError("Model must be XML with adjacent BIN")
                    self.store.allowed_path(model.with_suffix(".bin"))
                    options[key] = str(model)
            if options.get("openvino_dir"):
                options["openvino_dir"] = str(self.store.allowed_path(options["openvino_dir"], directory=True))
            if (flags["AUTOAIM_I9_THROUGHPUT"] or flags["AUTOAIM_I9_PREALLOC"]):
                if not options["openvino"] or not all(options.get(key) for key in ("yolov5_model", "yolo11_model")):
                    raise ValueError("Both I9 alternatives require OpenVINO and both YOLO XML/BIN pairs for acceptance")
            if not options["openvino"] and any(options.get(key) for key in ("yolov5_model", "yolo11_model", "openvino_dir")):
                raise ValueError("OpenVINO model options require an OpenVINO build")
        elif kind == "benchmark":
            runs = options.get("runs")
            if not isinstance(runs, list) or not runs:
                raise ValueError("benchmark requires at least one run")
            for run in runs:
                if not isinstance(run, dict):
                    raise ValueError("Each run requires build_id and profile_id")
                self.build(run.get("build_id", ""))
                self.store.profile(run.get("profile_id", ""))
            iou = options.get("iou", 0.5)
            if type(iou) not in (float, int) or not math.isfinite(iou) or not 0 < iou <= 1:
                raise ValueError("iou must be a finite number in (0,1]")
            options["iou"] = iou
            conditions = [options.get(key) is not None and options.get(key) != ""
                          for key in ("pose_reference", "position_limit", "rotation_limit")]
            if any(conditions) and not all(conditions):
                raise ValueError("Pose comparison requires reference and both uncertainty limits")
            if all(conditions):
                if not isinstance(options["pose_reference"], str):
                    raise ValueError("Pose reference must be a nonempty string")
                for key in ("position_limit", "rotation_limit"):
                    value = options[key]
                    if type(value) not in (float, int) or not math.isfinite(value) or value < 0:
                        raise ValueError(key + " must be a finite nonnegative number")
        else:
            self.build(options.get("build_id", ""))
            if kind != "ctest":
                self.store.profile(options.get("profile_id", ""))
            if kind == "synthetic":
                options["frames"] = integer(options.get("frames", 6), "frames", 1, 10000)
            if kind == "single_image":
                options["iterations"] = integer(options.get("iterations", 100), "iterations", 1, 10000)
                options["image"] = str(self.store.allowed_path(options.get("image", "")))
        if kind in {"replay", "benchmark"}:
            options["dataset"] = str(self.store.allowed_path(options.get("dataset", "")))
        return options

    def submit(self, kind, options):
        with self.lock:
            options = self._options(kind, options)
            identifier = uuid.uuid4().hex
            directory = self.store.jobs_dir / identifier
            directory.mkdir()
            job = {"id": identifier, "kind": kind, "options": options, "status": "queued",
                   "created_at": now(), "started_at": None, "finished_at": None,
                   "directory": str(directory), "commands": [], "snapshots": {},
                   "cancel_requested": False, "error": None}
            profiles = [run["profile_id"] for run in options["runs"]] if kind == "benchmark" else (
                [options["profile_id"]] if "profile_id" in options else [])
            for profile_id in dict.fromkeys(profiles):
                profile = self.store.profile(profile_id)
                destination = directory / "configurations" / profile_id
                shutil.copytree(Path(profile["config_path"]).parent, destination)
                write_json(destination / "profile.json", profile)
                job["snapshots"][profile_id] = str(destination / "fast_choose.yaml")
            (directory / "log.txt").touch()
            if kind == "build":
                write_json(directory / "selection.json", {"schema_version": 1,
                           "options": options["flags"], "openvino": options["openvino"]})
            if kind in {"nuc_probe", "nuc_run"}:
                request = {"job_id": identifier, "target": options["target"]}
                if kind == "nuc_run":
                    request["profile_values"] = read_yaml(Path(job["snapshots"][options["profile_id"]]))
                    request["selection"] = {key: options[key] for key in ("flags", "openvino", "build_type")}
                write_json(directory / "nuc_request.json", request)
            self.jobs[identifier] = job
            self._save(job)
            self.pending.put(identifier)
            return copy.deepcopy(job)

    def cancel(self, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if job is None:
                raise KeyError("Unknown job")
            remote_running = job["kind"] == "nuc_run" and job["status"] == "running"
            if remote_running:
                if identifier in self.remote_stops:
                    raise ValueError("NUC 停止正在确认，请稍候。")
                stop_event = threading.Event()
                self.remote_stops[identifier] = stop_event
            if job["status"] not in TERMINAL and not remote_running:
                job["cancel_requested"] = True
                if job["status"] == "queued":
                    job.update(status="cancelled", finished_at=now())
                self._save(job)
                process = self.process if self.current == identifier else None
            else:
                process = None
        if remote_running:
            # Keep it running until the remote process group has acknowledged termination.
            try:
                self._cancel_nuc(job)
                with self.lock:
                    if job["status"] not in TERMINAL:
                        job["cancel_requested"] = True
                        self._save(job)
                    process = self.process if self.current == identifier else None
            finally:
                stop_event.set()
                with self.lock:
                    self.remote_stops.pop(identifier, None)
        if process:
            self._terminate(process)
        return self.get(identifier)

    @staticmethod
    def _terminate(process):
        if process.poll() is not None:
            return
        if os.name == "nt":
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            process.wait(timeout=5)
            return
        try:
            # The supported execution host is Linux/WSL; descendants share this session.
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
            else:
                # A leader can exit while a descendant ignores TERM.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        except ProcessLookupError:
            pass

    def _binary(self, build_id, name):
        build = self.build(build_id)
        root = Path(build["directory"]) / "acceptance/build"
        matches = [root / name, root / "bin" / name,
                   root / build["options"]["build_type"] / name]
        matches += sorted(root.glob("**/" + name))
        for candidate in matches:
            if candidate.is_file():
                return str(candidate.resolve())
        raise ValueError("Selected build has no tool: " + name)

    def command_plan(self, job):
        options, root = job["options"], Path(job["directory"])
        kind = job["kind"]
        config = job["snapshots"].get(options.get("profile_id"))
        tool = lambda name: self._binary(options["build_id"], name)
        if kind in {"nuc_probe", "nuc_run"}:
            return [(kind, [sys.executable, "-B", "-m", "workbench.backend.nuc",
                           "probe" if kind == "nuc_probe" else "run", str(root / "nuc_request.json")], None)]
        if kind == "build":
            argv = [sys.executable, "-B", str(self.store.source / "tests/verify_alternatives.py"),
                    "--source", str(self.store.source), "--selection-file", str(root / "selection.json"),
                    "--output", str(root / "acceptance"), "--build-type", options["build_type"],
                    "--jobs", str(options["jobs"])]
            for key, flag in (("yolov5_model", "--yolov5-model"), ("yolo11_model", "--yolo11-model"),
                              ("openvino_dir", "--openvino-dir")):
                if options.get(key):
                    argv += [flag, options[key]]
            return [("build_acceptance", argv, None)]
        if kind == "ctest":
            build = self.build(options["build_id"])
            return [("ctest", [shutil.which("ctest") or "ctest", "--test-dir",
                                str(Path(build["directory"]) / "acceptance/build"), "-C",
                                build["options"]["build_type"], "--output-on-failure"], None)]
        if kind == "check_config":
            return [("check_config", [tool("autoaim_node"), "--check-config", "--config", config],
                     root / "check_config.yaml")]
        if kind == "synthetic":
            return [("synthetic", [tool("synthetic_sim"), "--config", config, "--output",
                                   str(root / "dataset"), "--frames", str(options["frames"])], None)]
        if kind == "single_image":
            return [("single_image", [tool("bench_detector"), "--config", config, "--image",
                                      options["image"], "--iterations", str(options["iterations"])],
                     root / "single_image.txt")]
        if kind == "replay":
            fire_age = read_yaml(Path(config))["common"]["safety"]["fire_age_s"]
            return [("offline_replay", [tool("offline_replay"), "--config", config, "--input", options["dataset"],
                      "--output", str(root / "commands.tsv"), "--uart-output", str(root / "uart.hex"),
                      "--record-session", str(root / "recording")], None),
                    ("replay_visualizer", [tool("replay_visualizer"), "--config", config,
                      "--input", options["dataset"], "--output", str(root / "visualized")], None),
                    ("pipeline_metrics", [tool("pipeline_metrics"), str(root / "commands.tsv"),
                      "--fire-age-s", str(fire_age)], root / "pipeline_metrics.txt")]
        groups = {}
        for run in options["runs"]:
            groups.setdefault(run["build_id"], []).append(run)
        plans = []
        for index, (build_id, runs) in enumerate(groups.items()):
            argv = [self._binary(build_id, "bench_detector"), "--dataset", options["dataset"]]
            for run in runs:
                argv += ["--config", job["snapshots"][run["profile_id"]]]
            argv += ["--iou", str(options["iou"]), "--output", str(root / ("benchmark_" + str(index)))]
            if options.get("pose_reference"):
                argv += ["--pose-reference", options["pose_reference"],
                         "--pose-position-limit-m", str(options["position_limit"]),
                         "--pose-rotation-limit-rad", str(options["rotation_limit"])]
            plans.append(("benchmark_" + str(index), argv, None))
        return plans

    def _environment(self):
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8")
        root = str(Path(__file__).resolve().parents[2])
        environment["PYTHONPATH"] = os.pathsep.join(filter(None, (root, environment.get("PYTHONPATH"))))
        return environment

    def _cancel_nuc(self, job):
        root = Path(job["directory"])
        argv = [sys.executable, "-B", "-m", "workbench.backend.nuc", "cancel", str(root / "nuc_request.json")]
        record = {"name": "nuc_cancel", "argv": argv, "cwd": str(root), "exit_code": None,
                  "started_at": now(), "finished_at": None}
        try:
            result = subprocess.run(argv, cwd=root, env=self._environment(), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, timeout=20, check=False)
            record["exit_code"] = result.returncode
            with (root / "log.txt").open("ab") as log:
                log.write(b"\n[workbench] Confirm remote stop:\n" + result.stdout)
            report = read_json(root / "nuc_cancel_report.json", {})
            if result.returncode or report.get("action") != "cancel" or report.get("status") != "cancelled":
                raise ValueError("NUC 停止尚未确认；请检查 SSH 连接及远端运行状态。")
        except subprocess.TimeoutExpired as error:
            raise ValueError("NUC 停止确认超时；远端运行状态未知。") from error
        finally:
            record["finished_at"] = now()
            with self.lock:
                job["commands"].append(record)
                self._save(job)

    def _run(self, job, label, argv, stdout_path):
        root = Path(job["directory"])
        record = {"name": label, "argv": list(argv), "cwd": str(root),
                  "exit_code": None, "started_at": now(), "finished_at": None}
        with self.lock:
            if job["cancel_requested"] or self.stopping.is_set():
                raise InterruptedError("Task cancelled or server stopping")
            job["commands"].append(record)
            self._save(job)
        with (root / "log.txt").open("ab", buffering=0) as log:
            log.write(("\n[workbench] " + label + ": " + repr(argv) + "\n").encode("utf-8"))
            separate = Path(stdout_path).open("wb") if stdout_path else None
            environment = self._environment()
            try:
                # No shell interpolation; tools receive exactly the persisted argv.
                with self.lock:
                    if job["cancel_requested"] or self.stopping.is_set():
                        raise InterruptedError("Task cancelled or server stopping")
                    process = subprocess.Popen(argv, cwd=root, stdout=separate or log, stderr=log,
                                               env=environment, start_new_session=True)
                    self.process = process
                while process.poll() is None:
                    if job["cancel_requested"] or self.stopping.is_set():
                        self._terminate(process)
                    time.sleep(0.05)
                record["exit_code"] = process.returncode
            finally:
                if separate:
                    separate.close()
                    log.write(Path(stdout_path).read_bytes())
                record["finished_at"] = now()
                with self.lock:
                    self.process = None
                    self._save(job)
        if record["exit_code"] != 0:
            raise RuntimeError(label + " exited with code " + str(record["exit_code"]))

    def _worker(self):
        while not self.stopping.is_set():
            try:
                identifier = self.pending.get(timeout=0.2)
            except queue.Empty:
                continue
            job = self.jobs[identifier]
            with self.lock:
                if job["status"] != "queued":
                    self.pending.task_done()
                    continue
                job.update(status="running", started_at=now())
                self.current = identifier
                self._save(job)
            try:
                for label, argv, stdout_path in self.command_plan(job):
                    self._run(job, label, argv, stdout_path)
                outcome, error_message = "succeeded", None
            except Exception as error:
                outcome, error_message = "failed", str(error)
                with (Path(job["directory"]) / "log.txt").open("a", encoding="utf-8") as log:
                    log.write("\n[workbench] " + str(error) + "\n")
            finally:
                with self.lock:
                    stop_event = self.remote_stops.get(identifier)
                if stop_event:
                    stop_event.wait(timeout=25)
                with self.lock:
                    job.update(status=outcome, error=error_message)
                    if self.stopping.is_set():
                        job["status"] = "interrupted"
                    elif job["cancel_requested"]:
                        job["status"] = "cancelled"
                    job["finished_at"] = now()
                    if job["kind"] == "build":
                        summary = read_json(Path(job["directory"]) / "acceptance/summary.json", {})
                        for nested in summary.get("commands", []):
                            record = copy.deepcopy(nested)
                            record["name"] = "acceptance/" + record.get("name", "command")
                            job["commands"].append(record)
                    self._save(job)
                    self.current = None
                self.pending.task_done()
