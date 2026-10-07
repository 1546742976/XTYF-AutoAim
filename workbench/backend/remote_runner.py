"""Standalone NUC helper sent over SSH stdin, never installed in the project.

It requires Python 3 and PyYAML on the NUC. Probe only reads files. Run creates
one new workspace/jobs/<id> directory, checks the hardware configuration, and
owns the program's process group until every child is stopped.
"""

import copy
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import threading
import time


FLAGS = ("AUTOAIM_I1_CONTRAST_IRLS", "AUTOAIM_I2_LM", "AUTOAIM_I3_LINEAR_CA",
         "AUTOAIM_I9_THROUGHPUT", "AUTOAIM_I9_PREALLOC", "AUTOAIM_USE_ESO")
DETECTORS = ("traditional", "yolov5", "yolo11")
EVENT_PREFIX = "NUC_EVENT "


def _now():
    return datetime.now(timezone.utc).isoformat()


def _write_json(path, value):
    temporary = path.with_name(path.name + "." + str(os.getpid()) + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def _read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _yaml(text):
    import yaml

    class UniqueLoader(yaml.SafeLoader):
        def construct_mapping(self, node, deep=False):
            self.flatten_mapping(node)
            keys = set()
            for key_node, _ in node.value:
                key = self.construct_object(key_node, deep=True)
                if key in keys:
                    raise ValueError("Duplicate YAML key: " + str(key))
                keys.add(key)
            return super().construct_mapping(node, deep=deep)

    value = yaml.load(text, Loader=UniqueLoader)
    if not isinstance(value, dict):
        raise ValueError("Configuration/check report must be a YAML mapping")
    return value


def _read_yaml(path):
    return _yaml(path.read_text(encoding="utf-8"))


def _write_yaml(path, value):
    import yaml
    path.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8")


def _job_id(request):
    job_id = request.get("job_id", "")
    if not isinstance(job_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", job_id):
        raise ValueError("job_id must be a simple unique identifier")
    return job_id


def _target(request, require_paths=False):
    target = request.get("target")
    if not isinstance(target, dict):
        raise ValueError("target must be an object")
    for key in ("project_dir", "build_dir", "device_config", "workspace_dir"):
        value = target.get(key) or ""
        if require_paths and not value:
            raise ValueError(key + " is required")
        if value and (not isinstance(value, str) or not value.startswith("/") or "\x00" in value):
            raise ValueError(key + " must be an absolute NUC path")
    return target


def _program(target):
    path = Path(target.get("executable") or "autoaim_node")
    if not path.is_absolute():
        if ".." in path.parts:
            raise ValueError("executable must stay inside build_dir")
        path = Path(target["build_dir"]) / path
    path = path.resolve()
    build = Path(target["build_dir"]).resolve()
    if path == build or build not in path.parents:
        raise ValueError("NUC executable must belong to the selected build_dir")
    return path


def _workspace(target):
    workspace = Path(target["workspace_dir"]).resolve()
    for key in ("project_dir", "build_dir"):
        source = Path(target[key]).resolve()
        if workspace == source or workspace in source.parents or source in workspace.parents:
            raise ValueError("NUC workspace_dir must be independent of " + key)
    return workspace


def _metadata(target):
    return _read_json(Path(target["build_dir"]) / "generated" / "autoaim_build_information.json")


def _boolean(value, name):
    if type(value) is bool:
        return value
    if isinstance(value, str) and value.upper() in ("ON", "OFF", "TRUE", "FALSE", "1", "0"):
        return value.upper() in ("ON", "TRUE", "1")
    raise ValueError("Missing/invalid build information: " + name)


def _check_selection(selection, metadata):
    if not isinstance(selection, dict) or not isinstance(selection.get("flags"), dict):
        raise ValueError("selection must contain six boolean flags, openvino, and build_type")
    flags = selection["flags"]
    if set(flags) != set(FLAGS) or any(type(flags[k]) is not bool for k in FLAGS):
        raise ValueError("selection.flags must contain exactly the six boolean CMake flags")
    if type(selection.get("openvino")) is not bool:
        raise ValueError("selection.openvino must be boolean")
    if selection.get("build_type") not in ("Debug", "Release"):
        raise ValueError("selection.build_type must be Debug or Release")
    mismatches = [key for key in FLAGS if _boolean(metadata.get(key), key) != flags[key]]
    if _boolean(metadata.get("openvino"), "openvino") != selection["openvino"]:
        mismatches.append("openvino")
    if metadata.get("build_type") != selection["build_type"]:
        mismatches.append("build_type")
    if mismatches:
        raise ValueError("NUC build does not match selected build: " + ", ".join(mismatches))
    if flags["AUTOAIM_I3_LINEAR_CA"] and flags["AUTOAIM_USE_ESO"]:
        raise ValueError("I3 and ESO cannot be enabled together")
    if flags["AUTOAIM_I9_THROUGHPUT"] and not selection["openvino"]:
        raise ValueError("I9 throughput requires OpenVINO")


def _absolute(value, owner):
    if not isinstance(value, str) or not value or "\x00" in value:
        raise ValueError("Device/template file reference must be a nonempty path")
    path = Path(value)
    return str(path if path.is_absolute() else (owner.parent / path).resolve())


def _remove_leaves(target, migrated):
    """Replace only declared fast_choose leaves; preserve hardware/contract fields."""
    for key, value in migrated.items():
        if isinstance(value, dict):
            if isinstance(target.get(key), dict):
                _remove_leaves(target[key], value)
                if not target[key]:
                    del target[key]
        else:
            target.pop(key, None)


def _prepare_config(request, directory):
    target = request["target"]
    device_path = Path(target["device_config"])
    device = _read_yaml(device_path)
    if device.get("execution") != "hardware":
        raise ValueError("device_config.execution must be hardware; replay cannot be run as a live device")
    template_path = Path(target["project_dir"]) / "config" / "fast_choose.yaml"
    template = _read_yaml(template_path)
    profile = copy.deepcopy(request.get("profile_values"))
    if not isinstance(profile, dict) or profile.get("active_detector") not in DETECTORS:
        raise ValueError("profile_values must contain a selected fast_choose detector")
    selected = profile["active_detector"]
    if not isinstance(profile.get("common"), dict) or not isinstance(profile.get("detectors"), dict):
        raise ValueError("profile_values must contain common and detectors maps")
    if set(profile["detectors"]) != set(DETECTORS):
        raise ValueError("profile_values must contain all three detector maps")
    # The Windows profile's paths must never be interpreted as NUC paths.
    # Device input references and NUC template/model paths are the remote authority.
    profile["common"]["input_manifest"] = _absolute(
        device.get("input_manifest", template["common"]["input_manifest"]),
        device_path if "input_manifest" in device else template_path)
    for kind in DETECTORS:
        profile["detectors"][kind]["config_file"] = "hardware.yaml"
        if kind != "traditional":
            model = target.get(kind + "_model") or _absolute(template["detectors"][kind]["model_path"], template_path)
            if not isinstance(model, str) or not model.startswith("/"):
                raise ValueError(kind + "_model must be an absolute NUC path")
            profile["detectors"][kind]["model_path"] = model
    if selected != "traditional":
        if not request["selection"]["openvino"]:
            raise ValueError("Selected YOLO detector requires an OpenVINO NUC build")
        model = Path(profile["detectors"][selected]["model_path"])
        if not model.is_file() or (model.suffix.lower() == ".xml" and not model.with_suffix(".bin").is_file()):
            raise ValueError("Selected NUC model/weights do not exist: " + str(model))
    base = copy.deepcopy(device)
    # Legacy hardware configurations can still contain tuning leaves. Their
    # explicit profile replacement prevents C++ composition duplicate conflicts.
    _remove_leaves(base, template["common"])
    _remove_leaves(base, {"eso": template.get("eso", {})})
    for kind in DETECTORS:
        migrated = {k: v for k, v in template["detectors"][kind].items() if k != "config_file"}
        _remove_leaves(base, {"detector": migrated})
    base.setdefault("detector", {})["kind"] = selected
    base["fast_choose_file"] = "fast_choose.yaml"
    for key in ("calibration_file",):
        if key in base:
            base[key] = _absolute(base[key], device_path)
    if "geometry_files" in base:
        if not isinstance(base["geometry_files"], list):
            raise ValueError("device geometry_files must be a list")
        base["geometry_files"] = [_absolute(value, device_path) for value in base["geometry_files"]]
    # Preserve camera/serial fields exactly, including /dev paths and physical
    # evidence references. Do not substitute synthetic contracts from Windows.
    config_dir = directory / "config"
    config_dir.mkdir()
    _write_yaml(config_dir / "hardware.yaml", base)
    _write_yaml(config_dir / "fast_choose.yaml", profile)
    _write_json(directory / "request.json", request)
    return config_dir / "fast_choose.yaml"


def _pid_identity(pid):
    try:
        # Process names may contain spaces or parentheses; split after last ')'.
        values = Path("/proc", str(pid), "stat").read_text().rsplit(")", 1)[1].split()
        return values[19]  # field 22, after fields pid and comm
    except (OSError, IndexError):
        return None


def _group_alive(pgid):
    # Ignore zombies: they cannot execute or access devices, and a different
    # parent may be responsible for reaping grandchildren.
    proc = Path("/proc")
    if proc.is_dir():
        for entry in proc.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) == pgid and fields[0] != "Z":
                    return True
            except (OSError, ValueError, IndexError):
                pass
        return False
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def _stop_group(pgid, process=None):
    if not isinstance(pgid, int) or pgid <= 1 or pgid == os.getpgrp():
        raise ValueError("Refusing an invalid/bridge process group")
    for sig, timeout in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process is not None:
                process.poll()
            if not _group_alive(pgid):
                if process is not None:
                    process.wait(timeout=1)
                return True
            time.sleep(0.05)
    return not _group_alive(pgid)


def _report(action, request, **values):
    result = {"schema_version": 1, "action": action, "job_id": request.get("job_id"),
              "status": "failed", "ready": False, "updated_at": _now()}
    result.update(values)
    return result


@contextmanager
def _launch_lock(directory):
    # Both cancellation and each Popen use the same remote file lock. An
    # acknowledged cancellation can therefore never race a subsequent launch.
    import fcntl
    with (directory / "launch.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def _collect_check(process, directory, emit, output, timeout=30):
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ, "stdout")
    selector.register(process.stderr, selectors.EVENT_READ, "stderr")
    captured = {"stdout": bytearray(), "stderr": bytearray()}
    deadline = time.monotonic() + timeout
    last_heartbeat = 0.0
    try:
        while True:
            if (directory / "cancel_requested").exists():
                raise _Interrupted("Cancellation requested during configuration check")
            if time.monotonic() > deadline:
                raise ValueError("NUC --check-config timed out")
            ready = selector.select(timeout=0.2)
            for key, _ in ready:
                data = os.read(key.fileobj.fileno(), 65536)
                if data:
                    captured[key.data].extend(data)
                    output(data)
                else:
                    selector.unregister(key.fileobj)
            if time.monotonic() - last_heartbeat >= 1:
                emit({"event": "heartbeat"})
                last_heartbeat = time.monotonic()
            if process.poll() is not None:
                # Drain buffered output without waiting for stray descendants
                # that may still hold a pipe; the owner cleans their group.
                while True:
                    ready = selector.select(timeout=0)
                    if not ready:
                        break
                    for key, _ in ready:
                        data = os.read(key.fileobj.fileno(), 65536)
                        if data:
                            captured[key.data].extend(data)
                            output(data)
                        else:
                            selector.unregister(key.fileobj)
                break
    finally:
        selector.close()
    return bytes(captured["stdout"]), bytes(captured["stderr"])


def probe(request):
    target = _target(request)
    info = {"uname": list(os.uname()), "files": {}, "missing": [], "diagnostics": []}
    for key in ("project_dir", "build_dir", "device_config", "workspace_dir"):
        value = target.get(key)
        if not value:
            info["missing"].append(key)
        else:
            path = Path(value)
            info["files"][key] = {"path": str(path), "exists": path.exists(), "directory": path.is_dir()}
            # Workspace can be created later; all other supplied paths must exist.
            if key != "workspace_dir" and not path.exists():
                info["diagnostics"].append(key + " does not exist")
    if target.get("build_dir"):
        try:
            info["build_information"] = _metadata(target)
        except (OSError, ValueError) as error:
            info["diagnostics"].append("Build metadata unavailable: " + str(error))
        try:
            executable = _program(target)
            info["files"]["executable"] = {"path": str(executable), "exists": executable.is_file(),
                                            "executable": os.access(executable, os.X_OK)}
            if not executable.is_file() or not os.access(executable, os.X_OK):
                info["diagnostics"].append("NUC executable is absent or not executable")
        except ValueError as error:
            info["diagnostics"].append(str(error))
    if all(target.get(key) for key in ("workspace_dir", "project_dir", "build_dir")):
        try:
            _workspace(target)
        except ValueError as error:
            info["diagnostics"].append(str(error))
    if target.get("device_config"):
        try:
            info["device_execution"] = _read_yaml(Path(target["device_config"])).get("execution")
            if info["device_execution"] != "hardware":
                info["diagnostics"].append("device_config.execution is not hardware")
        except (OSError, ValueError, ImportError) as error:
            info["diagnostics"].append("Device configuration unavailable: " + str(error))
    ready = not info["missing"] and not info["diagnostics"]
    return _report("probe", request, status="ready" if ready else "incomplete", ready=ready, **info)


class _Interrupted(Exception):
    pass


def run(request, emit):
    target = _target(request, require_paths=True)
    job_id = _job_id(request)
    directory = _workspace(target) / "jobs" / job_id
    # A request cannot replace an earlier remote job or its configuration.
    directory.mkdir(parents=True, exist_ok=False)
    state_path = directory / "state.json"
    state = _report("run", request, status="checking", remote_directory=str(directory), commands=[])
    _write_json(state_path, state)
    child = None
    old_handlers = {}

    def interrupted(signum, frame):
        raise _Interrupted("Remote helper received signal " + str(signum))

    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            old_handlers[sig] = signal.signal(sig, interrupted)

    def persist():
        state["updated_at"] = _now()
        _write_json(state_path, state)
        _write_json(directory / "commands.json", state["commands"])

    def register(process, command):
        state.update(pid=process.pid, pgid=process.pid, pid_starttime=_pid_identity(process.pid))
        state["commands"].append(command)
        persist()

    with (directory / "run.log").open("ab", buffering=0) as log:
        def output(data):
            if not data:
                return
            log.write(data)
            emit({"event": "log", "text": data.decode("utf-8", errors="replace")})

        try:
            if not Path(target["project_dir"]).is_dir() or not Path(target["build_dir"]).is_dir():
                raise ValueError("NUC project_dir and build_dir must exist")
            program = _program(target)
            if not program.is_file() or not os.access(program, os.X_OK):
                raise ValueError("NUC executable does not exist or is not executable: " + str(program))
            metadata = _metadata(target)
            state["build_information"] = metadata
            _check_selection(request.get("selection"), metadata)
            config = _prepare_config(request, directory)
            state["config_path"] = str(config)
            check_argv = [str(program), "--check-config", "--config", str(config)]
            command = {"argv": check_argv, "cwd": target["project_dir"], "exit_code": None, "kind": "check_config"}
            with _launch_lock(directory):
                if (directory / "cancel_requested").exists():
                    raise _Interrupted("Cancellation recorded before configuration check")
                child = subprocess.Popen(check_argv, cwd=target["project_dir"], stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE, start_new_session=True)
                register(child, command)
            stdout, stderr = _collect_check(child, directory, emit, output)
            command["exit_code"] = child.returncode
            (directory / "check_config.yaml").write_bytes(stdout)
            if (directory / "cancel_requested").exists():
                raise _Interrupted("Cancellation requested during configuration check")
            if child.returncode:
                raise ValueError("NUC --check-config rejected the hardware configuration (exit " + str(child.returncode) + ")")
            check = _yaml(stdout.decode("utf-8"))
            state["check_report"] = check
            if check.get("check_schema_version") != 1 or check.get("valid") is not True:
                raise ValueError("NUC --check-config did not report schema 1 and valid=true")
            effective = check.get("effective_configuration")
            if not isinstance(effective, dict) or effective.get("execution") != "hardware":
                raise ValueError("NUC --check-config must report effective_configuration.execution=hardware; offline entry refused")
            # Even a successful checker must not leave an auxiliary process alive.
            _stop_group(child.pid, child)
            child.stdout.close()
            child.stderr.close()
            child = None
            if (directory / "cancel_requested").exists():
                raise _Interrupted("Cancellation requested before hardware launch")
            argv = [str(program), "--config", str(config)]
            command = {"argv": argv, "cwd": target["project_dir"], "exit_code": None, "kind": "hardware_run"}
            with _launch_lock(directory):
                if (directory / "cancel_requested").exists():
                    raise _Interrupted("Cancellation recorded before hardware launch")
                child = subprocess.Popen(argv, cwd=target["project_dir"], stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, start_new_session=True)
                state.update(status="running", ready=True)
                register(child, command)
            emit({"event": "running", "pid": child.pid, "pgid": child.pid, "remote_directory": str(directory)})
            selector = selectors.DefaultSelector()
            selector.register(child.stdout, selectors.EVENT_READ)
            last_heartbeat = 0.0
            try:
                while True:
                    if (directory / "cancel_requested").exists():
                        raise _Interrupted("Remote cancellation requested")
                    for key, _ in selector.select(timeout=0.2):
                        data = os.read(key.fileobj.fileno(), 65536)
                        if data:
                            output(data)
                        else:
                            selector.unregister(key.fileobj)
                    # A silent program must still detect the broken SSH output.
                    if time.monotonic() - last_heartbeat >= 1:
                        emit({"event": "heartbeat"})
                        last_heartbeat = time.monotonic()
                    if child.poll() is not None:
                        # The main process may leave descendants holding stdout.
                        # Cleanup below stops them instead of waiting indefinitely.
                        while True:
                            pending = selector.select(timeout=0)
                            if not pending:
                                break
                            for key, _ in pending:
                                data = os.read(key.fileobj.fileno(), 65536)
                                if data:
                                    output(data)
                                else:
                                    selector.unregister(key.fileobj)
                        break
            finally:
                selector.close()
            command["exit_code"] = child.returncode
            state["status"] = "succeeded" if child.returncode == 0 else "failed"
            if child.returncode:
                state["error"] = "NUC program exited with code " + str(child.returncode)
        except _Interrupted as error:
            state.update(status="cancelled", error=str(error))
        except (Exception, BrokenPipeError) as error:
            state.update(status="failed", ready=False, error=str(error))
        finally:
            # Run owns this group on success, failure, signal, and SSH disconnect.
            # Temporarily ignore repeat termination while completing cleanup.
            for sig in old_handlers:
                signal.signal(sig, signal.SIG_IGN)
            if child is not None:
                stopped = _stop_group(child.pid, child)
                state["stopped"] = stopped
                if state["commands"]:
                    state["commands"][-1]["exit_code"] = child.poll()
                if not stopped:
                    state.update(status="failed", ready=False, error="NUC child process group did not stop")
                for pipe in (child.stdout, child.stderr):
                    if pipe is not None:
                        pipe.close()
            state["finished_at"] = _now()
            persist()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    return state


def cancel(request):
    target = _target(request)
    if not target.get("workspace_dir"):
        raise ValueError("workspace_dir is required for cancellation")
    directory = Path(target["workspace_dir"]) / "jobs" / _job_id(request)
    # An absent job cannot acknowledge cancellation: SSH may not yet have
    # delivered the run payload. The caller must keep the original run alive.
    if not (directory / "state.json").is_file():
        raise ValueError("Remote job is not created yet; cancellation is not acknowledged")
    with _launch_lock(directory):
        state = _read_json(directory / "state.json")
        try:
            (directory / "cancel_requested").touch(exist_ok=False)
        except FileExistsError:
            pass
        pgid = state.get("pgid")
        pid = state.get("pid")
        if pgid is None:
            # No child existed while the lock was held. The persistent marker
            # forbids both the check subprocess and hardware subprocess later.
            return _report("cancel", request, status="cancelled", stopped=True,
                           remote_directory=str(directory), message="Cancellation recorded before process launch")
        identity = _pid_identity(pid)
        if identity is not None and state.get("pid_starttime") != identity:
            raise ValueError("Remote PID has been reused; refusing to signal an unrelated process")
        if state.get("status") not in ("checking", "running"):
            if _group_alive(pgid):
                raise ValueError("Finished remote job has an unverified live group; refusing to signal it")
            stopped = True
        else:
            stopped = _stop_group(pgid)
    # Wait for the owning helper to reap its child and publish final state too.
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        current = _read_json(directory / "state.json")
        if current.get("status") not in ("checking", "running"):
            break
        time.sleep(0.05)
    return _report("cancel", request, status="cancelled" if stopped else "failed", stopped=stopped,
                   pid=pid, pgid=pgid, remote_directory=str(directory),
                   error=None if stopped else "Remote process group did not stop")


def dispatch(action, request, emit=None):
    """Run one helper action; emit receives protocol event dictionaries."""
    emit = emit or (lambda event: None)
    try:
        if action == "probe":
            return probe(request)
        if action == "run":
            return run(request, emit)
        if action == "cancel":
            return cancel(request)
        raise ValueError("Unknown helper action: " + str(action))
    except Exception as error:
        return _report(action, request, error=str(error))


def _emit(event):
    print(EVENT_PREFIX + json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)


def main(action=None, request=None):
    if action is None:
        import argparse
        parser = argparse.ArgumentParser(description="NUC SSH helper (normally supplied over SSH stdin)")
        parser.add_argument("action", choices=("run", "probe", "cancel"))
        parser.add_argument("request", type=Path)
        args = parser.parse_args()
        action, request = args.action, _read_json(args.request)
    report = dispatch(action, request, _emit)
    try:
        _emit({"event": "report", "report": report})
    except BrokenPipeError:
        # The run action already stopped its child group in finally.
        pass
    return_code = 0 if action == "probe" or report.get("status") in ("succeeded", "cancelled") else 1
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()
