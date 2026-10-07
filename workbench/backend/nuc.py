"""SSH transport for the NUC bridge; configuration never becomes shell input.

Public API: validate_target(mapping, require_paths=False), build_ssh_argv(target),
execute_request(action, request_path). Run/probe save nuc_report.json alongside
REQUEST.json; cancel saves nuc_cancel_report.json independently. Remote program
output is forwarded to stdout as it arrives.
"""

import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess
import sys


BOOTSTRAP = (
    "import json,sys; sys.dont_write_bytecode=True; "
    "e=json.load(sys.stdin); n={'__name__':'nuc_remote_runner'}; "
    "exec(compile(e['source'],'<nuc_remote_runner>','exec'),n); "
    "n['main'](e['action'],e['request'])"
)
EVENT_PREFIX = "NUC_EVENT "
REPORT_PREFIX = "NUC_REPORT "
REQUIRED_PATHS = ("project_dir", "build_dir", "device_config", "workspace_dir")


def _remote_path(value, key, required=False):
    if value is None or value == "":
        if required:
            raise ValueError(key + " is required for NUC execution")
        return ""
    if not isinstance(value, str) or any(c in value for c in "\x00\r\n\\"):
        raise ValueError(key + " must be a POSIX path")
    if not value.startswith("/"):
        raise ValueError(key + " must be an absolute NUC POSIX path")
    return str(PurePosixPath(value))


def validate_target(mapping, require_paths=False):
    """Return a normalized target, allowing incomplete paths for saving/probing."""
    if not isinstance(mapping, dict):
        raise ValueError("NUC target must be an object")
    host = mapping.get("host", "")
    user = mapping.get("user", "")
    # No whitespace, ssh options, user@host fragments, or shell metacharacters.
    if not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:%_\[\]-]*", host):
        raise ValueError("host must be a hostname or IP address without SSH options")
    if not isinstance(user, str) or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", user):
        raise ValueError("user must be a login name without SSH options or whitespace")
    port = mapping.get("port", 22)
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError("port must be an integer from 1 to 65535")
    identity = mapping.get("identity_file") or ""
    if not isinstance(identity, str) or any(c in identity for c in "\x00\r\n"):
        raise ValueError("identity_file must be a local file path")
    if identity:
        key_path = Path(identity).expanduser().resolve()
        if not key_path.is_file():
            raise ValueError("identity_file does not exist: " + str(key_path))
        identity = str(key_path)
    target = {"host": host, "user": user, "port": port, "identity_file": identity}
    for key in REQUIRED_PATHS:
        target[key] = _remote_path(mapping.get(key), key, require_paths)
    for key in ("yolov5_model", "yolo11_model"):
        target[key] = _remote_path(mapping.get(key), key)
    executable = mapping.get("executable") or "autoaim_node"
    if not isinstance(executable, str) or any(c in executable for c in "\x00\r\n\\"):
        raise ValueError("executable must be an absolute or build-relative POSIX path")
    executable_path = PurePosixPath(executable)
    if not executable_path.is_absolute() and (".." in executable_path.parts or executable.startswith("-")):
        raise ValueError("relative executable must stay inside build_dir")
    if str(executable_path) == ".":
        raise ValueError("executable must name a program")
    target["executable"] = str(executable_path)
    return target


def build_ssh_argv(target):
    """Build an argv array. Only the fixed Python bootstrap is shell-quoted."""
    target = validate_target(target)
    argv = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "ConnectTimeout=10", "-p", str(target["port"])]
    if target["identity_file"]:
        argv.extend(["-i", target["identity_file"]])
    argv.extend(["--", target["user"] + "@" + target["host"],
                 shlex.join(["python3", "-u", "-c", BOOTSTRAP])])
    return argv


def _save_report(path, report):
    temporary = path.with_name(path.name + "." + str(os.getpid()) + ".tmp")
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def execute_request(action, request_path):
    """Execute one request and return its durable remote/transport report.

    Remote events are framed separately from program output. A program cannot
    forge a report by printing NUC_REPORT or NUC_EVENT in its own log.
    """
    request_path = Path(request_path).resolve()
    report_path = request_path.parent / ("nuc_cancel_report.json" if action == "cancel" else "nuc_report.json")
    request = {}
    process = None
    report = None
    try:
        if action not in ("run", "probe", "cancel"):
            raise ValueError("Unknown NUC action: " + action)
        request = json.loads(request_path.read_text(encoding="utf-8"))
        if not isinstance(request, dict):
            raise ValueError("NUC request must be an object")
        request["target"] = validate_target(request.get("target"), require_paths=action == "run")
        if action == "cancel" and not request["target"]["workspace_dir"]:
            raise ValueError("workspace_dir is required for cancelling a NUC job")
        source = Path(__file__).with_name("remote_runner.py").read_text(encoding="utf-8")
        payload = json.dumps({"action": action, "request": request, "source": source},
                             ensure_ascii=False, allow_nan=False)
        argv = build_ssh_argv(request["target"])
        process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        process.stdin.write(payload)
        process.stdin.close()
        for line in process.stdout:
            if line.startswith(EVENT_PREFIX):
                try:
                    event = json.loads(line[len(EVENT_PREFIX):])
                except (ValueError, TypeError):
                    print(line, end="", flush=True)
                    continue
                if event.get("event") == "log":
                    print(event.get("text", ""), end="", flush=True)
                elif event.get("event") == "report":
                    candidate = event.get("report")
                    if (isinstance(candidate, dict) and candidate.get("action") == action
                            and candidate.get("job_id") == request.get("job_id")):
                        report = candidate
            else:
                print(line, end="", flush=True)
        exit_code = process.wait()
        if report is None:
            report = {"schema_version": 1, "action": action, "job_id": request.get("job_id"),
                      "status": "failed", "ready": False,
                      "error": "SSH ended without a remote report (exit " + str(exit_code) + ")"}
        report["ssh_exit_code"] = exit_code
        if exit_code and report.get("status") in ("succeeded", "ready"):
            report.update(status="failed", ready=False, error="SSH transport failed after the remote response")
    except (OSError, ValueError, TypeError, KeyboardInterrupt) as error:
        report = {"schema_version": 1, "action": action, "job_id": request.get("job_id"),
                  "status": "failed", "ready": False, "error": str(error) or "Local bridge interrupted"}
    finally:
        if process is not None and process.poll() is None:
            # Closing SSH closes the helper's output; its heartbeat/finally
            # terminates the remote child group. Explicit cancel uses remote PGID.
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if process is not None:
            for pipe in (process.stdin, process.stdout):
                if pipe is not None and not pipe.closed:
                    pipe.close()
        if report is not None:
            _save_report(report_path, report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run, inspect, or stop a NUC job through verified SSH")
    parser.add_argument("action", choices=("run", "probe", "cancel"))
    parser.add_argument("request", type=Path, help="JSON request; nuc_report.json (cancel: nuc_cancel_report.json) is saved alongside it")
    args = parser.parse_args(argv)
    report = execute_request(args.action, args.request)
    print(REPORT_PREFIX + json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)
    if args.action == "probe":
        return 0 if report.get("ssh_exit_code") == 0 else 1
    return 0 if report.get("status") in ("succeeded", "cancelled") else 1


if __name__ == "__main__":
    sys.exit(main())
